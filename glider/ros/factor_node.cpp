/*
*
* ROS node logic
*/

#include "glider/ros/factor_node.hpp"
#include "glider/ros/node_utils.hpp"

using namespace glider;

FactorNode::FactorNode(const rclcpp::NodeOptions& options) : rclcpp::Node("glider_node", options)
{
    ned_to_enu_ = (Eigen::Matrix3d() << 
                 0.0, 1.0, 0.0,
                 1.0, 0.0, 0.0,
                 0.0, 0.0, -1.0).finished();

    this->declare_parameter("rate", 10.0);  // default freq
    this->declare_parameter("path", "");
    this->declare_parameter("use_odom", false);
    this->declare_parameter("publish_nav_sat_fix", false);
    this->declare_parameter("utm_zone", "18S");

    // Get parameters
    double freq = this->get_parameter("rate").as_double();
    RCLCPP_INFO_STREAM(this->get_logger(), "Using prediction rate: " << freq);

    std::string path = this->get_parameter("path").as_string();
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Loading graph params from: " << path);

    // For use_sim_time, it's typically handled automatically in ROS2, but if you need it:
    use_sim_time_ = this->get_clock()->get_clock_type() == RCL_ROS_TIME;

    bool use_odom = this->get_parameter("use_odom").as_bool();
    publish_nsf_ = this->get_parameter("publish_nav_sat_fix").as_bool();
    utm_zone_ = this->get_parameter("utm_zone").as_string();

    imu_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    gps_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // Create subscribers with callback groups
    auto imu_sub_options = rclcpp::SubscriptionOptions();
    imu_sub_options.callback_group = imu_group_;
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu", 20, 
        std::bind(&FactorNode::imuCallback, this, std::placeholders::_1),
        imu_sub_options);
    
    auto gps_sub_options = rclcpp::SubscriptionOptions();
    gps_sub_options.callback_group = gps_group_;
    gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/gps", 1, 
        std::bind(&FactorNode::gpsCallback, this, std::placeholders::_1),
        gps_sub_options);

    current_state_ = glider::State::Uninitialized();

    std::map<std::string, double> config;
    config = params.load<double>(path);
    factor_manager_ = glider::FactorManager(config);

    if (use_odom)
    {
        auto odom_sub_options = rclcpp::SubscriptionOptions();
        odom_sub_options.callback_group = gps_group_;
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 1,
                                                                       std::bind(&FactorNode::odomCallback, this, std::placeholders::_1),
                                                                       odom_sub_options);
    }

    if (publish_nsf_)
    {
        RCLCPP_INFO(this->get_logger(), "[GLIDER] Publishing NavSatFix msg on /glider/odom");
        gps_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/glider/fix", 10);
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "[GLIDER] Publishing Odometry msg on /glider/odom");
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/glider/odom", 10);
    }

    std::chrono::milliseconds d = rosutil::hzToDuration(freq);
    timer_ = this->create_wall_timer(d, std::bind(&FactorNode::interpolationCallback, this));
    RCLCPP_INFO(this->get_logger(), "[GLIDER] Factor Manager Node Initialized");
}

int64_t FactorNode::getTime(const builtin_interfaces::msg::Time& stamp) const
{
   return (static_cast<int64_t>(stamp.sec) * 1000000000LL) + static_cast<int64_t>(stamp.nanosec);
}

void FactorNode::interpolationCallback()
{
    if (!initialized_ || !current_state_.isInitialized()) return;
    
    int64_t timestamp = getTime(this->now());
    glider::Odometry odom = factor_manager_.predict(timestamp);
    
    if (!odom.isInitialized()) return;
    if (publish_nsf_)
    {
        sensor_msgs::msg::NavSatFix gps_msg = rosutil::toRosMsg<sensor_msgs::msg::NavSatFix>(odom, utm_zone_.c_str());
        rosutil::addCovariance<sensor_msgs::msg::NavSatFix>(current_state_, gps_msg);
        gps_pub_->publish(gps_msg);
    }
    else
    {
        nav_msgs::msg::Odometry odom_msg = rosutil::toRosMsg<nav_msgs::msg::Odometry>(odom);
        rosutil::addCovariance<nav_msgs::msg::Odometry>(current_state_, odom_msg);
        odom_pub_->publish(odom_msg);
    }
}

void FactorNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    if (!initialized_)
    {
        prev_position_ << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
        prev_quat_ = glider::Quaternion(msg->pose.pose.orientation);
    }
    // calculate the difference between current and prev
    Eigen::Vector3d current_position(msg->pose.pose.position.x,
                                     msg->pose.pose.position.y,
                                     msg->pose.pose.position.z);
    glider::Quaternion current_quat(msg->pose.pose.orientation);

    Eigen::Vector3d between_position = (current_position - prev_position_);
    glider::Quaternion between_quat = current_quat - prev_quat_;

    int64_t timestamp;
    if (use_sim_time_)
    {
        timestamp = getTime(this->now());
    }
    else
    {
        timestamp = getTime(msg->header.stamp);
    }

    factor_manager_.addOdometryFactor(timestamp, between_position, between_quat.toEigen());

    prev_position_ = current_position;
    prev_quat_ = current_quat;
}

void FactorNode::gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{

    Eigen::Vector3d gps_factor;
    gps_factor(0) = msg->latitude;
    gps_factor(1) = msg->longitude;
    gps_factor(2) = msg->altitude;
   
    int64_t timestamp;
    if (use_sim_time_)
    {
        timestamp = getTime(this->now());
    }
    else
    {
        timestamp = getTime(msg->header.stamp);
    }
    factor_manager_.addGpsFactor(timestamp, gps_factor);
    
    current_state_ = factor_manager_.runner();
    if (!initialized_) initialized_ = true;
}

void FactorNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    if (!initialized_) return;

    Eigen::Vector3d gyro(msg->angular_velocity.x,
                         msg->angular_velocity.y,
                         msg->angular_velocity.z);
    
    Eigen::Vector3d accel(msg->linear_acceleration.x,
                          msg->linear_acceleration.y,
                          msg->linear_acceleration.z);
    
    Eigen::Quaterniond quat(msg->orientation.w,
                            msg->orientation.x,
                            msg->orientation.y,
                            msg->orientation.z);

    Eigen::Matrix3d rot_enu = ned_to_enu_ * quat.toRotationMatrix() * ned_to_enu_.transpose();
    quat = Eigen::Quaterniond(rot_enu);
    quat.normalize();

    int64_t timestamp;
    if (use_sim_time_)
    {
        timestamp = getTime(this->now());
    }
    else
    {
        timestamp = getTime(msg->header.stamp);
    }
    Eigen::Vector4d q_vec(quat.w(), quat.x(), quat.y(), quat.z());
    factor_manager_.addImuFactor(timestamp, accel, gyro, q_vec);
}

RCLCPP_COMPONENTS_REGISTER_NODE(glider::FactorNode)
