/*
*
* ROS node logic
*/

#include "glider/ros/factor_node.hpp"
#include "glider/ros/node_utils.hpp"

using namespace glider;

FactorManagerNode::FactorManagerNode(const rclcpp::NodeOptions& options) : rclcpp::Node("glider_node", options)
{

    const std::map<std::string, double>& config = { };

    factor_manager_ = glider::FactorManager(config);
    imu_msgs_ = 0;

    imu_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    gps_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/glider/odom", 1);
    
    // Create subscribers with callback groups
    auto imu_sub_options = rclcpp::SubscriptionOptions();
    imu_sub_options.callback_group = imu_group_;
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu", 20, 
        std::bind(&FactorManagerNode::imuCallback, this, std::placeholders::_1),
        imu_sub_options);
    
    auto gps_sub_options = rclcpp::SubscriptionOptions();
    gps_sub_options.callback_group = gps_group_;
    gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/gps", 1, 
        std::bind(&FactorManagerNode::gpsCallback, this, std::placeholders::_1),
        gps_sub_options);

    auto odom_sub_options = rclcpp::SubscriptionOptions();
    odom_sub_options.callback_group = gps_group_;
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 1, 
                                                                   std::bind(&FactorManagerNode::odomCallback, this, std::placeholders::_1),
                                                                   odom_sub_options);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&FactorManagerNode::predictCallback, this));

    RCLCPP_INFO(this->get_logger(), "Factor Graph Node Initialized...");
}

int64_t FactorManagerNode::getTime(const builtin_interfaces::msg::Time& stamp) const 
{
    return (static_cast<int64_t>(stamp.sec) * 1000000000LL) + static_cast<int64_t>(stamp.nanosec);
}

void FactorManagerNode::predictCallback()
{
    if (!initialized_) return;
    int64_t elapsed_time = getTime(this->now()) - last_time_now_;
    int64_t timestamp = last_imu_stamp_ + elapsed_time;
    auto [translation, quaternion] = factor_manager_.predict(timestamp);

    if (translation.norm() > 0) 
    {  
        pose_ = translation;
        orientation_ = quaternion;
        
        // Publish in local frame
        auto odom_msg = std::make_unique<nav_msgs::msg::Odometry>();
        
        odom_msg->pose.pose.position.x = pose_(0);//-482963.5387620803;
        odom_msg->pose.pose.position.y = pose_(1);//-4421274.811364001;
        odom_msg->pose.pose.position.z = pose_(2);//pose_(2);
    
        odom_msg->pose.pose.orientation.w = orientation_(0);
        odom_msg->pose.pose.orientation.x = orientation_(1);
        odom_msg->pose.pose.orientation.y = orientation_(2);
        odom_msg->pose.pose.orientation.z = orientation_(3);
        
        odom_msg->header.stamp = this->get_clock()->now();
        odom_msg->header.frame_id = "enu";
        
        odom_pub_->publish(std::move(odom_msg));
    }
}

void FactorManagerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
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

    Eigen::Vector3d between_position = (current_position - prev_position_).array().abs();
    glider::Quaternion between_quat = current_quat - prev_quat_;

    int64_t timestamp;
    if (use_sim_time_)
    {
        timestamp = getTime(this->get_clock()->now());
    }
    else
    {
        timestamp = getTime(msg->header.stamp);
    }

    factor_manager_.addOdometryFactor(timestamp, between_position, between_quat.toEigen());

    prev_position_ = current_position;
    prev_quat_ = current_quat;
}

void FactorManagerNode::gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) 
{
    // GPS output from VectorNAV
    if (!initialized_)
    {
        initial_alt_ = msg->altitude;
    }
    Eigen::Vector3d gps_factor;
    
    gps_factor(0) = msg->latitude;
    gps_factor(1) = msg->longitude;
    gps_factor(2) = msg->altitude - initial_alt_;
    
    int64_t timestamp = getTime(msg->header.stamp);
    
    factor_manager_.addGpsFactor(timestamp, gps_factor);
     
    auto [translation, quaternion, rotation] = factor_manager_.runner();
    initialized_ = true; 
}

void FactorManagerNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) 
{
    // Callback for vector nav imu
    imu_msgs_ ++; 
    Eigen::Vector3d gyro(
        msg->angular_velocity.x,
        msg->angular_velocity.y,
        msg->angular_velocity.z
    );
    
    Eigen::Vector3d accel(
        msg->linear_acceleration.x,
        msg->linear_acceleration.y,
        msg->linear_acceleration.z
    );
    
    Eigen::Vector4d quat(
        msg->orientation.w,
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z
    );
    last_imu_ros_stamp_ = msg->header.stamp; 
    int64_t timestamp = getTime(msg->header.stamp);
    last_imu_stamp_ = timestamp;
    last_time_now_ = getTime(this->now());

    factor_manager_.addImuFactor(timestamp, accel, gyro, quat);
}

Eigen::Vector4d FactorManagerNode::getOrientation() const 
{
    return orientation_;
}

Eigen::Vector3d FactorManagerNode::getPose() const {
    return pose_;
}

RCLCPP_COMPONENTS_REGISTER_NODE(glider::FactorManagerNode)
