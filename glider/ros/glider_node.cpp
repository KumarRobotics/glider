/*
*
* ROS node logic
*/

#include "ros/glider_node.hpp"

using namespace GliderROS;

GliderNode::GliderNode(const rclcpp::NodeOptions& options) : rclcpp::Node("glider_node", options)
{

    declare_parameter("rate", 10.0);  // default freq
    declare_parameter("path", "");
    declare_parameter("use_odom", false);
    declare_parameter("publish_nav_sat_fix", false);
    declare_parameter("utm_zone", "18S");
    declare_parameter("declination", 12.0);

    // Get parameters
    double freq = this->get_parameter("rate").as_double();
    RCLCPP_INFO_STREAM(this->get_logger(), "Using prediction rate: " << freq);

    std::string path = this->get_parameter("path").as_string();
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Loading graph params from: " << path);

    // For use_sim_time, it's typically handled automatically in ROS2, but if you need it:
    use_sim_time_ = this->get_clock()->get_clock_type() == RCL_ROS_TIME;

    bool use_odom = this->get_parameter("use_odom").as_bool();
    RCLCPP_INFO_STREAM(this->get_logger(), "Fusing Odometry: " << std::boolalpha << use_odom);
    publish_nsf_ = this->get_parameter("publish_nav_sat_fix").as_bool();
    utm_zone_ = this->get_parameter("utm_zone").as_string();
    declination_ = this->get_parameter("declination").as_double() * M_PI / 180.0d;
    RCLCPP_INFO_STREAM(this->get_logger(), "Using Magnetic Declination: " << declination_);

    glider_ = std::make_unique<Glider::Glider>(path);

    imu_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    gps_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // Create subscribers with callback groups
    auto imu_sub_options = rclcpp::SubscriptionOptions();
    imu_sub_options.callback_group = imu_group_;
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("/imu", 20, 
                                                                std::bind(&GliderNode::imuCallback, this, std::placeholders::_1),
                                                                imu_sub_options);
    mag_sub_ = this->create_subscription<sensor_msgs::msg::MagneticField>("/mag", 20, 
                                                                    std::bind(&GliderNode::magCallback, this, std::placeholders::_1), 
                                                                    imu_sub_options);
    
    auto gps_sub_options = rclcpp::SubscriptionOptions();
    gps_sub_options.callback_group = gps_group_;
    gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>("/gps", 1, 
                                                                      std::bind(&GliderNode::gpsCallback, this, std::placeholders::_1),
                                                                      gps_sub_options);

    current_state_ = Glider::State::Uninitialized();

    if (use_odom)
    {
        auto odom_sub_options = rclcpp::SubscriptionOptions();
        odom_sub_options.callback_group = gps_group_;
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 1,
                                                                       std::bind(&GliderNode::odomCallback, this, std::placeholders::_1),
                                                                       odom_sub_options);
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/pose", 1,
                                                                               std::bind(&GliderNode::poseCallback, this, std::placeholders::_1),
                                                                               odom_sub_options);
    }

    if (publish_nsf_)
    {
        RCLCPP_INFO(this->get_logger(), "Publishing NavSatFix msg on /glider/fix");
        gps_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/glider/fix", 10);
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Publishing Odometry msg on /glider/odom");
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/glider/odom", 10);
    }

    std::chrono::milliseconds d = GliderROS::Conversions::hzToDuration(freq);
    timer_ = this->create_wall_timer(d, std::bind(&GliderNode::interpolationCallback, this));
    RCLCPP_INFO(this->get_logger(), "GliderNode Initialized");
}

int64_t GliderNode::getTime(const builtin_interfaces::msg::Time& stamp) const
{
   return (static_cast<int64_t>(stamp.sec) * 1000000000LL) + static_cast<int64_t>(stamp.nanosec);
}

void GliderNode::interpolationCallback()
{
    if (!initialized_ || !current_state_.isInitialized()) return;
    
    int64_t timestamp = getTime(this->now());
    Glider::Odometry odom = glider_->interpolate(timestamp);
    
    if (!odom.isInitialized()) return;
    if (publish_nsf_)
    {
        sensor_msgs::msg::NavSatFix msg = GliderROS::Conversions::odomToRos<sensor_msgs::msg::NavSatFix>(odom);
        // TODO type addCovaraince
        //GliderROS::Conversions::addCovariance<sensor_msgs::msg::NavSatFix>(current_state_, msg);
        gps_pub_->publish(msg);
    }
    else
    {
        nav_msgs::msg::Odometry msg = GliderROS::Conversions::odomToRos<nav_msgs::msg::Odometry>(odom);
        GliderROS::Conversions::addCovariance<nav_msgs::msg::Odometry>(current_state_, msg);
        odom_pub_->publish(msg);
    }
}

void GliderNode::imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
    Eigen::Vector3d gyro = GliderROS::Conversions::rosToEigen<Eigen::Vector3d>(msg->angular_velocity);
    Eigen::Vector3d accel = GliderROS::Conversions::rosToEigen<Eigen::Vector3d>(msg->linear_acceleration);
    Eigen::Vector4d orient = GliderROS::Conversions::rosToEigen<Eigen::Vector4d>(msg->orientation);
    int64_t timestamp = getTime(msg->header.stamp);

    glider_->addIMU(timestamp, accel, gyro, orient);
}

void GliderNode::magCallback(const sensor_msgs::msg::MagneticField::ConstSharedPtr msg)
{
    int64_t timestamp = getTime(msg->header.stamp);
    double heading = std::atan2(msg->magnetic_field.x, msg->magnetic_field.y) + declination_;
    glider_->addMagnetometer(timestamp, heading);
}

void GliderNode::gpsCallback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)
{
    Eigen::Vector3d gps = GliderROS::Conversions::rosToEigen<Eigen::Vector3d>(*msg);

    int64_t timestamp = getTime(msg->header.stamp);

    glider_->addGPS(timestamp, gps);

    current_state_ = glider_->optimize();
    if (!initialized_) initialized_ = true;
}

void GliderNode::odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
    Eigen::Isometry3d pose = GliderROS::Conversions::rosToEigen<Eigen::Isometry3d>(*msg);
    int64_t timestamp = getTime(msg->header.stamp);
    glider_->addOdom(timestamp, pose);
}

void GliderNode::poseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
{
    Eigen::Isometry3d pose = GliderROS::Conversions::rosToEigen<Eigen::Isometry3d>(*msg);
    int64_t timestamp = getTime(msg->header.stamp);
    glider_->addOdom(timestamp, pose);
}

RCLCPP_COMPONENTS_REGISTER_NODE(GliderROS::GliderNode)
