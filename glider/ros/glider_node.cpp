/*
*
* ROS node logic
*/

#include "ros/glider_node.hpp"

using namespace GliderROS;

GliderNode::GliderNode(const rclcpp::NodeOptions& options) : rclcpp::Node("glider_node", options)
{
    // Declare ROS parameters
    declare_parameter("publishers.rate", 10.0);
    declare_parameter("publishers.nav_sat_fix", false);
    declare_parameter("publishers.viz.use", false);
    declare_parameter("publishers.viz.origin_easting", 0.0);
    declare_parameter("publishers.viz.origin_northing", 0.0);

    declare_parameter("subscribers.use_odom", false);

    declare_parameter("path", "");

    // Get parameters
    freq_ = this->get_parameter("publishers.rate").as_double();

    publish_nsf_ = this->get_parameter("publishers.nav_sat_fix").as_bool();
    viz_ = this->get_parameter("publishers.viz.use").as_bool();
    origin_easting_ = this->get_parameter("publishers.viz.origin_easting").as_double();
    origin_northing_ = this->get_parameter("publishers.viz.origin_northing").as_double();

    bool use_odom = this->get_parameter("subscribers.use_odom").as_bool();
    
    std::string path = this->get_parameter("path").as_string();

    glider_ = std::make_unique<Glider::Glider>(path);
    current_state_ = Glider::OdometryWithCovariance::Uninitialized();

    imu_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    gps_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // Create subscribers with callback groups
    auto imu_sub_options = rclcpp::SubscriptionOptions();
    imu_sub_options.callback_group = imu_group_;
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("/imu", 20, 
                                                                std::bind(&GliderNode::imuCallback, this, std::placeholders::_1),
                                                                imu_sub_options);
    
    auto gps_sub_options = rclcpp::SubscriptionOptions();
    gps_sub_options.callback_group = gps_group_;
    gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>("/gps", 1, 
                                                                      std::bind(&GliderNode::gpsCallback, this, std::placeholders::_1),
                                                                      gps_sub_options);

    dgps_sub_ = this->create_subscription<gps_msgs::msg::GPSFix>("/dgps", 1,
                                                                 std::bind(&GliderNode::dgpsCallback, this, std::placeholders::_1),
                                                                 gps_sub_options);

    auto odom_sub_options = rclcpp::SubscriptionOptions();
    odom_sub_options.callback_group = gps_group_;
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 1,
                                                                   std::bind(&GliderNode::odomCallback, this, std::placeholders::_1),
                                                                   odom_sub_options);

    if (publish_nsf_)
    {
        LOG(INFO) << "[GLIDER] Publishing NavSatFix msg on /glider/fix";
        LOG(INFO) << "[GLIDER] Using prediction rate: " << freq_;
        gps_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/glider/fix", 10);
    }
    else
    {
        LOG(INFO) << "[GLIDER] Publishing Odometry msg on /glider/odom";
        LOG(INFO) << "[GLIDER] Using prediction rate: " << freq_;
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/glider/odom", 10);
    }

    if(viz_)
    {
        LOG(INFO) << "[GLIDER] Publishing Odometry Viz message on /glider/odom/viz";
        odom_viz_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/glider/odom/viz", 10);
    }

    if (freq_ > 0)
    {
        std::chrono::milliseconds d = GliderROS::Conversions::hzToDuration(freq_);
        timer_ = this->create_wall_timer(d, std::bind(&GliderNode::interpolationCallback, this));
    }
    LOG(INFO) << "[GLIDER] GliderNode Initialized";
}

int64_t GliderNode::getTime(const builtin_interfaces::msg::Time& stamp) const
{
   return (static_cast<int64_t>(stamp.sec) * 1000000000LL) + static_cast<int64_t>(stamp.nanosec);
}

void GliderNode::interpolationCallback()
{
    // if the state is not initialized we cannot interpolate
    if (!current_state_.isInitialized()) return;
    int64_t timestamp = getTime(this->now());
    Glider::Odometry odom = glider_->interpolate(timestamp);

    (publish_nsf_) ? publishNavSatFix(odom) : publishOdometry(odom);
}

void GliderNode::imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
    LOG_FIRST_N(INFO, 1) << "[GLIDER] Recieved IMU measurement";
    Eigen::Vector3d gyro = GliderROS::Conversions::rosToEigen<Eigen::Vector3d>(msg->angular_velocity);
    Eigen::Vector3d accel = GliderROS::Conversions::rosToEigen<Eigen::Vector3d>(msg->linear_acceleration);
    Eigen::Vector4d orient = GliderROS::Conversions::rosToEigen<Eigen::Vector4d>(msg->orientation);
    int64_t timestamp = getTime(msg->header.stamp);

    glider_->addImu(timestamp, accel, gyro, orient);

    if (freq_ == 0 && current_state_.isInitialized())
    {
        Glider::Odometry odom = glider_->interpolate(timestamp);
        (publish_nsf_) ? publishNavSatFix(odom) : publishOdometry(odom);
    }
}

void GliderNode::dgpsCallback(const gps_msgs::msg::GPSFix::ConstSharedPtr msg)
{
    LOG_FIRST_N(INFO, 1) << "[GLIDER] Received DGPS measurement";
    std::pair<Eigen::Vector3d, Eigen::Vector2d> dgps = GliderROS::Conversions::rosToEigen<std::pair<Eigen::Vector3d, Eigen::Vector2d>>(*msg);
    int64_t timestamp = getTime(msg->header.stamp);
    
    //glider_->addGpsWithHeading(timestamp, dgps.first, dgps.second); 
    heading_ = dgps.second;
    //current_state_ = glider_->optimize(timestamp);
}

void GliderNode::gpsCallback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)
{
    LOG_FIRST_N(INFO, 1) << "[GLIDER] Recieved GPS measurement";
    Eigen::Vector3d gps = GliderROS::Conversions::rosToEigen<Eigen::Vector3d>(*msg);
    if (!init_alt_) {
        initial_alt_ = gps.z();
        gps = Eigen::Vector3d(gps.x(), gps.y(), 0.0);
        init_alt_ = true;
    } else {
        double alt = gps.z() - initial_alt_;
        gps = Eigen::Vector3d(gps.x(), gps.y(), alt);
    }
    int64_t timestamp = getTime(msg->header.stamp);

    glider_->addGpsWithHeading(timestamp, gps, heading_);

    current_state_ = glider_->optimize(timestamp);
}

void GliderNode::odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
    // TODO
    //Eigen::Isometry3d pose = GliderROS::Conversions::rosToEigen<Eigen::Isometry3d>(*msg);
    //int64_t timestamp = getTime(msg->header.stamp);
    //glider_->addOdom(timestamp, pose);
}

void GliderNode::publishOdometry(Glider::OdometryWithCovariance& state) const
{
    LOG_FIRST_N(INFO, 1) << "[GLIDER] Publishing Odometry from optimzation";
    nav_msgs::msg::Odometry msg = GliderROS::Conversions::odomToRos<nav_msgs::msg::Odometry>(state);
    odom_pub_->publish(msg);
    
    if (viz_) publishOdometryViz(msg);
}

void GliderNode::publishOdometry(Glider::Odometry& odom) const
{
    LOG_FIRST_N(INFO, 1) << "[GLIDER] Publishing Odometry from prediction";
    nav_msgs::msg::Odometry msg = GliderROS::Conversions::odomToRos<nav_msgs::msg::Odometry>(odom);
    GliderROS::Conversions::addCovariance<nav_msgs::msg::Odometry>(current_state_, msg);
    odom_pub_->publish(msg);

    if (viz_) publishOdometryViz(msg);
}

void GliderNode::publishNavSatFix(Glider::OdometryWithCovariance& state) const
{
    // TODO add covariance
    LOG_FIRST_N(INFO, 1) << "[GLIDER] Publishing NavSatFix from optimization";
    sensor_msgs::msg::NavSatFix msg = GliderROS::Conversions::odomToRos<sensor_msgs::msg::NavSatFix>(state);
    gps_pub_->publish(msg);
}

void GliderNode::publishNavSatFix(Glider::Odometry& odom) const
{
    // TODO add covariance
    LOG_FIRST_N(INFO, 1) << "[GLIDER] Publishing NavSatFix from prediction";
    sensor_msgs::msg::NavSatFix msg = GliderROS::Conversions::odomToRos<sensor_msgs::msg::NavSatFix>(odom);
    gps_pub_->publish(msg);
}

void GliderNode::publishOdometryViz(nav_msgs::msg::Odometry viz_msg) const
{ 
    double x = viz_msg.pose.pose.position.x - origin_easting_;
    double y = viz_msg.pose.pose.position.y - origin_northing_;
    viz_msg.pose.pose.position.x = x;
    viz_msg.pose.pose.position.y = y;

    odom_viz_pub_->publish(viz_msg);
}

RCLCPP_COMPONENTS_REGISTER_NODE(GliderROS::GliderNode)
