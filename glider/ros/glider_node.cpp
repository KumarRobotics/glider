/*
*
* ROS node logic
*/

#include "ros/glider_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>

using namespace GliderROS;

GliderNode::GliderNode(const rclcpp::NodeOptions& options) : rclcpp::Node("glider_node", options)
{
    // Declare ROS parameters
    declare_parameter("publishers.rate", 10.0);
    declare_parameter("publishers.nav_sat_fix", false);
    declare_parameter("publishers.viz.use", false);
    declare_parameter("publishers.viz.origin_easting", 0.0);
    declare_parameter("publishers.viz.origin_northing", 0.0);
    declare_parameter("publishers.utm_zone", "14R");
    declare_parameter("publishers.map_frame", "map");
    declare_parameter("publishers.base_link_frame", "base_link");

    declare_parameter("subscribers.dgps_topic", "/dgps");
    declare_parameter("subscribers.gps_topic", "/gps");
    declare_parameter("subscribers.imu_topic", "/imu");
    declare_parameter("subscribers.odom_topic", "/odom");
    declare_parameter("subscribers.use_gps", true);
    declare_parameter("subscribers.use_dgps", true);
    declare_parameter("subscribers.use_odom", false);
    declare_parameter("subscribers.gps_rejection_variance", 100.0);
    declare_parameter("subscribers.max_stamp_skew_sec", 1.0);
    declare_parameter("subscribers.gps_loss_timeout_sec", 3.0);

    declare_parameter("path", "");

    // Get parameters
    freq_ = this->get_parameter("publishers.rate").as_double();

    publish_nsf_ = this->get_parameter("publishers.nav_sat_fix").as_bool();
    viz_ = this->get_parameter("publishers.viz.use").as_bool();
    origin_easting_ = this->get_parameter("publishers.viz.origin_easting").as_double();
    origin_northing_ = this->get_parameter("publishers.viz.origin_northing").as_double();
    map_frame_ = this->get_parameter("publishers.map_frame").as_string();
    base_link_frame_ = this->get_parameter("publishers.base_link_frame").as_string();

    use_odom_ = this->get_parameter("subscribers.use_odom").as_bool();
    use_gps_ = this->get_parameter("subscribers.use_gps").as_bool();
    use_dgps_ = this->get_parameter("subscribers.use_dgps").as_bool();
    gps_rejection_variance_ = this->get_parameter("subscribers.gps_rejection_variance").as_double();
    max_stamp_skew_sec_ = this->get_parameter("subscribers.max_stamp_skew_sec").as_double();
    gps_loss_timeout_sec_ = this->get_parameter("subscribers.gps_loss_timeout_sec").as_double();

    std::string path = this->get_parameter("path").as_string();

    glider_ = std::make_unique<Glider::Glider>(path);
    utm_zone_ = this->get_parameter("publishers.utm_zone").as_string();

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    current_state_ = Glider::OdometryWithCovariance::Uninitialized();

    imu_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    gps_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // Create subscribers with callback groups
    auto imu_sub_options = rclcpp::SubscriptionOptions();
    imu_sub_options.callback_group = imu_group_;
    auto imu_topic = this->get_parameter("subscribers.imu_topic").as_string();
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic, rclcpp::SensorDataQoS(),
                                                                std::bind(&GliderNode::imuCallback, this, std::placeholders::_1),
                                                                imu_sub_options);

    auto gps_sub_options = rclcpp::SubscriptionOptions();
    gps_sub_options.callback_group = gps_group_;
    auto gps_topic = this->get_parameter("subscribers.gps_topic").as_string();
    gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(gps_topic, rclcpp::SensorDataQoS(),
                                                                      std::bind(&GliderNode::gpsCallback, this, std::placeholders::_1),
                                                                      gps_sub_options);

    auto dgps_topic = this->get_parameter("subscribers.dgps_topic").as_string();
    dgps_sub_ = this->create_subscription<dgps_msgs::msg::DifferentialNavSatFix>(dgps_topic, rclcpp::SensorDataQoS(),
                                                                 std::bind(&GliderNode::dgpsCallback, this, std::placeholders::_1),
                                                                 gps_sub_options);

    gps_goal_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>("/glider/gps_goal", 1,
                                                                               std::bind(&GliderNode::gpsGoalCallback, this, std::placeholders::_1));

    auto odom_sub_options = rclcpp::SubscriptionOptions();
    odom_sub_options.callback_group = gps_group_;
    auto odom_topic = this->get_parameter("subscribers.odom_topic").as_string();
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(odom_topic, rclcpp::SensorDataQoS(),
                                                                   std::bind(&GliderNode::odomCallback, this, std::placeholders::_1),
                                                                   odom_sub_options);

    LOG(INFO) << "[GLIDER] Publishing Odometry msg on /glider/odom";
    LOG(INFO) << "[GLIDER] Using prediction rate: " << freq_;
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/glider/odom", 10);

    if (publish_nsf_)
    {
        LOG(INFO) << "[GLIDER] Publishing NavSatFix msg on /glider/fix";
        gps_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/glider/fix", 10);
    }

    if(viz_)
    {
        LOG(INFO) << "[GLIDER] Publishing Odometry Viz message on /glider/odom/viz";
        odom_viz_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/glider/odom/viz", 10);
    }

    LOG(INFO) << "[GLIDER] Publishing Goal Pose on /goal_pose";
    goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);

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

int64_t GliderNode::getSynchronizedTime(const builtin_interfaces::msg::Time& stamp, const char* source,
                                        std::optional<int64_t>& clock_offset)
{
    const int64_t message_time = getTime(stamp);
    const int64_t ros_time = getTime(this->now());

    // Recorded header stamps already use the simulated clock timeline.
    if (this->get_parameter("use_sim_time").as_bool())
        return message_time > 0 ? message_time : ros_time;

    const int64_t max_skew = static_cast<int64_t>(max_stamp_skew_sec_ * 1e9);
    if (message_time <= 0) return ros_time;

    if (!clock_offset.has_value())
    {
        clock_offset = std::llabs(message_time - ros_time) > max_skew ? ros_time - message_time : 0;
        if (*clock_offset != 0)
            LOG(INFO) << "[GLIDER] Aligning " << source << " clock to the active ROS clock";
    }

    int64_t synchronized_time = message_time + *clock_offset;
    if (std::llabs(synchronized_time - ros_time) > max_skew)
    {
        *clock_offset = ros_time - message_time;
        synchronized_time = ros_time;
        LOG_FIRST_N(WARNING, 5) << "[GLIDER] " << source << " clock jumped; realigning";
    }
    return synchronized_time;
}

void GliderNode::updateEnvironmentState(int64_t timestamp)
{
    if (environment_state_ != EnvironmentState::Outdoor || !last_accepted_gps_time_) return;
    const double elapsed = static_cast<double>(timestamp - *last_accepted_gps_time_) / 1e9;
    if (elapsed >= gps_loss_timeout_sec_)
    {
        environment_state_ = EnvironmentState::Indoor;
        LOG(INFO) << "[GLIDER] Outdoor → Indoor";
    }
}

void GliderNode::markGpsAccepted(int64_t timestamp)
{
    if (environment_state_ == EnvironmentState::Indoor)
        LOG(INFO) << "[GLIDER] Indoor → Outdoor";
    environment_state_ = EnvironmentState::Outdoor;
    last_accepted_gps_time_ = timestamp;
}

void GliderNode::markGpsUnavailable()
{
    if (environment_state_ == EnvironmentState::Outdoor)
    {
        environment_state_ = EnvironmentState::Indoor;
        LOG(INFO) << "[GLIDER] Outdoor → Indoor";
    }
}

void GliderNode::interpolationCallback()
{
    // if the state is not initialized we cannot interpolate
    if (!current_state_.isInitialized()) return;
    int64_t timestamp = getTime(this->now());
    Glider::Odometry odom = glider_->interpolate(timestamp);

    publishOdometry(odom);
}

void GliderNode::imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
    LOG_FIRST_N(INFO, 1) << "[GLIDER] Recieved IMU measurement";
    Eigen::Vector3d gyro = GliderROS::Conversions::rosToEigen<Eigen::Vector3d>(msg->angular_velocity);
    Eigen::Vector3d accel = GliderROS::Conversions::rosToEigen<Eigen::Vector3d>(msg->linear_acceleration);
    Eigen::Vector4d orient = GliderROS::Conversions::rosToEigen<Eigen::Vector4d>(msg->orientation);
    if (!gyro.allFinite() || !accel.allFinite() || !orient.allFinite() || orient.norm() < 1e-6)
    {
        LOG_FIRST_N(WARNING, 10) << "[GLIDER] IMU ignored: non-finite data or invalid quaternion";
        return;
    }
    orient.normalize();
    int64_t timestamp = getSynchronizedTime(msg->header.stamp, "IMU", imu_clock_offset_);
    updateEnvironmentState(timestamp);

    glider_->addImu(timestamp, accel, gyro, orient);

    if (freq_ == 0 && current_state_.isInitialized())
    {
        Glider::Odometry odom = glider_->interpolate(timestamp);
        publishOdometry(odom);
    }
}

void GliderNode::dgpsCallback(const dgps_msgs::msg::DifferentialNavSatFix::ConstSharedPtr msg)
{
    if (!use_dgps_) return;
    const auto& fix = msg->nmea;
    const double variance = std::max({fix.position_covariance[0], fix.position_covariance[4],
                                      fix.position_covariance[8]});
    if (fix.status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX ||
        !std::isfinite(variance) || variance < 1e-6)
    {
        markGpsUnavailable();
        LOG_FIRST_N(WARNING, 5) << "[GLIDER] DGPS ignored: No fix or invalid covariance";
        return;
    }

    if (variance > gps_rejection_variance_)
    {
        markGpsUnavailable();
        LOG_FIRST_N(INFO, 5) << "[GLIDER] DGPS rejected due to unsafe covariance (> " << gps_rejection_variance_ << ")";
        return;
    }
    LOG_FIRST_N(INFO, 1) << "[GLIDER] Received DGPS measurement";
    Eigen::Vector3d gps = GliderROS::Conversions::rosToEigen<Eigen::Vector3d>(fix);
    if (!gps.allFinite())
    {
        markGpsUnavailable();
        LOG_FIRST_N(WARNING, 10) << "[GLIDER] DGPS ignored: non-finite position";
        return;
    }
    int64_t timestamp = getSynchronizedTime(fix.header.stamp, "DGPS", dgps_clock_offset_);
    markGpsAccepted(timestamp);

    // Include unmodeled frame, lever-arm, and synchronization uncertainty.
    const double sigma = std::max(glider_->params().gps_noise, std::sqrt(variance));
    // The custom message stores ENU heading in radians and covariance in rad^2.
    const double heading_covariance = static_cast<double>(msg->heading_covariance);
    if (!std::isfinite(static_cast<double>(msg->heading)) ||
        !std::isfinite(heading_covariance) || heading_covariance < 0.0)
    {
        // Retain position-only fixes when heading is unavailable.
        LOG_FIRST_N(WARNING, 10) << "[GLIDER] DGPS heading unavailable; fusing position only";
        glider_->addGps(timestamp, gps, sigma);
        // LIO publishes the queued factor at the next odometry update.
        if (!use_odom_)
        {
            current_state_ = glider_->optimize(timestamp);
            if (publish_nsf_ && current_state_.isInitialized()) publishNavSatFix(current_state_);
        }
        return;
    }
    const double heading_sigma = std::max(0.1, std::sqrt(std::max(0.0, heading_covariance)));
    Eigen::Vector2d heading(static_cast<double>(msg->heading), heading_sigma);
    glider_->addGpsWithHeading(timestamp, gps, heading, sigma);

    if (!use_odom_)
    {
        current_state_ = glider_->optimize(timestamp);
        if (publish_nsf_ && current_state_.isInitialized()) publishNavSatFix(current_state_);
    }
}

void GliderNode::gpsCallback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)
{
    if (!use_gps_) return;
    if (msg->status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX || msg->position_covariance[0] < 1e-6)
    {
        markGpsUnavailable();
        LOG_FIRST_N(WARNING, 5) << "[GLIDER] GPS ignored: No fix or invalid covariance";
        return;
    }

    const double variance = std::max({msg->position_covariance[0], msg->position_covariance[4],
                                      msg->position_covariance[8]});
    if (!std::isfinite(variance) || variance > gps_rejection_variance_)
    {
        markGpsUnavailable();
        LOG_FIRST_N(INFO, 5) << "[GLIDER] GPS rejected due to unsafe covariance (> " << gps_rejection_variance_ << ")";
        return;
    }
    LOG_FIRST_N(INFO, 1) << "[GLIDER] Recieved GPS measurement";
    Eigen::Vector3d gps = GliderROS::Conversions::rosToEigen<Eigen::Vector3d>(*msg);
    if (!gps.allFinite())
    {
        markGpsUnavailable();
        LOG_FIRST_N(WARNING, 10) << "[GLIDER] GPS ignored: non-finite position";
        return;
    }

    int64_t timestamp = getSynchronizedTime(msg->header.stamp, "GPS", gps_clock_offset_);
    markGpsAccepted(timestamp);

    const double sigma = std::max(glider_->params().gps_noise, std::sqrt(variance));
    glider_->addGps(timestamp, gps, sigma);

    if (!use_odom_)
    {
        current_state_ = glider_->optimize(timestamp);
        if (publish_nsf_ && current_state_.isInitialized()) publishNavSatFix(current_state_);
    }
}

void GliderNode::odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
    if (!use_odom_) return;
    Eigen::Isometry3d pose = GliderROS::Conversions::rosToEigen<Eigen::Isometry3d>(*msg);
    if (!pose.matrix().allFinite())
    {
        LOG_FIRST_N(WARNING, 10) << "[GLIDER] LIO odometry ignored: non-finite pose";
        return;
    }
    Eigen::Vector3d velocity_body(msg->twist.twist.linear.x,
                                  msg->twist.twist.linear.y,
                                  msg->twist.twist.linear.z);
    if (!velocity_body.allFinite())
    {
        LOG_FIRST_N(WARNING, 10) << "[GLIDER] LIO odometry ignored: non-finite velocity";
        return;
    }
    // nav_msgs/Odometry expresses twist in child_frame_id; the graph velocity
    // is in the map frame.
    Eigen::Vector3d velocity_map = pose.rotation() * velocity_body;
    const double velocity_variance = std::max({msg->twist.covariance[0],
                                               msg->twist.covariance[7],
                                               msg->twist.covariance[14]});
    const double velocity_sigma = std::isfinite(velocity_variance) && velocity_variance > 1e-8
                                      ? std::sqrt(velocity_variance)
                                      // Treat zero covariance as unknown.
                                      : 0.1;
    int64_t timestamp = getSynchronizedTime(msg->header.stamp, "LIO odometry", odom_clock_offset_);
    if (glider_->addOdom(timestamp, pose, velocity_map, velocity_sigma))
    {
        current_state_ = glider_->optimize(timestamp);
        if (publish_nsf_ && current_state_.isInitialized()) publishNavSatFix(current_state_);
    }
}

void GliderNode::gpsGoalCallback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)
{
    if (!glider_->isGpsOffsetInitialized())
    {
        LOG_FIRST_N(WARNING, 1) << "[GLIDER] GPS Goal ignored: System not initialized with global origin yet.";
        return;
    }

    LOG(INFO) << "[GLIDER] Received GPS Goal: " << msg->latitude << ", " << msg->longitude;

    double easting, northing;
    char zone[10];
    Glider::geodetics::LLtoUTM(msg->latitude, msg->longitude, northing, easting, zone);
    Eigen::Vector3d goal_utm(easting, northing, 0.0);

    Eigen::Vector3d gps_offset = glider_->getGpsOffset();
    Eigen::Vector3d goal_local = goal_utm - gps_offset;

    LOG(INFO) << "[GLIDER] GPS Goal in map frame: " << goal_local(0) << ", " << goal_local(1);

    geometry_msgs::msg::PoseStamped goal_msg;
    goal_msg.header.stamp = this->now();
    goal_msg.header.frame_id = map_frame_;
    goal_msg.pose.position.x = goal_local(0);
    goal_msg.pose.position.y = goal_local(1);
    goal_msg.pose.position.z = 0.0;
    goal_msg.pose.orientation.w = 1.0; // Default orientation

    goal_pub_->publish(goal_msg);
}

void GliderNode::publishOdometry(Glider::OdometryWithCovariance& state) const
{
    LOG_FIRST_N(INFO, 1) << "[GLIDER] Publishing Odometry from optimzation";
    nav_msgs::msg::Odometry msg = GliderROS::Conversions::odomToRos<nav_msgs::msg::Odometry>(state, map_frame_);
    msg.child_frame_id = base_link_frame_;
    odom_pub_->publish(msg);

    geometry_msgs::msg::TransformStamped tf;
    tf.header = msg.header;
    tf.child_frame_id = msg.child_frame_id;
    tf.transform.translation.x = msg.pose.pose.position.x;
    tf.transform.translation.y = msg.pose.pose.position.y;
    tf.transform.translation.z = msg.pose.pose.position.z;
    tf.transform.rotation = msg.pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf);

    if (viz_) publishOdometryViz(msg);
}

void GliderNode::publishOdometry(Glider::Odometry& odom) const
{
    nav_msgs::msg::Odometry msg = GliderROS::Conversions::odomToRos<nav_msgs::msg::Odometry>(odom, map_frame_);
    msg.child_frame_id = base_link_frame_;
    GliderROS::Conversions::addCovariance<nav_msgs::msg::Odometry>(current_state_, msg);
    odom_pub_->publish(msg);

    geometry_msgs::msg::TransformStamped tf;
    tf.header = msg.header;
    tf.child_frame_id = msg.child_frame_id;
    tf.transform.translation.x = msg.pose.pose.position.x;
    tf.transform.translation.y = msg.pose.pose.position.y;
    tf.transform.translation.z = msg.pose.pose.position.z;
    tf.transform.rotation = msg.pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf);

    if (viz_) publishOdometryViz(msg);
}

void GliderNode::publishNavSatFix(Glider::OdometryWithCovariance& state) const
{
    if (!glider_->isGpsInitialized()) return;

    // TODO add covariance
    LOG_FIRST_N(INFO, 1) << "[GLIDER] Publishing NavSatFix from optimization";
    Eigen::Vector3d offset = glider_->getGpsOffset();
    if (offset.norm() == 0.0 && origin_easting_ != 0.0) {
        offset(0) = origin_easting_;
        offset(1) = origin_northing_;
    }
    std::string zone = glider_->getUtmZone();
    if (zone.empty()) zone = utm_zone_;
    sensor_msgs::msg::NavSatFix msg = GliderROS::Conversions::odomToRos<sensor_msgs::msg::NavSatFix>(state, base_link_frame_, zone.c_str(), offset);
    if (current_state_.isInitialized())
        GliderROS::Conversions::addCovariance<sensor_msgs::msg::NavSatFix>(current_state_, msg);
    gps_pub_->publish(msg);
}

void GliderNode::publishNavSatFix(Glider::Odometry& odom) const
{
    if (!glider_->isGpsInitialized()) return;

    // TODO add covariance
    LOG_FIRST_N(INFO, 1) << "[GLIDER] Publishing NavSatFix from prediction";
    Eigen::Vector3d offset = glider_->getGpsOffset();
    if (offset.norm() == 0.0 && origin_easting_ != 0.0) {
        offset(0) = origin_easting_;
        offset(1) = origin_northing_;
    }
    sensor_msgs::msg::NavSatFix msg = GliderROS::Conversions::odomToRos<sensor_msgs::msg::NavSatFix>(odom, base_link_frame_, utm_zone_.c_str(), offset);
    if (current_state_.isInitialized())
        GliderROS::Conversions::addCovariance<sensor_msgs::msg::NavSatFix>(current_state_, msg);
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
