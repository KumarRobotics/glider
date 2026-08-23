

#include "ros/gekf_node.hpp"

using namespace GliderROS;

constexpr double kDegToRad = M_PI / 180.0;
constexpr double kRadToDeg = 180.0 / M_PI;

GlobalEKFNode::GlobalEKFNode(const rclcpp::NodeOptions& options) : rclcpp::Node("gekf_node", options)
{
    ekf_ = std::make_unique<Glider::GlobalEKF>();

    gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
        "/ublox_gps_node/fix", 
        10,
        std::bind(&GlobalEKFNode::gpsCallback, this, std::placeholders::_1)
    );

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "/vectornav/imu",
        10,
        std::bind(&GlobalEKFNode::imuCallback, this, std::placeholders::_1)
    );

    nav_heading_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "/ublox_raw/navheading",
        10,
        std::bind(&GlobalEKFNode::navHeadingCallback, this, std::placeholders::_1)
    );

    mag_heading_sub_ = create_subscription<std_msgs::msg::Float64>(
        "/mavros/global_position/heading_deg",
        10,
        std::bind(&GlobalEKFNode::magHeadingCallback, this, std::placeholders::_1)
    );

    state_pub_ = create_publisher<gps_msgs::msg::GPSFix>(
        "/gekf/dgps",
        10 
    );
}

int64_t GlobalEKFNode::getTime(const builtin_interfaces::msg::Time& stamp) const
{
   return (static_cast<int64_t>(stamp.sec) * 1000000000LL) + static_cast<int64_t>(stamp.nanosec);
}

void GlobalEKFNode::publishState(const Glider::GlobalState& state) const
{
    gps_msgs::msg::GPSFix msg;
    msg.latitude = state.latitude;
    msg.longitude = state.longitude;
    msg.altitude = state.altitude;
    msg.track = state.heading;
}

void GlobalEKFNode::gpsCallback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)
{
    int64_t timestamp = getTime(msg->header.stamp);
    int8_t status = msg->status.status;
    Eigen::Vector3d gps = GliderROS::Conversions::rosToEigen<Eigen::Vector3d>(*msg);
    
    ekf_->updateFixStatus(timestamp, status);

    double hs = msg->position_covariance[0];
    double vs = msg->position_covariance[8];
    ekf_->updatePosition(gps, hs, vs);

    Glider::GlobalState state = ekf_->state<Glider::GlobalFrame::NED>();
    publishState(state);
    std::cout << "[GEKF] Heading: " << state.heading_deg << std::endl;
}

void GlobalEKFNode::imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
    int64_t timestamp = getTime(msg->header.stamp);
    Eigen::Vector3d gyro = GliderROS::Conversions::rosToEigen<Eigen::Vector3d>(msg->angular_velocity);

    ekf_->predict(timestamp, gyro);
}

void GlobalEKFNode::navHeadingCallback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
    // this timestamp is NO BUENO
    Eigen::Quaterniond quat = GliderROS::Conversions::rosToEigen<Eigen::Quaterniond>(msg->orientation);
    double sigma = msg->orientation_covariance[8];

    ekf_->updateGpsHeading(quat, sigma);
}

void GlobalEKFNode::magHeadingCallback(const std_msgs::msg::Float64::ConstSharedPtr msg)
{
    double heading = kDegToRad * msg->data;
    
    ekf_->updateCompass(heading);
}


