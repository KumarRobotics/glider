

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

    nav_heading_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "/ublox_raw/navheading",
        10,
        std::bind(&GlobalEKFNode::navHeadingCallback, this, std::placeholders::_1)
    );

    auto qos = rclcpp::SensorDataQoS();
    mag_heading_sub_ = create_subscription<std_msgs::msg::Float64>(
        "/mavros/global_position/compass_hdg",
        qos,
        std::bind(&GlobalEKFNode::magHeadingCallback, this, std::placeholders::_1)
    );

    state_pub_ = create_publisher<gps_msgs::msg::GPSFix>(
        "/gekf/dgps",
        10 
    );

    std::chrono::milliseconds d = GliderROS::Conversions::hzToDuration(10.0);
    timer_ = create_wall_timer(
        d,
        std::bind(&GlobalEKFNode::magPassCallback, this)
    );
}

int64_t GlobalEKFNode::getTime(const builtin_interfaces::msg::Time& stamp) const
{
   return (static_cast<int64_t>(stamp.sec) * 1000000000LL) + static_cast<int64_t>(stamp.nanosec);
}

void GlobalEKFNode::magPassCallback()
{ 
    bool status = ekf_->updateCompass(mag_stamped_.first, mag_stamped_.second);
}

void GlobalEKFNode::publishState(const Glider::GlobalState& state)
{
    gps_msgs::msg::GPSFix msg;
    msg.latitude = state.latitude;
    msg.longitude = state.longitude;
    msg.altitude = state.altitude;
    msg.track = state.heading;

    state_pub_->publish(msg);
}

void GlobalEKFNode::gpsCallback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)
{
    int64_t timestamp = getTime(this->now());
    int8_t status = msg->status.status;
    Eigen::Vector3d gps = GliderROS::Conversions::rosToEigen<Eigen::Vector3d>(*msg);
    
    ekf_->updateFixStatus(timestamp, status);

    double hs = msg->position_covariance[0];
    double vs = msg->position_covariance[8];
    ekf_->updatePosition(gps, hs, vs);

    Glider::GlobalState state = ekf_->state<Glider::GlobalFrame::NED>();
    publishState(state);
    std::cout << "[GEKF] Heading: " << state.heading_deg << " Using GPS: "<< to_string(ekf_->gate().state()) << std::endl;
}

void GlobalEKFNode::navHeadingCallback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
    // this timestamp is NO BUENO
    int64_t timestamp = getTime(this->now());
    Eigen::Quaterniond quat = GliderROS::Conversions::rosToEigen<Eigen::Quaterniond>(msg->orientation);
    double sigma = msg->orientation_covariance[8];

    bool status = ekf_->updateGpsHeading(timestamp, quat);
    std::cout << "[GEKF] GPS Heading Status: " << std::boolalpha << status << " heading: "<< ekf_->heading()* 180.0 / M_PI << std::endl;
}

void GlobalEKFNode::magHeadingCallback(const std_msgs::msg::Float64::ConstSharedPtr msg)
{
    int64_t timestamp = getTime(this->now());
    double heading = kDegToRad * msg->data;
    
    mag_stamped_ = std::make_pair(timestamp, heading);
}


