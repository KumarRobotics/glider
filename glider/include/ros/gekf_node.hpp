#pragma once 

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>
#include <gps_msgs/msg/gps_fix.hpp>

#include "ros/conversions.hpp"
#include "glider/gekf/global_ekf.hpp"

namespace GliderROS
{

class GlobalEKFNode : public rclcpp::Node
{
    public:
        GlobalEKFNode() = default;
        GlobalEKFNode(const rclcpp::NodeOptions& options);

    private:
        std::unique_ptr<Glider::GlobalEKF> ekf_;

        void gpsCallback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg);
        void imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr msg);
        void navHeadingCallback(const sensor_msgs::msg::Imu::ConstSharedPtr msg);
        void magHeadingCallback(const std_msgs::msg::Float64::ConstSharedPtr msg);

        void publishState(const Glider::GlobalState& state) const;

        int64_t getTime(const builtin_interfaces::msg::Time& stamp) const;

        rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::ConstSharedPtr gps_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::ConstSharedPtr imu_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::ConstSharedPtr nav_heading_sub_;
        rclcpp::Subscription<std_msgs::msg::Float64>::ConstSharedPtr mag_heading_sub_;

        rclcpp::Publisher<gps_msgs::msg::GPSFix>::ConstSharedPtr state_pub_;
};

} // namespace GliderROS
