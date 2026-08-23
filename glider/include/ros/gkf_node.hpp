#pragma once 

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>
#include <gps_msgs/msg/gps_fix.hpp>

#include "ros/conversions.hpp"
#include "glider/gkf/global_kf.hpp"

namespace GliderROS
{

class GlobalKFNode : public rclcpp::Node
{
    public:
        GlobalKFNode() = default;
        GlobalKFNode(const rclcpp::NodeOptions& options);

    private:
        std::unique_ptr<Glider::GlobalKF> ekf_;

        void gpsCallback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg);
        void navHeadingCallback(const sensor_msgs::msg::Imu::ConstSharedPtr msg);
        void magHeadingCallback(const std_msgs::msg::Float64::ConstSharedPtr msg);

        void publishState(const Glider::GlobalState& state);

        void magPassCallback();

        int64_t getTime(const builtin_interfaces::msg::Time& stamp) const;

        rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::ConstSharedPtr gps_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::ConstSharedPtr nav_heading_sub_;
        rclcpp::Subscription<std_msgs::msg::Float64>::ConstSharedPtr mag_heading_sub_;

        rclcpp::Publisher<gps_msgs::msg::GPSFix>::SharedPtr state_pub_;

        rclcpp::TimerBase::SharedPtr timer_;
        std::pair<int64_t, double> mag_stamped_{0,0};
};

} // namespace GliderROS
