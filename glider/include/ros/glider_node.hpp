/*
* Jason Hughes
* July 2025
*
* ROS node header
*/

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "glider/core/glider.hpp"
#include "glider/core/odometry.hpp"
#include "glider/core/state.hpp"

#include "ros/conversions.hpp"

namespace GliderROS
{
class GliderNode : public rclcpp::Node
{
    public:
        GliderNode() = default;
        GliderNode(const rclcpp::NodeOptions& options);

    private:
        std::unique_ptr<Glider::Glider> glider_;

        // timer callbacks 
        void interpolationCallback();

        // subscriber callbacks
        void gpsCallback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg);
        void imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr msg);
        void magCallback(const sensor_msgs::msg::MagneticField::ConstSharedPtr msg);
        void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
        void poseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);

        // utility functions
        int64_t getTime(const builtin_interfaces::msg::Time& stamp) const;

        // subscriptions
        rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::ConstSharedPtr gps_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::ConstSharedPtr imu_sub_;
        rclcpp::Subscription<sensor_msgs::msg::MagneticField>::ConstSharedPtr mag_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::ConstSharedPtr odom_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::ConstSharedPtr pose_sub_;

        // groups
        rclcpp::CallbackGroup::SharedPtr imu_group_;
        rclcpp::CallbackGroup::SharedPtr gps_group_;
        
        // publishers 
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;

        // timers
        rclcpp::TimerBase::SharedPtr timer_;

        // parameters
        bool initialized_;
        bool use_sim_time_;
        bool publish_nsf_;
        std::string utm_zone_;
        double declination_;

        // tracker
        Glider::State current_state_;
};
}
