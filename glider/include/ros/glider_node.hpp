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
#include <gps_msgs/msg/gps_fix.hpp>

#include "glider/core/glider.hpp"
#include "glider/core/odometry.hpp"
#include "glider/core/odometry_with_covariance.hpp"

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
        void dgpsCallback(const gps_msgs::msg::GPSFix::ConstSharedPtr msg);
        void gpsCallback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg);
        void imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr msg);
        void magCallback(const sensor_msgs::msg::MagneticField::ConstSharedPtr msg);
        void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);

        // utility functions
        int64_t getTime(const builtin_interfaces::msg::Time& stamp) const;
        void publishOdometry(Glider::OdometryWithCovariance& state) const;
        void publishOdometry(Glider::Odometry& odom) const;
        void publishNavSatFix(Glider::OdometryWithCovariance& state) const;
        void publishNavSatFix(Glider::Odometry& odom) const;
        void publishOdometryViz(nav_msgs::msg::Odometry viz_msg) const;

        // subscriptions
        rclcpp::Subscription<gps_msgs::msg::GPSFix>::ConstSharedPtr dgps_sub_;
        rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::ConstSharedPtr gps_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::ConstSharedPtr imu_sub_;
        rclcpp::Subscription<sensor_msgs::msg::MagneticField>::ConstSharedPtr mag_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::ConstSharedPtr odom_sub_;

        // groups
        rclcpp::CallbackGroup::SharedPtr imu_group_;
        rclcpp::CallbackGroup::SharedPtr gps_group_;
        
        // publishers 
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_viz_pub_;
        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;

        // timers
        rclcpp::TimerBase::SharedPtr timer_;

        // parameters
        bool initialized_;
        bool publish_nsf_;
        bool viz_;
        std::string utm_zone_;
        double origin_easting_;
        double origin_northing_;
        double freq_;
        Eigen::Vector2d heading_;
        double initial_alt_;
        bool init_alt_{false};

        // tracker
        Glider::OdometryWithCovariance current_state_;
};
}
