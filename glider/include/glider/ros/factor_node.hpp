/*
*
*
* ROS node header
*/
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <Eigen/Dense>
#include <memory>
#include <chrono>

#include "glider/core/factor_manager.hpp"
#include "glider/utils/quaternion.hpp"

class FactorManagerNode : public rclcpp::Node
{
    public:
        //COMPOSITION_PUBLIC
        explicit FactorManagerNode(const rclcpp::NodeOptions& options);

        Eigen::Vector4d getOrientation() const;
        Eigen::Vector3d getPose() const;

    private:

        Eigen::Vector3d pose_;
        Eigen::Vector4d orientation_;

        glider::FactorManager factor_manager_;
        int imu_msgs_;
        bool initialized_;
        bool use_sim_time_;

        double initial_alt_;

        builtin_interfaces::msg::Time last_imu_ros_stamp_;
        int64_t last_imu_stamp_;
        int64_t last_time_now_;
        Eigen::Vector3d prev_position_;
        glider::Quaternion prev_quat_;

        void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
        void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
        void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

        void predictCallback();

        int64_t getTime(const builtin_interfaces::msg::Time& stamp) const;

        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
        
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
        rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

        rclcpp::TimerBase::SharedPtr timer_;

        rclcpp::CallbackGroup::SharedPtr imu_group_;
        rclcpp::CallbackGroup::SharedPtr gps_group_;
};
