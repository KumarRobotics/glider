/*
*
*
* ROS node header
*/
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "glider/core/factor_manager.hpp"
#include "glider/utils/params.hpp"
#include "glider/utils/quaternion.hpp"
#include "glider/core/state.hpp"
#include "glider/core/odometry.hpp"

namespace glider
{

class FactorNode : public rclcpp::Node
{
    public:
        FactorNode() = default;
        FactorNode(const rclcpp::NodeOptions& options);

    private:
        glider::FactorManager factor_manager_;

        void interpolationCallback();
        void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
        void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
        void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
      
        int64_t getTime(const builtin_interfaces::msg::Time& stamp) const;

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
        rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;

        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;

        rclcpp::TimerBase::SharedPtr timer_;

        bool initialized_;
        bool use_sim_time_;
        bool publish_nsf_;
        std::string utm_zone_;
        Params params;

        glider::Quaternion prev_quat_;
        Eigen::Vector3d prev_position_;

        Eigen::Matrix3d ned_to_enu_;
        glider::State current_state_;

        rclcpp::CallbackGroup::SharedPtr imu_group_;
        rclcpp::CallbackGroup::SharedPtr gps_group_;
};
} // namespace glider
