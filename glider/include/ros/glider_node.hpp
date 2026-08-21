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
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <gps_msgs/msg/gps_fix.hpp>
#include <dgps_msgs/msg/differential_nav_sat_fix.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <optional>

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
        void dgpsCallback(const dgps_msgs::msg::DifferentialNavSatFix::ConstSharedPtr msg);
        void gpsCallback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg);
        void imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr msg);
        void magCallback(const sensor_msgs::msg::MagneticField::ConstSharedPtr msg);
        void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
        void poseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
        void gpsGoalCallback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg);

        // utility functions
        int64_t getTime(const builtin_interfaces::msg::Time& stamp) const;
        int64_t getSynchronizedTime(const builtin_interfaces::msg::Time& stamp, const char* source,
                                    std::optional<int64_t>& clock_offset);
        void updateEnvironmentState(int64_t timestamp);
        void markGpsAccepted(int64_t timestamp);
        void markGpsUnavailable();
        void publishOdometry(Glider::OdometryWithCovariance& state) const;
        void publishOdometry(Glider::Odometry& odom) const;
        void publishNavSatFix(Glider::OdometryWithCovariance& state) const;
        void publishNavSatFix(Glider::Odometry& odom) const;
        void publishOdometryViz(nav_msgs::msg::Odometry viz_msg) const;

        // subscriptions
        rclcpp::Subscription<dgps_msgs::msg::DifferentialNavSatFix>::ConstSharedPtr dgps_sub_;
        rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::ConstSharedPtr gps_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::ConstSharedPtr imu_sub_;
        rclcpp::Subscription<sensor_msgs::msg::MagneticField>::ConstSharedPtr mag_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::ConstSharedPtr odom_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::ConstSharedPtr pose_sub_;
        rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_goal_sub_;

        // groups
        rclcpp::CallbackGroup::SharedPtr imu_group_;
        rclcpp::CallbackGroup::SharedPtr gps_group_;

        // publishers
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_viz_pub_;
        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;

        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

        // timers
        rclcpp::TimerBase::SharedPtr timer_;

        // parameters
        bool publish_nsf_;
        bool viz_;
        bool use_odom_;
        bool use_gps_;
        bool use_dgps_;
        double gps_rejection_variance_;
        double max_stamp_skew_sec_;
        double gps_loss_timeout_sec_;
        std::string utm_zone_;
        std::string map_frame_;
        std::string base_link_frame_;
        double origin_easting_;
        double origin_northing_;
        double freq_;

        // tracker
        Glider::OdometryWithCovariance current_state_;
        std::optional<int64_t> imu_clock_offset_;
        std::optional<int64_t> gps_clock_offset_;
        std::optional<int64_t> dgps_clock_offset_;
        std::optional<int64_t> odom_clock_offset_;
        enum class EnvironmentState { Unknown, Outdoor, Indoor };
        EnvironmentState environment_state_{EnvironmentState::Unknown};
        std::optional<int64_t> last_accepted_gps_time_;
};
}
