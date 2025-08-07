/*
* Jason Hughes
* July 2025
*
* Header file for ros to eigen conversions
*/

#include <chrono>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/header.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <Eigen/Dense>

#include "glider/core/odometry.hpp"
#include "glider/core/state.hpp"

namespace GliderROS
{
class Conversions
{
    public:
        Conversions() = default;

        template<typename Output, typename Input>
        static Output rosToEigen(const Input& msg);

        template<typename Output, typename Input>
        static Output eigenToRos(const Input& vec);

        template<typename Output>
        static Output odomToRos(Glider::Odometry& odom, const char* zone = nullptr);

        template<typename Output>
        static Output stateToRos(Glider::State& state, const char* zone = nullptr);

        template<typename T>
        static void addCovariance(const Glider::State& state, T& msg);

        static std::chrono::milliseconds hzToDuration(const double freq);

    private:
        
        struct RosToEigen
        {
            static Eigen::Vector3d vector3Convert(const geometry_msgs::msg::Vector3& vec);
            static Eigen::Vector4d orientConvert(const geometry_msgs::msg::Quaternion& orient);

            static Eigen::Vector3d gpsConvert(const sensor_msgs::msg::NavSatFix& gps);
            static Eigen::Isometry3d poseConvert(const geometry_msgs::msg::PoseStamped& msg);
            static Eigen::Isometry3d odomConvert(const nav_msgs::msg::Odometry& msg);
        };

        struct EigenToRos
        {
            static geometry_msgs::msg::Vector3 vector3Convert(const Eigen::Vector3d& vec);
            static geometry_msgs::msg::Quaternion orientConvert(const Eigen::Vector4d& orient);
            static geometry_msgs::msg::PoseStamped poseConvert(const Eigen::Isometry3d& vec);

            static sensor_msgs::msg::NavSatFix gpsConvert(const Eigen::Vector3d& gps);

            static sensor_msgs::msg::PointCloud2 pointCloudConvert(const Eigen::Array3Xd& vec);
        };

        static std_msgs::msg::Header getHeader(int64_t timestamp, std::string frame);
};
}
