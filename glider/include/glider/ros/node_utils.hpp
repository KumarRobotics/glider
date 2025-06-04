
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Header.h>

#include "glider/core/state.hpp"

namespace rosutil
{

ros::Duration hzToDuration(double freq)
{
    if (freq <= 0)
    {
        ROS_WARN("Invalid frequency: %f Hz. Using 1 Hz instead", freq);
        freq = 1.0;
    }

    double period_seconds = 1.0 / freq;

    return ros::Duration(period_seconds);
}

std_msgs::Header getHeader(std::string frame_id)
{
    std_msgs::Header msg;
    msg.stamp = ros::Time::now();
    msg.frame_id = frame_id;

    return msg;
}

template <typename T>
T toRosMsg(glider::State& state, const char* zone = nullptr)
{
    if constexpr (std::is_same_v<T, sensor_msgs::NavSatFix>)
    {
        sensor_msgs::NavSatFix msg;
        if (zone == nullptr)
        {
            throw std::invalid_argument("specify a zone for UTM to GPS conversion");
        }
        else
        {
            std::pair<double, double> latlon = state.getLatLon(zone);

            msg.latitude = latlon.first;
            msg.longitude = latlon.second;
            msg.altitude = state.getAltitude();
            msg.position_covariance_type = 3;
            Eigen::Matrix3d cov = state.getPositionCovariance();
            for (int i = 0; i < cov.rows(); ++i) 
            {
                for (int j = 0; j < cov.cols(); ++j) 
                {
                    msg.position_covariance[i * 3 + j] = cov(i, j);
                }
            }
            msg.header = getHeader("enu");
        }
        return msg;
    }
    else if constexpr (std::is_same_v<T, nav_msgs::Odometry>)
    {
        nav_msgs::Odometry msg;

        Eigen::Vector3d p = state.getPosition<Eigen::Vector3d>();
        msg.pose.pose.position.x = p(0);
        msg.pose.pose.position.y = p(1);
        msg.pose.pose.position.z = p(2);

        Eigen::Quaterniond q = state.getOrientation<Eigen::Quaterniond>();
        msg.pose.pose.orientation.w = q.w();
        msg.pose.pose.orientation.x = q.x();
        msg.pose.pose.orientation.y = q.y();
        msg.pose.pose.orientation.z = q.z();

        Eigen::MatrixXd cov = state.getPoseCovariance();
        for (int i = 0; i < cov.rows(); ++i) 
        {
            for (int j = 0; j < cov.cols(); ++j) 
            {
                msg.pose.covariance[i * cov.rows() + j] = cov(i, j);
            }
        }

        Eigen::Vector3d v = state.getVelocity<Eigen::Vector3d>();

        msg.twist.twist.linear.x = v(0);
        msg.twist.twist.linear.y = v(1);
        msg.twist.twist.linear.z = v(2);
        
        cov = state.getVelocityCovariance();
        for (int i = 0; i < cov.rows(); ++i) 
        {
            for (int j = 0; j < cov.cols(); ++j) 
            {
                msg.twist.covariance[i * cov.rows() + j] = cov(i, j);
            }
        }
        msg.header = getHeader("enu-utm");

    }
    else
    {
        static_assert(!std::is_same_v<T, sensor_msgs::NavSatFix> ||
                      !std::is_same_v<T, nav_msgs::Odometry>, "unsupported ros msg, use Odometry or NavSatFix");
    }
}

template <typename T>
T toRosMsg(glider::Odometry& odom, const char* zone = nullptr)
{
    if constexpr (std::is_same_v<T, sensor_msgs::NavSatFix>)
    {
        sensor_msgs::NavSatFix msg;
        if (zone == nullptr)
        {
            throw std::invalid_argument("specify a zone for UTM to GPS converstion");
        }
        else
        {
            std::pair<double, double> latlon = odom.getLatLon(zone);

            msg.latitude = latlon.first;
            msg.longitude = latlon.second;
            msg.altitude = odom.getAltitude();
            msg.position_covariance_type = 3;
            msg.header = getHeader("enu");
        }
        return msg;
    }
    else if constexpr (std::is_same_v<T, nav_msgs::Odometry>)
    { 
        nav_msgs::Odometry msg;

        Eigen::Vector3d p = odom.getPosition<Eigen::Vector3d>();
        msg.pose.pose.position.x = p(0);
        msg.pose.pose.position.y = p(1);
        msg.pose.pose.position.z = p(2);

        Eigen::Quaterniond q = odom.getOrientation<Eigen::Quaterniond>();
        msg.pose.pose.orientation.w = q.w();
        msg.pose.pose.orientation.x = q.x();
        msg.pose.pose.orientation.y = q.y();
        msg.pose.pose.orientation.z = q.z();

        Eigen::Vector3d v = odom.getVelocity<Eigen::Vector3d>();

        msg.twist.twist.linear.x = v(0);
        msg.twist.twist.linear.y = v(1);
        msg.twist.twist.linear.z = v(2);
        msg.header = getHeader("enu-utm");

        return msg;
    }
    else
    {
        static_assert(!std::is_same_v<T, sensor_msgs::NavSatFix> ||
                      !std::is_same_v<T, nav_msgs::Odometry>, "unsupported ros msg, use Odometry or NavSatFix");
    }
}

template<typename T>
void addCovariance(glider::State& state, T& msg)
{
    if constexpr (std::is_same_v<T, sensor_msgs::NavSatFix>)
    {
        Eigen::Matrix3d cov = state.getPositionCovariance();
        for (int i = 0; i < cov.rows(); ++i) 
        {
            for (int j = 0; j < cov.cols(); ++j) 
            {
                msg.position_covariance[i * 3 + j] = cov(i, j);
            }
        }
    }
    else if constexpr (std::is_same_v<T, nav_msgs::Odometry>)
    { 
        Eigen::MatrixXd cov = state.getPoseCovariance();
        for (int i = 0; i < cov.rows(); ++i) 
        {
            for (int j = 0; j < cov.cols(); ++j) 
            {
                msg.pose.covariance[i * cov.rows() + j] = cov(i, j);
            }
        }
        cov = state.getVelocityCovariance();
        for (int i = 0; i < cov.rows(); ++i) 
        {
            for (int j = 0; j < cov.cols(); ++j) 
            {
                msg.twist.covariance[i * cov.rows() + j] = cov(i, j);
            }
        }
    }
    else
    {
        static_assert(!std::is_same_v<T, sensor_msgs::NavSatFix> ||
                      !std::is_same_v<T, nav_msgs::Odometry>, "unsupported ros msg, use Odometry or NavSatFix");
    }
}

} // namespace rosutil
