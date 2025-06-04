
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>

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
            std::copy(state.getPositionCovariance().data(), 
                      state.getPositionCovariance().data()+9, 
                      msg.position_covariance.begin());
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

        std::copy(state.getPoseCovariance().data(),
                  state.getPoseCovariance().data()+36,
                  msg.pose.covariance.begin());

        Eigen::Vector3d v = state.getVelocity<Eigen::Vector3d>();

        msg.twist.twist.linear.x = v(0);
        msg.twist.twist.linear.y = v(1);
        msg.twist.twist.linear.z = v(2);

        std::copy(state.getVelocityCovariance().data(),
                  state.getVelocityCovariance().data()+36,
                  msg.twist.covariance.begin());
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

        return msg;
    }
    else
    {
        static_assert(!std::is_same_v<T, sensor_msgs::NavSatFix> ||
                      !std::is_same_v<T, nav_msgs::Odometry>, "unsupported ros msg, use Odometry or NavSatFix");
    }
}

template<typename T>
T addCovariance(glider::State& state, T& msg)
{
    if constexpr (std::is_same_v<T, sensor_msgs::NavSatFix>)
    { 
        std::copy(state.getPositionCovariance().data(), 
                  state.getPositionCovariance().data()+9, 
                  msg.position_covariance.begin());
    }
    else if constexpr (std::is_same_v<T, nav_msgs::Odometry>)
    { 
        std::copy(state.getPoseCovariance().data(),
                  state.getPoseCovariance().data()+36,
                  msg.pose.covariance.begin());
        std::copy(state.getVelocityCovariance().data(),
                  state.getVelocityCovariance().data()+36,
                  msg.twist.covariance.begin());
    }
    else
    {
        static_assert(!std::is_same_v<T, sensor_msgs::NavSatFix> ||
                      !std::is_same_v<T, nav_msgs::Odometry>, "unsupported ros msg, use Odometry or NavSatFix");
    }
}

} // namespace rosutil
