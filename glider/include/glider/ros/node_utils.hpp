
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

sensor_msgs::NavSatFix toNavSatFix(glider::State& state, const char* zone)
{
    sensor_msgs::NavSatFix msg;
    std::pair<double, double> latlon = state.getLatLon(zone);

    msg.latitude = latlon.first;
    msg.longitude = latlon.second;
    msg.altitude = state.altitude;
    // TODO need to come back to this
    return msg;
}

nav_msgs::Odometry toOdometry(const glider::State& state)
{

}
} // namespace rosutil
