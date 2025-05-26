
#include <ros/ros.h>

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

} // namespace rosutil
