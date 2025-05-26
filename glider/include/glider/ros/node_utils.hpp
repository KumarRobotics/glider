
#include <rclcpp/rclcpp.hpp>

namespace rosutil
{

rclcpp::Duration hzToDuration(double freq)
{
    if (freq <= 0)
    {
        std::cout << "[GLIDER] Invalid frequency: " << freq << " Hz. Using 1 Hz instead" << std::endl;
        freq = 1.0;
    }

    double period_seconds = 1.0 / freq;

    return rclcpp::Duration(period_seconds);
}

} // namespace rosutil
