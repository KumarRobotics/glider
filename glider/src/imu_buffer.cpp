/* Jason Hughes
*
*
*
*/


#include "glider/core/imu_buffer.hpp"

ImuBuffer::ImuBuffer(size_t size = 20) : max_size_(size) { }

void ImuBuffer::add(double timestamp, const Eigen::Vector3d& accel, const Eigen::Vector3d& gyro, const Eigen::Vector4d& orient)
{
    ImuData measurement;
    measurement.accel = accel;
    measurement.gyro = gyro;
    measurement.orient = orient;

    buffer_[timestamp] = measurement;

    while (buffer_.size() > max_size_)
    {
        buffer_.erase(buffer_.begin());
    }
}

std::vector<std::pair<double, ImuData>> ImuBuffer::get(double timestamp_start, double timestamp_end)
{
    std::vector<std::pair<double, ImuData>> measurements;
    auto it = buffer_.lower_bound(timestamp_start);
    
    while (it != buffer_.end() && it -> first < timestamp_end)
    {
        if (it->first >= timestamp_start)
        {
            measurements.push_back(std::make_pair(it->first, it->second));
            ++it;
        }
        if (it->first >= timestamp_end) break;
    }

    return measurements;
}
