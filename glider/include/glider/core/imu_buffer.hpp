/* Jason Hughes 
*
*
*
*/
#pragma once

#include <map>
#include <vector>
#include <utility>
#include <Eigen/Dense>

struct ImuData
{
    Eigen::Vector3d accel;
    Eigen::Vector3d gyro;
    Eigen::Vector4d orient;
};

class ImuBuffer
{
    public:
        ImuBuffer() = default;
        ImuBuffer(size_t size);

        void add(double timestamp, const Eigen::Vector3d& accel, const Eigen::Vector3d& gyro, const Eigen::Vector4d& orient);
        std::vector<std::pair<double, ImuData>> get(double timestamp_start, double timestamp_end);

    private:
        std::map<double, ImuData> buffer_;
        size_t max_size_;
};


