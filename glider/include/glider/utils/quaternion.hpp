/*
* struct for quaternion operations
*/

#pragma once

#include <Eigen/Dense>
#include <geometry_msgs/msg/quaternion.hpp>

namespace glider
{

struct Quaternion
{
    Quaternion() = default;
    Quaternion(const double x, const double y, const double z, const double w);
    Quaternion(const geometry_msgs::msg::Quaternion& q);    

    friend Quaternion operator-(const Quaternion& q1, const Quaternion& q2);
    friend Quaternion operator*(const Quaternion& q1, const Quaternion& q2);

    static Quaternion fromRot3(Eigen::Matrix3d rot);

    Quaternion inverse() const;
    Quaternion conjugate() const;

    Eigen::Vector4d toEigen();

    double x;
    double y;
    double z;
    double w;
};
}
