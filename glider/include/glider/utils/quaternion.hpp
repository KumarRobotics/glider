/*
* struct for quaternion operations
*/

#pragma once

#include <Eigen/Dense>
#include <geometry_msgs/Quaternion.h>

namespace glider
{

struct Quaternion
{
    Quaternion() = default;
    Quaternion(double x, double y, double z, double w);
    Quaternion(geometry_msgs::Quaternion q);

    friend Quaternion operator-(const Quaternion& q1, const Quaternion& q2);
    friend Quaternion operator*(const Quaternion& q1, const Quaternion& q2);
    
    Quaternion inverse() const;
    Quaternion conjugate() const;

    Eigen::Vector4d toEigen();

    double x;
    double y;
    double z;
    double w;
};
}
