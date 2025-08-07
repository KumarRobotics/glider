/*
* struct for quaternion operations
*/

#pragma once

#include <Eigen/Dense>

namespace Glider
{

struct Quaternion
{
    Quaternion() = default;
    Quaternion(double x, double y, double z, double w);

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
