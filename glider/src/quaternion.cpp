/*
*
*
*/

#include "glider/utils/quaternion.hpp"

namespace glider
{

Quaternion::Quaternion(double x_in, double y_in, double z_in, double w_in)
{
    x = x_in;
    y = y_in;
    z = z_in;
    w = w_in;
}

Quaternion::Quaternion(geometry_msgs::Quaternion q)
{
    x = q.x;
    y = q.y;
    z = q.z;
    w = q.w;
}

Quaternion Quaternion::inverse() const
{
    Quaternion qc = conjugate();
    double norm_squared = pow(x, 2.0) + pow(y, 2.0) + pow(z, 2.0) + pow(w, 2.0);

    Quaternion qinv(qc.x/norm_squared, qc.y/norm_squared, qc.z/norm_squared, qc.w/norm_squared);
    return qinv;
}

Quaternion Quaternion::conjugate() const
{
    return Quaternion(-x, -y, -z, w);
}   

Quaternion operator-(const Quaternion& q1, const Quaternion& q2)
{
    return q2 * q1.inverse();
}

Quaternion operator*(const Quaternion& q1, const Quaternion& q2)
{
    double x_new = (q1.w * q2.x) + (q1.x * q2.w) + (q1.y * q2.z) - (q1.z * q2.y); 
    double y_new = (q1.w * q2.y) - (q1.x * q2.z) + (q1.y * q2.w) + (q1.z * q2.x);
    double z_new = (q1.w * q2.z) + (q1.x * q2.y) - (q1.y * q2.x) + (q1.z * q2.w);
    double w_new = (q1.w * q2.w) - (q1.x * q2.x) - (q1.y * q2.y) - (q1.z * q2.z);

    return Quaternion(x_new, y_new, z_new, w_new);
}

Eigen::Vector4d Quaternion::toEigen()
{
    Eigen::Vector4d ret(w, x, y, z);
    return ret;
}
}
