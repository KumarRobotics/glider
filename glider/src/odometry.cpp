/*
* Jason Hughes
* May 2025
*
* Struct to keep track of the robots odometry, 
* not its full state.
*/

#include "glider/core/odometry.hpp"
#include "glider/utils/geodetics.hpp"

using namespace glider;

Odometry::Odometry(gtsam::Values& vals, gtsam::Key key_index, bool init)
{
    pose_ = vals.at<gtsam::Pose3>(X(key_index));
    orientation_ = pose_.rotation();
    position_ = pose_.translation();
    velocity_ = vals.at<gtsam::Point3>(V(key_index));

    altitude_ = pose_.translation().z();
    heading_ = pose_.rotation().yaw();
    initialized_ = init;
}

Odometry::Odometry(gtsam::NavState& ns, bool init)
{
    position_ = ns.position();
    orientation_ = ns.attitude();
    pose_ = gtsam::Pose3(orientation_, position_);
    velocity_ = ns.v();
    altitude_ = position_.z();
    heading_ = orientation_.yaw();
    initialized_ = init;
}

Odometry Odometry::Uninitialized()
{
    Odometry odom;
    odom.setInitializedStatus(false);

    return odom;
}

bool Odometry::isInitialized() const
{
     return initialized_;
}

gtsam::NavState Odometry::getNavState() const
{
    gtsam::NavState ns(pose_, velocity_);
    return ns;
}

template<typename T>
T Odometry::getPose() const
{
    if constexpr (std::is_same_v<T, gtsam::Pose3>) 
    {
        return pose_;
    }
    else if constexpr (std::is_same_v<T, std::pair<Eigen::Vector3d, Eigen::Vector4d>>)
    {
        return getEigenPose<Eigen::Vector3d, Eigen::Vector4d>();
    }
    else if constexpr (std::is_same_v<T, std::pair<Eigen::Vector3d, Eigen::Quaterniond>>)
    {
        return getEigenPose<Eigen::Vector3d, Eigen::Quaterniond>();
    }
    else
    {
       static_assert(std::is_same_v<T, gtsam::Pose3> ||
                     std::is_same_v<T, std::pair<Eigen::Vector3d, Eigen::Vector4d>> ||
                     std::is_same_v<T, std::pair<Eigen::Vector3d, Eigen::Quaterniond>>, "unsupported type");
    }
}

template<typename TF, typename TS>
std::pair<TF,TS> Odometry::getEigenPose() const
{
    //DONE
    if constexpr (std::is_same_v<TS, Eigen::Vector4d>)
    {
        Eigen::Vector3d t(position_.x(), position_.y(), position_.z());
        Eigen::Matrix3d rot = orientation_.matrix();
        Eigen::Quaterniond q(rot);
        Eigen::Vector4d qvec(q.w(), q.x(), q.y(), q.z());

        return std::make_pair(t, qvec);
    }
    else if constexpr (std::is_same_v<TS, Eigen::Quaterniond>)
    {
        Eigen::Vector3d t(position_.x(), position_.y(), position_.z());
        Eigen::Matrix3d rot = orientation_.matrix();
        Eigen::Quaterniond q(rot);

        return std::make_pair(t, q);
    }
}

template<typename T>
T Odometry::getPosition() const
{
    if constexpr (std::is_same_v<T, gtsam::Point3>)
    {
        return position_;
    }
    else if constexpr (std::is_same_v<T, Eigen::Vector3d>)
    {
        Eigen::Vector3d p(position_.x(), position_.y(), position_.z());
        return p;
    }
    else
    {      
        static_assert(std::is_same_v<T, gtsam::Point3> ||
                      std::is_same_v<T, Eigen::Vector3d>, "unsupported type");
    }
}

template<typename T>
T Odometry::getVelocity() const
{
    if constexpr (std::is_same_v<T, gtsam::Vector3>)
    {
        return velocity_; 
    }
    else if constexpr (std::is_same_v<T, Eigen::Vector3d>)
    {
        Eigen::Vector3d v(velocity_.x(), velocity_.y(), velocity_.z());
        return v;
    }
    else
    {   
        static_assert(std::is_same_v<T, gtsam::Vector3> ||
                      std::is_same_v<T, Eigen::Vector3d>, "unsupported type");
    }
}

template<typename T>
T Odometry::getOrientation() const
{
    if constexpr (std::is_same_v<T, gtsam::Rot3>)
    {
        return orientation_;
    }
    else if constexpr (std::is_same_v<T, gtsam::Quaternion>)
    {
        gtsam::Quaternion q(orientation_.matrix());
        return q;
    }
    else if constexpr (std::is_same_v<T, Eigen::Vector4d>)
    {
        gtsam::Quaternion q(orientation_.matrix());
        Eigen::Vector4d qvec(q.w(), q.x(), q.y(), q.z());

        return qvec;
    }
    else if constexpr (std::is_same_v<T, Eigen::Quaterniond>)
    {
        Eigen::Quaterniond q(orientation_.matrix());
        return q;
    }
    else
    {
        static_assert(std::is_same_v<T, gtsam::Rot3> ||
                      std::is_same_v<T, gtsam::Quaternion> ||
                      std::is_same_v<T, Eigen::Vector4d> ||
                      std::is_same_v<T, Eigen::Quaterniond>, "unsupported type");
    }
}

double Odometry::getLatitude(const char* zone)
{
    double temp;
    geodetics::UTMtoLL(position_.x(), position_.y(), zone, latitude_, temp);

    return latitude_;
}

double Odometry::getLongitude(const char* zone)
{
    double temp;
    geodetics::UTMtoLL(position_.x(), position_.y(), zone, temp, longitude_);

    return longitude_;
}

std::pair<double, double> Odometry::getLatLon(const char* zone)
{
    geodetics::UTMtoLL(position_.x(), position_.y(), zone, latitude_, longitude_);
    return std::make_pair(latitude_, longitude_);
}

double Odometry::getHeading() const
{
    return heading_;
}

double Odometry::getAltitude() const
{
    return altitude_;
}

double Odometry::getHeadingDegrees() const
{
    double heading_deg = (heading_ * 180.0) / M_PI;
    if (heading_deg < 0.0) 
    {
        heading_deg += 360.0;
    }
    else if (heading_deg > 360.0)
    {
        heading_deg -= 360.0;
    }

    return heading_deg;
}

void Odometry::setInitializedStatus(bool init)
{
    initialized_ = init;
}

template gtsam::Pose3 Odometry::getPose<gtsam::Pose3>() const;
template std::pair<Eigen::Vector3d, Eigen::Vector4d> Odometry::getPose<std::pair<Eigen::Vector3d, Eigen::Vector4d>>() const;
template std::pair<Eigen::Vector3d, Eigen::Quaterniond> Odometry::getPose<std::pair<Eigen::Vector3d, Eigen::Quaterniond>>() const;

template std::pair<Eigen::Vector3d, Eigen::Vector4d> Odometry::getEigenPose<Eigen::Vector3d, Eigen::Vector4d>() const;
template std::pair<Eigen::Vector3d, Eigen::Quaterniond> Odometry::getEigenPose<Eigen::Vector3d, Eigen::Quaterniond>() const;

template gtsam::Point3 Odometry::getPosition<gtsam::Point3>() const;

template gtsam::Vector3 Odometry::getVelocity<gtsam::Vector3>() const;

template gtsam::Rot3 Odometry::getOrientation<gtsam::Rot3>() const;
template gtsam::Quaternion Odometry::getOrientation<gtsam::Quaternion>() const;
template Eigen::Vector4d Odometry::getOrientation<Eigen::Vector4d>() const;
template Eigen::Quaterniond Odometry::getOrientation<Eigen::Quaterniond>() const;
