/*
* Jason Hughes
* July 2025
*
* convert between ros and eigen
*/

#include "ros/conversions.hpp"

#include <cctype>

using namespace GliderROS;

template<typename Output, typename Input>
Output Conversions::rosToEigen(const Input& msg)
{
    if constexpr (std::is_same_v<Input, geometry_msgs::msg::Vector3>)
    {
        return RosToEigen::vector3Convert(msg);
    }
    else if constexpr (std::is_same_v<Input, geometry_msgs::msg::Quaternion>)
    {
        return RosToEigen::orientConvert(msg);
    }
    else if constexpr(std::is_same_v<Input, sensor_msgs::msg::NavSatFix>)
    {
        return RosToEigen::gpsConvert(msg);
    }
    else if constexpr(std::is_same_v<Input, geometry_msgs::msg::PoseStamped>)
    {
        return RosToEigen::poseConvert(msg);
    }
    else if constexpr(std::is_same_v<Input, nav_msgs::msg::Odometry>)
    {
        return RosToEigen::odomConvert(msg);
    }
    else if constexpr(std::is_same_v<Input, gps_msgs::msg::GPSFix>)
    {
        return RosToEigen::dgpsConvert(msg);
    }
}

template<typename Output, typename Input>
Output Conversions::eigenToRos(const Input& vec)
{
    if constexpr (std::is_same_v<Input, Eigen::Vector3d> && std::is_same_v<Output, geometry_msgs::msg::Vector3>)
    {
        return EigenToRos::vector3Convert(vec);
    }
    else if constexpr (std::is_same_v<Input, Eigen::Vector3d> && std::is_same_v<Output, sensor_msgs::msg::NavSatFix>)
    {
        return EigenToRos::gpsConvert(vec);
    }
    else if constexpr (std::is_same_v<Input, Eigen::Vector4d>)
    {
        return EigenToRos::orientConvert(vec);
    }
    else if constexpr (std::is_same_v<Input, Eigen::Isometry3d>)
    {
        return EigenToRos::poseConvert(vec);
    }
    else
    {
        static_assert(std::is_same_v<Input, Eigen::Vector3d> ||
                      std::is_same_v<Input, Eigen::Vector4d> ||
                      std::is_same_v<Input, Eigen::Isometry3d>, "Unsupported Eigen type");
        static_assert(std::is_same_v<Output, geometry_msgs::msg::Vector3> ||
                      std::is_same_v<Output, sensor_msgs::msg::NavSatFix> ||
                      std::is_same_v<Output, geometry_msgs::msg::Quaternion> ||
                      std::is_same_v<Output, geometry_msgs::msg::PoseStamped>, "Unsupported Ros Msg type");
    }
}

Eigen::Isometry3d Conversions::RosToEigen::poseConvert(const geometry_msgs::msg::PoseStamped& msg)
{
    Eigen::Quaterniond quat(msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z);
    Eigen::Vector3d trans(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);

    Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
    iso.linear() = quat.toRotationMatrix();
    iso.translation() = trans;

    return iso;
}

Eigen::Vector3d Conversions::RosToEigen::vector3Convert(const geometry_msgs::msg::Vector3& msg)
{
    return Eigen::Vector3d(msg.x, msg.y, msg.z);
}

Eigen::Vector4d Conversions::RosToEigen::orientConvert(const geometry_msgs::msg::Quaternion& msg)
{
    return Eigen::Vector4d(msg.w, msg.x, msg.y, msg.z);
}

Eigen::Vector3d Conversions::RosToEigen::gpsConvert(const sensor_msgs::msg::NavSatFix& msg)
{
    return Eigen::Vector3d(msg.latitude, msg.longitude, msg.altitude);
}

std::pair<Eigen::Vector3d, Eigen::Vector2d> Conversions::RosToEigen::dgpsConvert(const gps_msgs::msg::GPSFix& msg)
{
    Eigen::Vector3d gps(msg.latitude, msg.longitude, msg.altitude);
    Eigen::Vector2d heading(msg.track, msg.err_track);

    return std::make_pair(gps, heading);
}

Eigen::Isometry3d Conversions::RosToEigen::odomConvert(const nav_msgs::msg::Odometry& msg)
{
    Eigen::Quaterniond quat(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z);
    Eigen::Vector3d trans(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z);

    Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
    iso.linear() = quat.toRotationMatrix();
    iso.translation() = trans;

    return iso;
}

geometry_msgs::msg::Vector3 Conversions::EigenToRos::vector3Convert(const Eigen::Vector3d& vec)
{
    geometry_msgs::msg::Vector3 msg;
    msg.x = vec(0);
    msg.y = vec(1);
    msg.z = vec(2);

    return msg;
}

sensor_msgs::msg::NavSatFix Conversions::EigenToRos::gpsConvert(const Eigen::Vector3d& vec)
{
    sensor_msgs::msg::NavSatFix msg;
    msg.latitude = vec(0);
    msg.longitude = vec(1);
    msg.altitude = vec(2);

    return msg;
}

geometry_msgs::msg::Quaternion Conversions::EigenToRos::orientConvert(const Eigen::Vector4d& vec)
{
    geometry_msgs::msg::Quaternion msg;

    msg.w = vec(0);
    msg.x = vec(1);
    msg.y = vec(2);
    msg.z = vec(3);

    return msg;
}

geometry_msgs::msg::PoseStamped Conversions::EigenToRos::poseConvert(const Eigen::Isometry3d& vec)
{
    geometry_msgs::msg::PoseStamped msg;
    msg.pose.position.x = vec.translation().x();
    msg.pose.position.y = vec.translation().y();
    msg.pose.position.z = vec.translation().z();

    Eigen::Quaterniond quat(vec.rotation());
    msg.pose.orientation.x = quat.x();
    msg.pose.orientation.y = quat.y();
    msg.pose.orientation.z = quat.z();
    msg.pose.orientation.w = quat.w();

    return msg;
}

std_msgs::msg::Header Conversions::getHeader(int64_t timestamp, std::string frame)
{
    std_msgs::msg::Header msg;
    msg.frame_id = frame;
    msg.stamp.sec = static_cast<int32_t>(timestamp / 1000000000LL);
    msg.stamp.nanosec = static_cast<uint32_t>(timestamp % 1000000000LL);

    return msg;
}

template<typename Output>
Output Conversions::odomToRos(Glider::Odometry& odom, std::string frame_id, const char* zone, const Eigen::Vector3d& offset)
{
    if constexpr (std::is_same_v<Output, sensor_msgs::msg::NavSatFix>)
    {
        sensor_msgs::msg::NavSatFix msg;
        // Backward compatibility: the original two-argument API used its
        // string argument as the UTM zone and always published in ENU.
        std::string legacy_zone;
        if (zone == nullptr && frame_id.size() >= 2 && std::isdigit(frame_id.front()))
        {
            legacy_zone = frame_id;
            zone = legacy_zone.c_str();
            frame_id = "enu";
        }
        if (zone == nullptr || std::string(zone) == "")
        {
            throw std::invalid_argument("specify a zone for UTM to GPS converstion");
        }
        else
        {
            std::pair<double, double> latlon = odom.getLatLon(zone, offset);

            msg.status.status = odom.isInitialized() ?
                                sensor_msgs::msg::NavSatStatus::STATUS_FIX :
                                sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;

            msg.latitude = latlon.first;
            msg.longitude = latlon.second;
            msg.altitude = odom.getAltitude() + offset(2);
            msg.position_covariance_type = 3;
            msg.header = getHeader(odom.getTimestamp(), frame_id);
        }
        return msg;
    }
    else if constexpr (std::is_same_v<Output, nav_msgs::msg::Odometry>)
    {
        nav_msgs::msg::Odometry msg;

        Eigen::Vector3d p = odom.getPosition<Eigen::Vector3d>();
        msg.pose.pose.position.x = p(0);
        msg.pose.pose.position.y = p(1);
        msg.pose.pose.position.z = p(2);

        Eigen::Quaterniond q = odom.getOrientation<Eigen::Quaterniond>();
        msg.pose.pose.orientation.w = q.w();
        msg.pose.pose.orientation.x = q.x();
        msg.pose.pose.orientation.y = q.y();
        msg.pose.pose.orientation.z = q.z();

        Eigen::Vector3d v = odom.getVelocity<Eigen::Vector3d>();

        msg.twist.twist.linear.x = v(0);
        msg.twist.twist.linear.y = v(1);
        msg.twist.twist.linear.z = v(2);
        msg.header = getHeader(odom.getTimestamp(), frame_id);

        return msg;
    }
    else
    {
        static_assert(std::is_same_v<Output, sensor_msgs::msg::NavSatFix> ||
                      std::is_same_v<Output, nav_msgs::msg::Odometry>, "unsupported ros msg, use Odometry or NavSatFix");
    }
}

template<typename Output>
Output Conversions::odomToRos(Glider::OdometryWithCovariance& odom_wc, std::string frame_id, const char* zone, const Eigen::Vector3d& offset)
{
    if constexpr (std::is_same_v<Output, sensor_msgs::msg::NavSatFix>)
    {
        sensor_msgs::msg::NavSatFix msg;
        if (zone == nullptr || std::string(zone) == "")
        {
            throw std::invalid_argument("specify a zone for UTM to GPS conversion");
        }
        else
        {
            std::pair<double, double> latlon = odom_wc.getLatLon(zone, offset);

            msg.status.status = odom_wc.isGpsOffsetInitialized() ?
                                sensor_msgs::msg::NavSatStatus::STATUS_FIX :
                                sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;

            msg.latitude = latlon.first;
            msg.longitude = latlon.second;
            msg.altitude = odom_wc.getAltitude() + offset(2);
            msg.position_covariance_type = 3;
            Eigen::Matrix3d cov = odom_wc.getPositionCovariance();
            for (int i = 0; i < cov.rows(); ++i)
            {
                for (int j = 0; j < cov.cols(); ++j)
                {
                    msg.position_covariance[i * 3 + j] = cov(i, j);
                }
            }
            msg.header = getHeader(odom_wc.getTimestamp(), frame_id);
        }
        return msg;
    }
    else if constexpr (std::is_same_v<Output, nav_msgs::msg::Odometry>)
    {
        nav_msgs::msg::Odometry msg;

        Eigen::Vector3d p = odom_wc.getPosition<Eigen::Vector3d>();
        msg.pose.pose.position.x = p(0);
        msg.pose.pose.position.y = p(1);
        msg.pose.pose.position.z = p(2);

        Eigen::Quaterniond q = odom_wc.getOrientation<Eigen::Quaterniond>();
        msg.pose.pose.orientation.w = q.w();
        msg.pose.pose.orientation.x = q.x();
        msg.pose.pose.orientation.y = q.y();
        msg.pose.pose.orientation.z = q.z();

        Eigen::MatrixXd cov = odom_wc.getPoseCovariance();
        for (int i = 0; i < cov.rows(); ++i)
        {
            for (int j = 0; j < cov.cols(); ++j)
            {
                msg.pose.covariance[i * cov.rows() + j] = cov(i, j);
            }
        }

        Eigen::Vector3d v = odom_wc.getVelocity<Eigen::Vector3d>();

        msg.twist.twist.linear.x = v(0);
        msg.twist.twist.linear.y = v(1);
        msg.twist.twist.linear.z = v(2);

        cov = odom_wc.getVelocityCovariance();
        for (int i = 0; i < cov.rows(); ++i)
        {
            for (int j = 0; j < cov.cols(); ++j)
            {
                msg.twist.covariance[i * cov.rows() + j] = cov(i, j);
            }
        }
        msg.header = getHeader(odom_wc.getTimestamp(), frame_id);
        return msg;
    }
    else
    {
        static_assert(!std::is_same_v<Output, sensor_msgs::msg::NavSatFix> ||
                      !std::is_same_v<Output, nav_msgs::msg::Odometry>, "unsupported ros msg, use Odometry or NavSatFix");
    }
}

std::chrono::milliseconds Conversions::hzToDuration(double freq)
{
    if (freq <= 0)
    {
        std::cerr << "Invalid frequency: "<< freq << " Hz. Using 1 Hz instead" << std::endl;
        freq = 1.0;
    }

    double period_seconds = 1.0 / freq;

    std::chrono::milliseconds period_ms = std::chrono::milliseconds(static_cast<int64_t>(period_seconds * 1e3));
    return period_ms;
}

template<typename T>
void Conversions::addCovariance(const Glider::OdometryWithCovariance& odom_wc, T& msg)
{
    if constexpr (std::is_same_v<T, sensor_msgs::msg::NavSatFix>)
    {
        Eigen::Matrix3d cov = odom_wc.getPositionCovariance();
        for (int i = 0; i < cov.rows(); ++i)
        {
            for (int j = 0; j < cov.cols(); ++j)
            {
                msg.position_covariance[i * 3 + j] = cov(i, j);
            }
        }
    }
    else if constexpr (std::is_same_v<T, nav_msgs::msg::Odometry>)
    {
        Eigen::MatrixXd cov = odom_wc.getPoseCovariance();
        for (int i = 0; i < cov.rows(); ++i)
        {
            for (int j = 0; j < cov.cols(); ++j)
            {
                msg.pose.covariance[i * cov.rows() + j] = cov(i, j);
            }
        }
        cov = odom_wc.getVelocityCovariance();
        for (int i = 0; i < cov.rows(); ++i)
        {
            for (int j = 0; j < cov.cols(); ++j)
            {
                msg.twist.covariance[i * cov.rows() + j] = cov(i, j);
            }
        }
    }
    else
    {
        static_assert(!std::is_same_v<T, sensor_msgs::msg::NavSatFix> ||
                      !std::is_same_v<T, nav_msgs::msg::Odometry>, "unsupported ros msg, use Odometry or NavSatFix");
    }
}

template Eigen::Vector3d Conversions::rosToEigen<Eigen::Vector3d>(const geometry_msgs::msg::Vector3& msg);
template Eigen::Vector3d Conversions::rosToEigen<Eigen::Vector3d>(const sensor_msgs::msg::NavSatFix& msg);
template Eigen::Vector4d Conversions::rosToEigen<Eigen::Vector4d>(const geometry_msgs::msg::Quaternion& msg);
template Eigen::Isometry3d Conversions::rosToEigen<Eigen::Isometry3d>(const geometry_msgs::msg::PoseStamped& msg);
template Eigen::Isometry3d Conversions::rosToEigen<Eigen::Isometry3d>(const nav_msgs::msg::Odometry& msg);
template std::pair<Eigen::Vector3d, Eigen::Vector2d> Conversions::rosToEigen<std::pair<Eigen::Vector3d, Eigen::Vector2d>>(const gps_msgs::msg::GPSFix& msg);

template geometry_msgs::msg::Vector3 Conversions::eigenToRos<geometry_msgs::msg::Vector3>(const Eigen::Vector3d& vec);
template geometry_msgs::msg::Quaternion Conversions::eigenToRos<geometry_msgs::msg::Quaternion>(const Eigen::Vector4d& vec);
template sensor_msgs::msg::NavSatFix Conversions::eigenToRos<sensor_msgs::msg::NavSatFix>(const Eigen::Vector3d& vec);
template geometry_msgs::msg::PoseStamped Conversions::eigenToRos<geometry_msgs::msg::PoseStamped>(const Eigen::Isometry3d& vec);

template nav_msgs::msg::Odometry Conversions::odomToRos<nav_msgs::msg::Odometry>(Glider::Odometry& odom, std::string frame_id, const char* zone, const Eigen::Vector3d& offset);
template sensor_msgs::msg::NavSatFix Conversions::odomToRos<sensor_msgs::msg::NavSatFix>(Glider::Odometry& odom, std::string frame_id, const char* zone, const Eigen::Vector3d& offset);

template nav_msgs::msg::Odometry Conversions::odomToRos<nav_msgs::msg::Odometry>(Glider::OdometryWithCovariance& odom_wc, std::string frame_id, const char* zone, const Eigen::Vector3d& offset);
template sensor_msgs::msg::NavSatFix Conversions::odomToRos<sensor_msgs::msg::NavSatFix>(Glider::OdometryWithCovariance& odom_wc, std::string frame_id, const char* zone, const Eigen::Vector3d& offset);

template void Conversions::addCovariance<nav_msgs::msg::Odometry>(const Glider::OdometryWithCovariance& odom_wc, nav_msgs::msg::Odometry& msg);
template void Conversions::addCovariance<sensor_msgs::msg::NavSatFix>(const Glider::OdometryWithCovariance& odom_wc, sensor_msgs::msg::NavSatFix& msg);
