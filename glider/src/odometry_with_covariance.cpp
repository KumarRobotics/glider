/*
* Jason Hughes
* May 2025
*
* Struct to keep track of the robots state
*/

#include "glider/core/odometry_with_covariance.hpp"
#include "glider/core/odometry.hpp"
#include "glider/utils/geodetics.hpp"

using namespace Glider;

OdometryWithCovariance::OdometryWithCovariance(gtsam::Values& vals, int64_t timestamp, gtsam::Key key, gtsam::Matrix& pose_cov, gtsam::Matrix& velocity_cov, bool init) : Odometry(vals, timestamp, key, init)
{
    gtsam::imuBias::ConstantBias bias = vals.at<gtsam::imuBias::ConstantBias>(B(key));

    key_index_ = key;
    accelerometer_bias_ = bias.accelerometer();
    gyroscope_bias_ = bias.gyroscope();

    pose_covariance_ = pose_cov;
    velocity_covariance_ = velocity_cov;
    // GTSAM Pose3 tangent covariance is ordered [rotation, translation].
    position_covariance_ = pose_cov.block<3,3>(3,3);

    is_moving_ = (velocity_.norm() > 0.01) ? true : false;
    initialized_ = init;
    is_gps_offset_initialized_ = false;
}

OdometryWithCovariance OdometryWithCovariance::Uninitialized()
{
    OdometryWithCovariance odom;
    odom.setInitializedStatus(false);

    return odom;
}

Eigen::MatrixXd OdometryWithCovariance::getPoseCovariance() const
{
    return pose_covariance_;
}

Eigen::MatrixXd OdometryWithCovariance::getPositionCovariance() const
{
    return position_covariance_;
}

Eigen::MatrixXd OdometryWithCovariance::getVelocityCovariance() const
{
    return velocity_covariance_;
}

template<typename T>
T OdometryWithCovariance::getBias() const
{
    if constexpr (std::is_same_v<T, gtsam::imuBias::ConstantBias>)
    {
        return gtsam::imuBias::ConstantBias(accelerometer_bias_, gyroscope_bias_);
    }
    else if constexpr (std::is_same_v<T, std::pair<Eigen::Vector3d, Eigen::Vector3d>>)
    {
        Eigen::Vector3d ab = accelerometer_bias_;
        Eigen::Vector3d gb = gyroscope_bias_;

        return std::make_pair(ab, gb);
    }
    else
    {
        static_assert(std::is_same_v<T, gtsam::imuBias::ConstantBias> ||
                      std::is_same_v<T, std::pair<Eigen::Vector3d, Eigen::Vector3d>>, "unsupported type");
    }
}

// make the typing explicit on the gtsam::Vector3
template<typename T>
T OdometryWithCovariance::getAccelerometerBias() const
{
    if constexpr (std::is_same_v<T, gtsam::Vector3>)
    {
        return accelerometer_bias_;
    }
    else if constexpr (std::is_same_v<T, Eigen::Vector3d>)
    {
        Eigen::Vector3d bias = accelerometer_bias_;
        return bias;
    }
    else
    {
        static_assert(std::is_same_v<T, gtsam::Vector3> ||
                      std::is_same_v<T, Eigen::Vector3d>, "unsupported type");
    }
}

template<typename T>
T OdometryWithCovariance::getGyroscopeBias() const
{
    if constexpr (std::is_same_v<T, gtsam::Vector3>)
    {
        return gyroscope_bias_;
    }
    else if constexpr (std::is_same_v<T, Eigen::Vector3d>)
    {
        Eigen::Vector3d bias = gyroscope_bias_;
        return bias;
    }
    else
    {
        static_assert(std::is_same_v<T, gtsam::Vector3> ||
                      std::is_same_v<T, Eigen::Vector3d>, "unsupported type");
    }
}

template<typename T>
T OdometryWithCovariance::getKeyIndex() const
{
    if constexpr (std::is_same_v<T, gtsam::Key>)
    {
        return key_index_;
    }
    if constexpr (std::is_same_v<T, int>)
    {
        int i = static_cast<int>(gtsam::symbolIndex(key_index_));
        return i;
    }
}

std::string OdometryWithCovariance::getKeyIndex(const char* symbol)
{
    if (std::strcmp(symbol,"x") == 0 || std::strcmp(symbol,"X") == 0)
    {
        return gtsam::DefaultKeyFormatter(X(key_index_));
    }
    else if (std::strcmp(symbol,"b") == 0 || std::strcmp(symbol,"B") == 0)
    {
        return gtsam::DefaultKeyFormatter(B(key_index_));
    }
    else if (std::strcmp(symbol,"v") == 0 || std::strcmp(symbol,"V") == 0)
    {
        return gtsam::DefaultKeyFormatter(V(key_index_));
    }
    else
    {
        throw std::invalid_argument("unsupported symbol, use: X, B or V");
    }
}


bool OdometryWithCovariance::isMoving() const
{
    return is_moving_;
}

bool OdometryWithCovariance::isMovingFasterThan(const double vel) const
{
    double curr_vel_norm = velocity_.norm();
    return curr_vel_norm > vel;
}

template gtsam::imuBias::ConstantBias OdometryWithCovariance::getBias<gtsam::imuBias::ConstantBias>() const;
template std::pair<Eigen::Vector3d, Eigen::Vector3d> OdometryWithCovariance::getBias<std::pair<Eigen::Vector3d,Eigen::Vector3d>>() const;

template gtsam::Vector3 OdometryWithCovariance::getAccelerometerBias<gtsam::Vector3>() const;
template gtsam::Vector3 OdometryWithCovariance::getGyroscopeBias<gtsam::Vector3>() const;

template gtsam::Key OdometryWithCovariance::getKeyIndex<gtsam::Key>() const;
template int OdometryWithCovariance::getKeyIndex<int>() const;
