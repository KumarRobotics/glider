/*
* Jason Hughes
* May 2025
*
* Struct to keep track of the robots state
*/

#include "glider/core/state.hpp"
#include "glider/core/odometry.hpp"
#include "glider/utils/geodetics.hpp"

using namespace Glider;

State::State(gtsam::Values& vals, gtsam::Key key, double scale, gtsam::Matrix& pose_cov, gtsam::Matrix& velocity_cov, bool init) : Odometry(vals, key, scale, init)
{
    gtsam::imuBias::ConstantBias bias = vals.at<gtsam::imuBias::ConstantBias>(B(key));
        
    key_index_ = key;
    scale_ = scale;
    accelerometer_bias_ = bias.accelerometer();
    gyroscope_bias_ = bias.gyroscope();

    pose_covariance_ = pose_cov;
    velocity_covariance_ = velocity_cov;
    position_covariance_ = pose_cov.block<3,3>(0,0);

    is_moving_ = (velocity_.norm() > 0.01) ? true : false;

    initialized_ = init;
}

State State::Uninitialized()
{
    State state;
    state.setInitializedStatus(false);

    return state;
}

Eigen::MatrixXd State::getPoseCovariance() const
{
    return pose_covariance_;
}

Eigen::MatrixXd State::getPositionCovariance() const
{
    return position_covariance_;
}

Eigen::MatrixXd State::getVelocityCovariance() const
{
    return velocity_covariance_;
}

template<typename T>
T State::getBias() const
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
T State::getAccelerometerBias() const
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
T State::getGyroscopeBias() const
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
T State::getKeyIndex() const
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

std::string State::getKeyIndex(const char* symbol)
{
    if (std::strcmp(symbol,"x") || std::strcmp(symbol,"X"))
    {
        return gtsam::DefaultKeyFormatter(X(key_index_));
    }
    else if (std::strcmp(symbol,"b") || std::strcmp(symbol,"B"))
    {
        return gtsam::DefaultKeyFormatter(B(key_index_));
    }
    else if (std::strcmp(symbol,"v") || std::strcmp(symbol,"V"))
    {
        return gtsam::DefaultKeyFormatter(V(key_index_));
    }
    else
    {
        throw std::invalid_argument("unsupported symbol, use: X, B or V");
    }
}


bool State::isMoving() const
{
    return is_moving_;
}


template gtsam::imuBias::ConstantBias State::getBias<gtsam::imuBias::ConstantBias>() const;
template std::pair<Eigen::Vector3d, Eigen::Vector3d> State::getBias<std::pair<Eigen::Vector3d,Eigen::Vector3d>>() const;

template gtsam::Vector3 State::getAccelerometerBias<gtsam::Vector3>() const;
template gtsam::Vector3 State::getGyroscopeBias<gtsam::Vector3>() const;

template gtsam::Key State::getKeyIndex<gtsam::Key>() const;
template int State::getKeyIndex<int>() const;
