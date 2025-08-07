/*
* Jason Hughes
* May 2025
*
* Struct to keep track of robot state
*/

#pragma once

#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/navigation/ImuBias.h>

#include <Eigen/Dense>
#include <utility>

#include "odometry.hpp"

using gtsam::symbol_shorthand::B; // Bias
using gtsam::symbol_shorthand::V; // Velocity
using gtsam::symbol_shorthand::X; // Pose

namespace Glider
{


class State : public Odometry
{
    public:
        State() = default;
        State(gtsam::Values& val, gtsam::Key key, double scale, gtsam::Matrix& pose_cov, gtsam::Matrix& velocity_cov, bool initialized = true);
        State(gtsam::Values& val, bool initialized = true);
        
        static State Uninitialized();

        template<typename T>
        T getBias() const;
        template<typename T>
        T getAccelerometerBias() const;
        template<typename T>
        T getGyroscopeBias() const;
        template<typename T>
        T getKeyIndex() const;
        
        Eigen::MatrixXd getPoseCovariance() const;
        Eigen::MatrixXd getPositionCovariance() const;
        Eigen::MatrixXd getVelocityCovariance() const;
        
        std::string getKeyIndex(const char* symbol);

        bool isMoving() const;

    private:

        bool is_moving_;
        
        gtsam::Vector3 accelerometer_bias_;
        gtsam::Vector3 gyroscope_bias_;

        gtsam::Matrix pose_covariance_;
        gtsam::Matrix position_covariance_;
        gtsam::Matrix velocity_covariance_;

        gtsam::Key key_index_;
};
} // namespace glider
