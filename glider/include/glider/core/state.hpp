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

using gtsam::symbol_shorthand::B; // Bias
using gtsam::symbol_shorthand::V; // Velocity
using gtsam::symbol_shorthand::X; // Pose

namespace glider
{

struct State
{
    public:
        State() = default;
        State(gtsam::Values& val, gtsam::Key key, gtsam::Matrix& pose_cov, gtsam::Matrix& velocity_cov);
        State(gtsam::Values& val);
        
        static State Zero();

        Eigen::Vector3d velocity;
        Eigen::Vector3d position;
        Eigen::Vector4d orientation;
        Eigen::MatrixXd pose_covariance;
        Eigen::MatrixXd position_covariance;
        Eigen::MatrixXd velocity_covariance;
        Eigen::Vector3d accelerometer_bias;
        Eigen::Vector3d gyroscope_bias;

        int key_index;

        double altitude;
        double heading;

        double getHeadingDegrees() const;
        double getLatitude(const char* zone);
        double getLongitude(const char* zone);
        std::pair<double, double> getLatLon(const char* zone);

        bool isMoving() const;
        bool isInitialized() const;

        void setLatitude(const double lat);
        void setLongitude(const double lon);
        void setMovingStatus(const bool status);
        void setInitializedStatus(const bool status);

    private:
        double latitude;
        double longitude;
        
        bool is_moving;
        bool is_initialized;
};
} // namespace glider
