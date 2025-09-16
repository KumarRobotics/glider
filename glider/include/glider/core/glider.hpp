/*
* Jason Hughes
* July 2025
*
* main pose graph code
*/

#pragma once

#include <cmath>
#include <Eigen/Dense>
#include <gtsam/geometry/Pose3.h>

#include "glider/core/factor_manager.hpp"
#include "glider/utils/geodetics.hpp"
#include "glider/utils/calibration.hpp"

namespace Glider
{

class Glider
{
    public:
        Glider() = default;
        Glider(const std::string& path, const std::string& cpath);

        void addGPS(int64_t timestamp, Eigen::Vector3d& gps);
        void addIMU(int64_t timestamp, Eigen::Vector3d& accel, Eigen::Vector3d& gyro, Eigen::Vector4d& quat);
        void addOdom(int64_t timestamp, Eigen::Isometry3d& pose);
        void addMagnetometer(int64_t timestamp, double heading);

        Odometry interpolate(int64_t timestamp);
        State optimize();
        
    private:

        FactorManager factor_manager_;
        HeadingCalibrator calibrator_;

        double origin_x_;
        double origin_y_;
        double initial_heading_;
        double current_heading_;
        double heading_diff_;
        bool set_initial_heading_;
        bool correct_imu_;
        bool calibrate_;
        std::string frame_;
        Eigen::Vector3d t_imu_gps_;

        Eigen::Matrix3d ned_to_enu_rot_;
        Eigen::Quaterniond ned_to_enu_quat_;

        Eigen::Isometry3d prev_pose_;

        gtsam::Pose3 isometryToPose(const Eigen::Isometry3d& iso);
        double northEastToEastNorth(double heading_ne);
        Eigen::Vector4d correctImuOrientation(const Eigen::Vector4d orient);
        Eigen::Vector3d bodyENUToENUBody(const Eigen::Vector3d& meas);
};
}
