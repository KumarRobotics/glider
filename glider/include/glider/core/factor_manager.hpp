/*
* April 2025
* Manage the Factor Graph
*/

#pragma once

#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/ExpressionFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam_unstable/nonlinear/FixedLagSmoother.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/slam/InitializePose3.h>
#include <gtsam/slam/RangeFactor.h>

#include "imu_buffer.hpp"
#include "state.hpp"

#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <map>
#include <cmath>
#include <numeric>
#include <tuple>

// Symbol shorthand
using gtsam::symbol_shorthand::B; // Bias
using gtsam::symbol_shorthand::V; // Velocity
using gtsam::symbol_shorthand::X; // Pose
using gtsam::symbol_shorthand::R; // Rotation
using gtsam::symbol_shorthand::P; // point
using gtsam::symbol_shorthand::G; // gps
using gtsam::symbol_shorthand::T; // translation

// Helper function declarations
Eigen::Vector3d vector3(double x, double y, double z);
double nanosecInt2Float(int64_t timestamp);

namespace glider 
{

class FactorManager
{
    public:
        FactorManager() = default;
        FactorManager(const std::map<std::string, double>& config, int64_t start_time);
        
        static boost::shared_ptr<gtsam::PreintegrationCombinedParams> defaultParams(double g);
        
        void initializeGraph();
        void imuInitialize(const Eigen::Vector3d& accel_meas, const Eigen::Vector3d& gyro_meas, const Eigen::Vector4d& orient);

        std::tuple<Eigen::Vector3d, Eigen::Vector4d> predict(int64_t timestamp); 

        void addGpsFactor(int64_t timestamp, const Eigen::Vector3d& gps);
        void addOdometryFactor(int64_t timestamp, const Eigen::Vector3d& pose, const Eigen::Vector4d& quat);
        void addImuFactor(int64_t timestamp, const Eigen::Vector3d& accel, const Eigen::Vector3d& gyro, const Eigen::Vector4d& orient);
        void addHeadingFactor(int64_t timestamp, const double& heading);

        gtsam::Values optimize();  
        State runner();

        gtsam::ExpressionFactorGraph getGraph();
        bool isInitialized();

        template <typename T>
        T getKeyIndex();

    private:
        // parameters
        std::map<std::string, double> config_;
        std::map<std::string, Eigen::MatrixXd> matrix_config_;
        gtsam::ISAM2Params parameters_;
        boost::shared_ptr<gtsam::PreintegrationCombinedParams> params_;
        
        // IMU
        Eigen::Vector3d gravity_vec_;
        Eigen::MatrixXd bias_estimate_vec_;
        Eigen::Matrix3d imu2body_;
        
        int init_counter_;
        
        gtsam::imuBias::ConstantBias bias_;
        boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements> pim_;
        boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements> pim_copy_;

        ImuBuffer imu_buffer_;

        // noise
        gtsam::noiseModel::Isotropic::shared_ptr prior_noise_;
        gtsam::noiseModel::Isotropic::shared_ptr odom_noise_;
        gtsam::noiseModel::Isotropic::shared_ptr gps_noise_;
        gtsam::noiseModel::Base::shared_ptr heading_noise_;

        // factor graph
        gtsam::ExpressionFactorGraph graph_;
        gtsam::Values initials_;
        gtsam::Key key_index_;
        gtsam::IncrementalFixedLagSmoother smoother_;
        gtsam::FixedLagSmoother::KeyTimestampMap smoother_timestamps_;
        gtsam::ISAM2 isam_;

        // previous state
        Eigen::Vector3d last_gps_;
        gtsam::Point3 last_velocity_;
        gtsam::Matrix last_marginal_covariance_;
        gtsam::NavState last_nav_state_; 
        Eigen::Vector3d last_accel_meas_;
        Eigen::Vector3d last_gyro_meas_;
        Eigen::Vector4d last_imu_orientation_;
        gtsam::Pose3 last_optimized_pose_;
        gtsam::Pose3 last_odom_;
        double last_optimize_time_;
        double last_imu_time_;
        double last_gps_time_;

        // current state
        Eigen::Vector4d orientation_;
        Eigen::Vector3d translation_;
        gtsam::Pose3 current_optimized_pose_;
        gtsam::Pose3 current_navstate_pose_;
        gtsam::Point3 current_velocity_;
        double current_heading_;

        // initialization
        bool initialized_;
        bool compose_odom_;
        uint8_t start_odom_;
        uint8_t heading_count_;
        gtsam::Rot3 initial_orientation_;
        gtsam::Pose3 initial_pose_for_odom_;
        gtsam::NavState initial_navstate_;

        State current_state_;
        State last_state_;

        gtsam::Matrix3 NED2ENU;
};
}
