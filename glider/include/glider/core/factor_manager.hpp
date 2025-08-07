/*
* April 2025
* Manage the Factor Graph
*/

#pragma once

#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/geometry/Similarity3.h>
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

#include "imu_buffer.hpp"
#include "state.hpp"
#include "odometry.hpp"
#include "glider/utils/parameters.hpp"

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
using gtsam::symbol_shorthand::S; // scale
using gtsam::symbol_shorthand::R; // Rotation

// Helper function declarations
Eigen::Vector3d vector3(double x, double y, double z);
double nanosecInt2Float(int64_t timestamp);

namespace Glider 
{

class FactorManager
{
    public:
        FactorManager() = default;
        FactorManager(const Parameters& params);    
        
        static boost::shared_ptr<gtsam::PreintegrationCombinedParams> defaultImuParams(double g);
        
        void initializeGraph();
        void imuInitialize(const Eigen::Vector3d& accel_meas, const Eigen::Vector3d& gyro_meas, const Eigen::Vector4d& orient);

        Odometry predict(int64_t timestamp); 

        void addGpsFactor(int64_t timestamp, const Eigen::Vector3d& gps);
        void addOdometryFactor(int64_t timestamp, const Eigen::Vector3d& pose, const Eigen::Vector4d& quat);
        void addOdometryFactor(int64_t timestamp, const gtsam::Pose3& pose);
        void addImuFactor(int64_t timestamp, const Eigen::Vector3d& accel, const Eigen::Vector3d& gyro, const Eigen::Vector4d& orient);
        void addHeadingFactor(int64_t timestamp, const double& heading);

        gtsam::Values optimize();  
        State runner();

        gtsam::ExpressionFactorGraph getGraph();
        double getInitialHeading() const;
        bool isInitialized();

    private:
        // parameters
        std::map<std::string, Eigen::MatrixXd> matrix_config_;
        gtsam::ISAM2Params isam_params_;
        boost::shared_ptr<gtsam::PreintegrationCombinedParams> imu_params_;
        Parameters params_;

        // IMU
        Eigen::Vector3d gravity_vec_;
        Eigen::MatrixXd bias_estimate_vec_;
        Eigen::Matrix3d imu2body_;
        gtsam::Matrix3 NED2ENU;
        Eigen::Vector4d orient_;
        Eigen::Vector3d gyro_;
        int init_counter_;
        
        gtsam::imuBias::ConstantBias bias_;
        boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements> pim_;
        boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements> pim_copy_;

        ImuBuffer imu_buffer_;

        // noise
        gtsam::noiseModel::Isotropic::shared_ptr prior_noise_;
        gtsam::noiseModel::Base::shared_ptr odom_noise_;
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
        gtsam::Matrix last_marginal_covariance_;
        gtsam::Pose3 last_odom_;
        double last_optimize_time_;
        double last_imu_time_;
        double last_gps_time_;
        double estimated_scale_;
        State last_state_;

        // current state
        State current_state_;
        double current_heading_;

        // initialization
        bool initialized_;
        bool compose_odom_;
        bool sim3_prior_added_; // dep
        uint8_t start_odom_; // dep
        uint8_t heading_count_; // dep
        gtsam::Rot3 initial_orientation_;
        gtsam::Pose3 initial_pose_for_odom_;
        gtsam::NavState initial_navstate_;
};
}
