/*
 * Jason Hughes
 * April 2025
 *
 * This manages everything with the factor graph. It adds measurements,
 * runs the optimization with the smoother or isam and predicts with the pim
*/

#pragma once

#include <glog/logging.h>

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

#include "odometry_with_covariance.hpp"
#include "odometry.hpp"
#include "point.hpp"
#include "glider/utils/parameters.hpp"
#include "glider/utils/time.hpp"

#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <map>
#include <mutex>
#include <cmath>
#include <numeric>
#include <tuple>

// Symbol shorthand
using gtsam::symbol_shorthand::B; // Bias
using gtsam::symbol_shorthand::V; // Velocity
using gtsam::symbol_shorthand::X; // Pose

namespace Glider
{

class FactorManager
{
    public:
        // Constructos
        /*! @brief default constructor */
        FactorManager() = default;
        /*! @brief constructor that initalizes all parameters in the
         * factor manager
         * @param params: the parameters loaded from the yaml file*/
        FactorManager(const Parameters& params);
        /*! @brief initializes all parameters in the factor manager
         * @param params: the parameters loaded from the yaml file*/
        void initialize(const Parameters& params);

        // state predictors
        /*! @brief calls the pim predict method
         *  @param timestamp: the time at which this method is being called
         *  @return the odometry from the pim prediction */
        Odometry predict(int64_t timestamp);
        /*! @brief the runner takes care of everything with the optimization, it calls the
         *  optimizer, and resets everything after optimization is done
         *  @param timestamp: time at which the runner is called
         *  @return the odometry from the optimization over the graph with covariance
         *  estimates from the optimizer */
        OdometryWithCovariance runner(int64_t timestamp);

        // measurements adders
        /*! @brief adds the gps measurement and pim to the factor graph
         *  @param timestamp: time of the gps measurement
         *  @param gps: GPS measurement in the UTM frame */
        void addGpsFactor(int64_t timestamp, const Eigen::Vector3d& gps, const double sigma = 0.0);
        /*! @brief adds the gps measurement and a heading from dgps
         *  @param timestamp: time of the gps measurement
         *  @param gps: GPS measurement in the UTM frame
         *  @param heading: heading from dgpsfm in the ENU frame
         *  @param fuse: whether or not to add the heading measurement
         *  to the factor graph
         *  @param sigma: the standard deviation of the gps measurement, if 0 use param */
        void addGpsFactor(int64_t timestamp, const Eigen::Vector3d& gps, const double& heading, const bool fuse,
                          const double sigma = 0.0, const double heading_sigma = 0.0);
        /*! @brief adds the imu measurements to the pim and saves the orientation
         *  @param timestamp: time of the imu measurement
         *  @param accel: the accelerometer reading
         *  @param gyro: gyroscopre reading
         *  @param orient: orientation in quaternion (w,x,y,z) format */
        void addImuFactor(int64_t timestamp, const Eigen::Vector3d& accel, const Eigen::Vector3d& gyro, const Eigen::Vector4d& orient);
        /*! @brief adds an odometry measurement
         *  @param timestamp: time of the odometry
         *  @param odom: estimated odometry pose
         *  @return true if a graph node was created */
        bool addOdomFactor(int64_t timestamp, const Eigen::Isometry3d& odom,
                           const Eigen::Vector3d& velocity = Eigen::Vector3d::Zero(),
                           double velocity_sigma = 1.0);
        /*! @brief adds a landmark factor for an estimated utm point and covariance
         *  @param timestamp: time of the landmark measurements
         *  @param landmark_id: a unique id for the landmark
         *  @param utm: the estimated utm coordinate of the landmark
         *  @param cov: the 3x3 covariance matrix for the estimated utm coordinate of the landmark */
        void addLandmarkFactor(int64_t timestamp, size_t landmark_id, const Eigen::Vector3d& utm, const Eigen::Matrix3d& cov);


        // getters and checkers
        /*! @brief gets the estimated landmark utm coordinate and covariance
         *  @param landmark_id: the uinque id for the landmark
         *  @return the estimated utm point and covariance */
        PointWithCovariance getLandmarkPoint(size_t landmark_id) const;
        /*! @brief gets the complete factor graph */
        gtsam::ExpressionFactorGraph getGraph();
        /*! @brief checks if the imu has been initialized
         *  @return true if imu bias calibration is complete else false*/
        bool isImuInitialized() const;
        /*! @brief checks if the gps is initialized
         *  @return true if the gps reading has been added to the graph else false */
        bool isGpsInitialized() const;
        /*! @brief checks if the odometry system is initialized
         *  @return true if graph has been optimized more than specified
         *  number of times else false */
        bool isSystemInitialized() const;
        /*! @brief gets the matrix used for bias estimation
         *  @return 6-by-bias_num_measurements matrix */
        Eigen::MatrixXd getBiasEstimate() const;
        /*! @brief gets the current pim object
         *  @return the current pim object dereferenced */
        gtsam::PreintegratedCombinedMeasurements getPim() const;
        /*! @brief gets the key index
         *  @return the current key index */
        gtsam::Key getKeyIndex() const;
        /*! @brief gets the gps offset
         *  @return the 3D gps offset */
        Eigen::Vector3d getGpsOffset() const;
        /*! @brief checks if the gps offset has been initialized
         *  @return true if the gps offset has been initialized else false */
        bool isGpsOffsetInitialized() const;

        /*! @brief gets the current parameters
         *  @return the current parameters */
        const Parameters& params() const { return params_; }

    private:
        /*! @brief handles the optimization call with the specified
         *  optimizer, either isam or fixed lag smoother
         *  @return the output estimates from the optimization */
        gtsam::Values optimize();

        /*! @brief initializes all the parameters for the pim
         *  @param g: gravity as defined in the yaml config
         *  @return pim parameters as a shared_ptr */
        boost::shared_ptr<gtsam::PreintegrationCombinedParams> defaultImuParams(double g);

        /*! @brief helper function that sets initial values in the graph */
        void initializeGraph();
        /*! @brief estiamtes the bias using the specified number of measurements
         *  up initialization, and saves the orientation as the initial orientation
         *  @param accel_meas: accelerometer measurement
         *  @param gytro_meas: gytroscop measurement
         *  @param orient: the 3D orientation of the robot as a quaternion from the imu*/
        void initializeImu(const Eigen::Vector3d& accel_meas, const Eigen::Vector3d& gyro_meas, const Eigen::Vector4d& orient);

        // @brief a mutex to use accross function that access the pim
        // as the pim could be accessd by multiple threads
        static std::mutex mutex_;
        // @brief a mutex to protect the factor graph and its variables
        mutable std::mutex graph_mutex_;

        // parameters
        // @brief parameters for the isam2 optimizer
        gtsam::ISAM2Params isam_params_;
        // @brief parameters for the pim
        boost::shared_ptr<gtsam::PreintegrationCombinedParams> imu_params_;
        // @brief parameters set in the config file
        Parameters params_;

        // IMU
        // @brief number of measurements used in initialization so far
        // stops incrementing when it reaches bias_num_measurements
        int init_counter_;
        // @brief saves the orientation from the imu
        Eigen::Vector4d orient_;
        // @brief a gravity vector (0,0,+/-9.81) depending on frame
        Eigen::Vector3d gravity_vec_;
        // @brief 6-by-bias_num_measurements matrix to store measurements
        // from accel and gyro to measurem bias
        Eigen::MatrixXd bias_estimate_vec_;

        // @brief saves the bias estimate from gtsam optimization
        gtsam::imuBias::ConstantBias bias_;
        // @brief the pim for imu measurements
        std::shared_ptr<gtsam::PreintegratedCombinedMeasurements> pim_;

        // noise
        // @brief noise on the prior estimate
        gtsam::noiseModel::Isotropic::shared_ptr prior_noise_;
        // @brief noise on the gps position estimate
        gtsam::noiseModel::Isotropic::shared_ptr gps_noise_;
        // @brief noise in the orientation estimate from the imu
        gtsam::noiseModel::Base::shared_ptr orient_noise_;
        // @brief noise in the heading estimate of differential gps
        gtsam::noiseModel::Base::shared_ptr dgpsfm_noise_;
        // @brief noise for the odometry constraints
        gtsam::noiseModel::Diagonal::shared_ptr odom_noise_;

        // factor graph
        // @brief tracks the number of times the optimizer has been called
        uint64_t optimized_count_;
        // @brief the factor graph
        gtsam::ExpressionFactorGraph graph_;
        // @breif stores all initial values for the optimizer
        gtsam::Values initials_;
        // @brief the current key index of the factor graph
        gtsam::Key key_index_;
        // @brief the fixed lag smoother optimizer
        gtsam::IncrementalFixedLagSmoother smoother_;
        // @brief the timestamp tracker for the fixed lag smoother
        gtsam::FixedLagSmoother::KeyTimestampMap smoother_timestamps_;
        // @brief the isam2 optimizer, can be used in lue of the smoother
        gtsam::ISAM2 isam_;

        // @brief the timestamp of the previous imu measurement
        double last_imu_time_;

        // track states
        // @brief the current state estimate from the optimizer
        OdometryWithCovariance current_state_;

        // initial state
        // @brief the initial orientation from the IMU
        gtsam::Rot3 initial_orientation_;

        // initialization
        // @brief tracks odometry system initializtion
        // true when we optimize more than the specified number of times
        bool sys_initialized_;
        // @param tracks if the imu bias estimate is complete
        bool imu_initialized_;
        // @param tracks if a gps measurement has been received
        bool gps_initialized_;
        // @param tracks if an odom measurement has been received
        bool odom_initialized_;
        // @param true if the first graph node originated from Odometry (LIO), making it a local origin
        bool using_local_origin_;
        bool gps_offset_initialized_;
        Eigen::Vector3d gps_offset_;
        double last_node_time_;
        // @param previous odometry measurement for relative constraints
        Eigen::Isometry3d last_odom_meas_;
        // @param accumulated odometry transform between nodes
        Eigen::Isometry3d accumulated_odom_delta_;

        // landmark variables
        std::unordered_map<size_t, Eigen::Matrix3d> landmark_info_;
        std::unordered_map<size_t, Eigen::Vector3d> landmark_info_vec_;
};
}
