/* Jason Hughes
 * May 2025
 *
 */
#include <mutex>

#include "glider/core/factor_manager.hpp"
#include "glider/utils/geodetics.hpp"

#include <gtsam/slam/expressions.h>

using namespace Glider;

std::mutex Glider::FactorManager::mutex_;

FactorManager::FactorManager(const Parameters& params)
{
    // set initialization status
    imu_initialized_ = false;
    gps_initialized_ = false;
    sys_initialized_ = false;

    // setup parameters
    params_ = params;
    imu_params_ = defaultImuParams(params.gravity);


    // imu initialization
    init_counter_ = 0;
    bias_estimate_vec_ = Eigen::MatrixXd::Zero(params.bias_num_measurements, 6);
    gravity_vec_ = Eigen::Vector3d(0.0, 0.0, params.gravity);

    // set noise model
    gps_noise_ = gtsam::noiseModel::Isotropic::Sigma(3, params.gps_noise);
    orient_noise_ = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(params.roll_pitch_cov, params.roll_pitch_cov, 100));
    dgpsfm_noise_ = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(100, 100, params.dgpsfm_cov));
    
    // set key index
    key_index_ = 0;

    // setup graph
    optimized_count_ = 0;
    isam_params_ = gtsam::ISAM2Params();
    isam_params_.setRelinearizeThreshold(0.1);
    isam_params_.relinearizeSkip = 1;
    isam_ = gtsam::ISAM2(isam_params_);
    smoother_ = gtsam::IncrementalFixedLagSmoother(params_.lag_time, isam_params_);

    LOG(INFO) << "[GLIDER] Factor Manager initialzed";
}

boost::shared_ptr<gtsam::PreintegrationCombinedParams> FactorManager::defaultImuParams(double g)
{
    boost::shared_ptr<gtsam::PreintegrationCombinedParams> params;
    params = gtsam::PreintegrationCombinedParams::MakeSharedU(g);
    double gyro_sigma = (0.5 * M_PI / 180.0) / 60.0;
    double accel_sigma = 0.001;
    
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();

    params->setGyroscopeCovariance(std::pow(gyro_sigma, 2) * I);
    params->setAccelerometerCovariance(std::pow(accel_sigma, 2) * I);
    params->setIntegrationCovariance(std::pow(0.0000001, 2) * I);

    return params;
}

void FactorManager::initializeImu(const Eigen::Vector3d& accel_meas, const Eigen::Vector3d& gyro_meas, const Eigen::Vector4d& orient) 
{
    if (init_counter_ < params_.bias_num_measurements)
    {
        // the measurement to bias matrix
        bias_estimate_vec_.row(init_counter_).head(3) = accel_meas - gravity_vec_;
        bias_estimate_vec_.row(init_counter_).tail(3) = gyro_meas;
        init_counter_++;
    }
    else
    {
        Eigen::VectorXd bias_mean = bias_estimate_vec_.colwise().mean();
        bias_ = gtsam::imuBias::ConstantBias(Eigen::Vector3d(bias_mean.head(3)),
                                             Eigen::Vector3d(bias_mean.tail(3)));

        // initialize the pim
        pim_ = std::make_shared<gtsam::PreintegratedCombinedMeasurements>(imu_params_, bias_);
        // initialize the orientation
        initial_orientation_ = gtsam::Rot3::Quaternion(orient(0), orient(1), orient(2), orient(3));
        // initialize the graph once the imu is initialized
        initializeGraph();

        imu_initialized_ = true;
        LOG(INFO) << "[GLIDER] IMU initalized";
    }
}

void FactorManager::initializeGraph() 
{
    initials_ = gtsam::InitializePose3::initialize(graph_);
}

void FactorManager::addGpsFactor(int64_t timestamp, const Eigen::Vector3d& gps) 
{
    // wait until the imu is initialized
    if (!imu_initialized_) return;
    
    double time = nanosecIntToDouble(timestamp);

    if (key_index_ == 0)
    {
        // set the initial NavState 
        // The initial orientation is the the initial orientation from the imu initialization
        // The initial position is from the gps 
        // Initial velocity is set to zero
        gtsam::Pose3 initial_pose = gtsam::Pose3(initial_orientation_, gtsam::Point3(gps(0), gps(1), gps(2)));
        gtsam::NavState initial_navstate(initial_pose, gtsam::Point3(0.0, 0.0, 0.0)); // TODO why do I need this??
        
        // save the initial values
        initials_.insert(X(key_index_), initial_pose);
        initials_.insert(V(key_index_), gtsam::Point3(0.0, 0.0, 0.0));
        initials_.insert(B(key_index_), bias_);

        // save the timestamps for the smoother
        smoother_timestamps_[X(key_index_)] = time;
        smoother_timestamps_[V(key_index_)] = time;
        smoother_timestamps_[B(key_index_)] = time;

        // add prior factors to the graph
        graph_.add(gtsam::PriorFactor<gtsam::Pose3>(X(key_index_), initial_pose, gtsam::noiseModel::Isotropic::Sigma(6, 0.001)));
        graph_.add(gtsam::PriorFactor<gtsam::Point3>(V(key_index_), gtsam::Point3(0.0, 0.0, 0.0), gtsam::noiseModel::Isotropic::Sigma(3, 0.001)));
        graph_.add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(B(key_index_), bias_, gtsam::noiseModel::Isotropic::Sigma(6, 0.001)));

        key_index_++;
        gps_initialized_ = true;
        LOG(INFO) << "[GLIDER] GPS Initialized";
        return;
    }
   
    // add the pim to the graph under a mutex
    std::unique_lock<std::mutex> lock(mutex_);
    graph_.add(gtsam::CombinedImuFactor(X(key_index_-1), V(key_index_-1), X(key_index_), V(key_index_), B(key_index_-1), B(key_index_), *pim_));
    lock.unlock();

    // insert new initial values
    initials_.insert(X(key_index_), current_state_.getPose<gtsam::Pose3>());
    initials_.insert(V(key_index_), current_state_.getVelocity<gtsam::Vector3>());
    initials_.insert(B(key_index_), bias_);

    // save the time for the smoother
    smoother_timestamps_[X(key_index_)] = time;
    smoother_timestamps_[V(key_index_)] = time;
    smoother_timestamps_[B(key_index_)] = time;
    
    // convert eigen to gtsam
    gtsam::Point3 meas(gps(0), gps(1), gps(2));
    gtsam::Rot3 rot = gtsam::Rot3::Quaternion(orient_(0), orient_(1), orient_(2), orient_(3));

    // add gps measurement to factor graph as gtsam object
    graph_.add(gtsam::GPSFactor(X(key_index_), gps, gps_noise_));
    graph_.addExpressionFactor(gtsam::rotation(X(key_index_)), rot, orient_noise_);
 
    // increment key index
    key_index_++;
}

void FactorManager::addGpsFactor(int64_t timestamp, const Eigen::Vector3d& gps, const double& heading, const bool fuse) 
{
    // wait until the imu is initialized
    if (!imu_initialized_) return;
    
    double time = nanosecIntToDouble(timestamp);

    if (key_index_ == 0)
    {
        // set the initial NavState 
        // The initial orientation is the the initial orientation from the imu initialization
        // The initial position is from the gps 
        // Initial velocity is set to zero
        gtsam::Pose3 initial_pose = gtsam::Pose3(initial_orientation_, gtsam::Point3(gps(0), gps(1), gps(2)));
        gtsam::NavState initial_navstate(initial_pose, gtsam::Point3(0.0, 0.0, 0.0)); // TODO why do I need this??
        
        // save the initial values
        initials_.insert(X(key_index_), initial_pose);
        initials_.insert(V(key_index_), gtsam::Point3(0.0, 0.0, 0.0));
        initials_.insert(B(key_index_), bias_);

        // save the timestamps for the smoother
        smoother_timestamps_[X(key_index_)] = time;
        smoother_timestamps_[V(key_index_)] = time;
        smoother_timestamps_[B(key_index_)] = time;

        // add prior factors to the graph
        graph_.add(gtsam::PriorFactor<gtsam::Pose3>(X(key_index_), initial_pose, gtsam::noiseModel::Isotropic::Sigma(6, 0.001)));
        graph_.add(gtsam::PriorFactor<gtsam::Point3>(V(key_index_), gtsam::Point3(0.0, 0.0, 0.0), gtsam::noiseModel::Isotropic::Sigma(3, 0.001)));
        graph_.add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(B(key_index_), bias_, gtsam::noiseModel::Isotropic::Sigma(6, 0.001)));

        key_index_++;
        gps_initialized_ = true;
        LOG(INFO) << "[GLIDER] GPS Initialized";
        return;
    }
   
    // add the pim to the graph under a mutex
    std::unique_lock<std::mutex> lock(mutex_);
    graph_.add(gtsam::CombinedImuFactor(X(key_index_-1), V(key_index_-1), X(key_index_), V(key_index_), B(key_index_-1), B(key_index_), *pim_));
    lock.unlock();

    // insert new initial values
    initials_.insert(X(key_index_), current_state_.getPose<gtsam::Pose3>());
    initials_.insert(V(key_index_), current_state_.getVelocity<gtsam::Vector3>());
    initials_.insert(B(key_index_), bias_);

    // save the time for the smoother
    smoother_timestamps_[X(key_index_)] = time;
    smoother_timestamps_[V(key_index_)] = time;
    smoother_timestamps_[B(key_index_)] = time;
    
    // add gps measurement to factor graph as gtsam object
    gtsam::Point3 meas(gps(0), gps(1), gps(2));
    gtsam::Rot3 rot = gtsam::Rot3::Ypr(heading, 0.0, 0.0);
    //gtsam::Rot3 imu_rot = gtsam::Rot3::Quaternion(orient_(0), orient_(1), orient_(2), orient_(3));
    //graph_.addExpressionFactor(gtsam::rotation(X(key_index_)), imu_rot, orient_noise_);

    graph_.add(gtsam::GPSFactor(X(key_index_), gps, gps_noise_));
    if (fuse) graph_.addExpressionFactor(gtsam::rotation(X(key_index_)), rot, dgpsfm_noise_);

    // increment key index
    key_index_++;
}


void FactorManager::addImuFactor(int64_t timestamp, const Eigen::Vector3d& accel, const Eigen::Vector3d& gyro, const Eigen::Vector4d& orient) 
{
    // if the imu is not initialized, pass the meaurements to initialize it
    if (!imu_initialized_)
    {
        last_imu_time_ = nanosecIntToDouble(timestamp);
        initializeImu(accel, gyro, orient);
        return;
    }
    // if the imu is initialzied we want to add measurements to the pim
    double current_time = nanosecIntToDouble(timestamp);
    double dt = current_time - last_imu_time_;
    if (dt <= 0.0)
    {   
        LOG(WARNING) << "[GLIDER] Recieved IMU measurement out of order, ignoring";
        return;
    }
    // both the runner and the add imu access the pim in different threads
    // so we need to lock it when we manipulate it
    std::lock_guard<std::mutex> lock(mutex_);
    
    pim_->integrateMeasurement(accel, gyro, dt);
    orient_ = orient;

    last_imu_time_ = current_time;
}

void FactorManager::addLandmarkFactor(int64_t timestamp, size_t landmark_id, const Eigen::Vector3d& utm, const Eigen::Matrix3d& cov)
{
    Eigen::Matrix3d obs_info = cov.inverse();
    auto it = landmark_info_.find(landmark_id);
    if (it == landmark_info_.end()) {
        landmark_info_[landmark_id] = obs_info;
        landmark_info_vec_[landmark_id] = obs_info * utm;
    } else {
        it->second += obs_info;
        landmark_info_vec_[landmark_id] += obs_info * utm;
    }
}

PointWithCovariance FactorManager::getLandmarkPoint(size_t landmark_id) const
{
    auto it = landmark_info_.find(landmark_id);
    if (it == landmark_info_.end()) return PointWithCovariance();
    
    Eigen::Matrix3d cov = it->second.inverse();
    Eigen::Vector3d point = cov * landmark_info_vec_.at(landmark_id);
    return PointWithCovariance(point, cov);
}

Odometry FactorManager::predict(int64_t timestamp)
{
    // TODO update this.
    //return Odometry::Uninitialized();
    if (sys_initialized_ && pim_)
    {
        gtsam::NavState result = pim_->predict(current_state_.getNavState(), bias_);
        return Odometry(result, timestamp, true);
    }
    else
    {
        return Odometry::Uninitialized();
    }
}


gtsam::Values FactorManager::optimize() 
{
    isam_.update(graph_, initials_);
    gtsam::Values result;
    // call the specified optimizer
    if (params_.smooth)
    {
        smoother_.update(graph_, initials_, smoother_timestamps_);
        result = smoother_.calculateEstimate();
    }
    else
    {
        result = isam_.calculateEstimate();
    }
    optimized_count_++;
    // if weve optimized the specified number of times, initialize the system
    if (optimized_count_ == params_.initial_num_measurements)
    {
        LOG(INFO) << "[GLIDER] System Initialized";
        sys_initialized_ = true;
    }

    return result;
}

OdometryWithCovariance FactorManager::runner(int64_t timestamp) 
{
    // if the graph or imu is not initialized we cannot optimize
    // so we return an uninitialized state
    if (!imu_initialized_ || !gps_initialized_) return OdometryWithCovariance::Uninitialized();

    gtsam::Values result = optimize();

    // get the covariance from isam or the smoother
    gtsam::Matrix pose_cov, vel_cov;
    if (params_.smooth)
    {
        pose_cov = smoother_.marginalCovariance(X(key_index_-1));
        vel_cov = smoother_.marginalCovariance(X(key_index_-1));
    }
    else
    {
        pose_cov = isam_.marginalCovariance(X(key_index_-1));
        vel_cov = isam_.marginalCovariance(V(key_index_-1));
    }
    // save the current state we just optimized for
    current_state_ = OdometryWithCovariance(result, timestamp, key_index_-1, pose_cov, vel_cov, true);

    // reset the pim
    pim_->resetIntegration();
    // reset the graph
    initials_.clear();
    smoother_timestamps_.clear();
    graph_.resize(0);

    // we want to optimize a few times before
    // publishing to allow convergence
    // otherwise we return an unitialized state
    if (!sys_initialized_) return OdometryWithCovariance::Uninitialized();

    return current_state_;
}

gtsam::ExpressionFactorGraph FactorManager::getGraph()
{
    return graph_;
}

bool FactorManager::isSystemInitialized() const
{
    return sys_initialized_;
}

bool FactorManager::isImuInitialized() const
{
    return imu_initialized_;
}

bool FactorManager::isGpsInitialized() const
{
    return gps_initialized_;
}

Eigen::MatrixXd FactorManager::getBiasEstimate() const
{
    return bias_estimate_vec_;
}

gtsam::PreintegratedCombinedMeasurements FactorManager::getPim() const
{
    return *pim_;
}

gtsam::Key FactorManager::getKeyIndex() const
{
    return key_index_;
}
