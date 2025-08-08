/* Jason Hughes
 * May 2025
 *
 */
#include <mutex>

#include "glider/core/factor_manager.hpp"
#include "glider/core/imu_buffer.hpp"
#include "glider/utils/geodetics.hpp"
#include "glider/utils/gps_heading.hpp"

#include <gtsam/slam/expressions.h>

using namespace Glider;

Eigen::Vector3d vector3(double x, double y, double z)
{
    return Eigen::Vector3d(x, y, z);
}

double nanosecInt2Float(int64_t timestamp)
{
    return timestamp * 1e-9;
}

FactorManager::FactorManager(const Parameters& params)
{
    bias_estimate_vec_ = Eigen::MatrixXd::Zero(params.bias_num_measurements, 6);
    init_counter_ = 0;
    imu2body_ = Eigen::Matrix3d::Identity();

    NED2ENU << 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0;

    heading_count_ = 0;
    initialized_ = false;
    sim3_prior_added_ = false;
    key_index_ = 0;
    last_optimize_time_ = 0.0;
    last_imu_time_ = 0.0;
    imu_params_ = defaultImuParams(params.gravity);
    gravity_vec_ = Eigen::Vector3d(0.0, 0.0, params.gravity);

    prior_noise_ = gtsam::noiseModel::Isotropic::Sigma(6, params.gps_noise); // maybe this can be odom noise??
    gps_noise_ = gtsam::noiseModel::Isotropic::Sigma(3, params.gps_noise);
    odom_noise_ = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector6(params.odom_orientation_noise, params.odom_orientation_noise, params.odom_orientation_noise, params.odom_translation_noise, params.odom_translation_noise, params.odom_orientation_noise)); //, params.odom_scale_noise));
    heading_noise_ = gtsam::noiseModel::Isotropic::Sigma(3, params.heading_noise);

    graph_ = gtsam::ExpressionFactorGraph();
    initials_ = gtsam::Values();

    isam_params_ = gtsam::ISAM2Params();
    isam_params_.setRelinearizeThreshold(0.1);
    isam_params_.relinearizeSkip = 1;

    smoother_ = gtsam::IncrementalFixedLagSmoother(params.lag_time, isam_params_);
    isam_ = gtsam::ISAM2(isam_params_);

    current_state_ = State::Uninitialized();

    imu_buffer_ = ImuBuffer(1000);
    compose_odom_ = false;
    start_odom_ = 0;
    estimated_scale_ = 1.0;

    params_ = params;

    std::cout << "[GLIDER-MONO] Factor Manager Initialized" << std::endl;
}

boost::shared_ptr<gtsam::PreintegrationCombinedParams> FactorManager::defaultImuParams(double g)
{
    auto params = gtsam::PreintegrationCombinedParams::MakeSharedD(g);
    double k_gyro_sigma = (0.5 * M_PI / 180.0) / 60.0;  // 0.5 degree ARW
    double k_accel_sigma = 0.001; // / 60.0;  // 10 cm VRW
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    params->setGyroscopeCovariance(std::pow(k_gyro_sigma, 2) * I);
    params->setAccelerometerCovariance(std::pow(k_accel_sigma, 2) * I);
    params->setIntegrationCovariance(std::pow(0.0000001, 2) * I);

    return params;
}

void FactorManager::imuInitialize(const Eigen::Vector3d& accel_meas, const Eigen::Vector3d& gyro_meas, const Eigen::Vector4d& orient) 
{
    if (init_counter_ < params_.bias_num_measurements) 
    {
        bias_estimate_vec_.row(init_counter_).head(3) = accel_meas + gravity_vec_;
        bias_estimate_vec_.row(init_counter_).tail(3) = gyro_meas;
        init_counter_++;
    }

    if (init_counter_ == params_.bias_num_measurements) 
    {
        Eigen::VectorXd bias_mean = bias_estimate_vec_.colwise().mean();
        bias_ = gtsam::imuBias::ConstantBias(
            Eigen::Vector3d(bias_mean.head(3)),
            Eigen::Vector3d(bias_mean.tail(3))
        );
        
        pim_ = boost::make_shared<gtsam::PreintegratedCombinedMeasurements>(imu_params_, bias_);
        pim_copy_ = boost::make_shared<gtsam::PreintegratedCombinedMeasurements>(imu_params_, bias_);
        
        // GTSAM expects quaternion as w,x,y,z
        gtsam::Rot3 rot = gtsam::Rot3::Quaternion(orient(0), orient(1), orient(2), orient(3));
        Eigen::Matrix3d init_orient_matrix = imu2body_ * rot.matrix();
        initial_orientation_ = gtsam::Rot3(init_orient_matrix);
        current_heading_ = initial_orientation_.yaw();
        double heading_deg = geodetics::headingRadiansToDegrees(current_heading_);
        std::cout << "[GLIDER-MONO] Initialized IMU with heading: " << heading_deg << std::endl;
        initialized_ = true;
    }
}

void FactorManager::addGpsFactor(int64_t timestamp, const Eigen::Vector3d& gps) 
{
    if (!initialized_) 
    {
        last_gps_ = gps;
        last_gps_time_ = nanosecInt2Float(timestamp);
        return;
    }
    
    Eigen::Vector3d meas = gps;
    
    double timestamp_f = nanosecInt2Float(timestamp);

    if (key_index_ == 0) 
    {
        // Add initials for IMU and GPS factors
        gtsam::Pose3 current_navstate_pose = gtsam::Pose3(initial_orientation_, gtsam::Point3(meas(0), meas(1), meas(2)));

        initial_navstate_ = gtsam::NavState(current_navstate_pose, gtsam::Point3(0, 0, 0));
        
        initials_.insert(X(key_index_), current_navstate_pose);
        initials_.insert(V(key_index_), gtsam::Point3(0, 0, 0));
        initials_.insert(B(key_index_), bias_);

        smoother_timestamps_[X(key_index_)] = timestamp_f;
        smoother_timestamps_[V(key_index_)] = timestamp_f;
        smoother_timestamps_[B(key_index_)] = timestamp_f;

        graph_.add(gtsam::PriorFactor<gtsam::Pose3>(X(key_index_), current_navstate_pose, gtsam::noiseModel::Isotropic::Sigma(6, 0.001)));
        graph_.add(gtsam::PriorFactor<gtsam::Point3>(V(key_index_), gtsam::Point3(0, 0, 0), gtsam::noiseModel::Isotropic::Sigma(3, 0.001)));
        graph_.add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(B(key_index_), bias_, gtsam::noiseModel::Isotropic::Sigma(6, 0.001)));
       
        key_index_++;

        return;
    }
                                  
    if (key_index_ > 0) 
    {
        static std::mutex imu_mutex;
        std::unique_lock<std::mutex> lock(imu_mutex);
        
        graph_.add(gtsam::CombinedImuFactor(X(key_index_),
                                            V(key_index_),
                                            X(key_index_-1),
                                            V(key_index_-1),
                                            B(key_index_),
                                            B(key_index_-1),
                                            *pim_copy_));
        pim_copy_->resetIntegration();
    
        initials_.insert(X(key_index_), current_state_.getPose<gtsam::Pose3>());
        initials_.insert(V(key_index_), current_state_.getVelocity<gtsam::Vector3>());
        initials_.insert(B(key_index_), bias_);

        smoother_timestamps_[X(key_index_)] = timestamp_f;
        smoother_timestamps_[V(key_index_)] = timestamp_f;
        smoother_timestamps_[B(key_index_)] = timestamp_f;

        lock.unlock();
    }

    gtsam::Rot3 imu_rot = gtsam::Rot3(orient_(0), orient_(1), orient_(2), orient_(3));
    graph_.addExpressionFactor(gtsam::rotation(X(key_index_)), imu_rot, heading_noise_);
    
    // Add gps factor in utm frame
    graph_.add(gtsam::GPSFactor(X(key_index_), gtsam::Point3(meas(0), meas(1), meas(2)), gps_noise_));
    
    if (initialized_ && compose_odom_)
    {
        // calculate scale in the 2D-space
        Eigen::Vector2d gps2d(gps(0), gps(1));
        Eigen::Vector2d last_gps2d(last_gps_(0), last_gps_(1));

        double gps_distance = (gps2d - last_gps2d).norm();
        Eigen::Vector2d last_odom2d(last_odom_.translation().x(), last_odom_.translation().y());

        double odom_distance = last_odom2d.norm();

        gtsam::Pose3 pose;
        if (params_.scale_odom)
        {
            if (odom_distance > 1e-6)
            {
                estimated_scale_ = gps_distance / odom_distance;
            }
            else 
            {
                std::cerr << "[GLIDER-MONO] Unable to estimate scale" << std::endl;
                return;
            }
            pose = gtsam::Pose3(last_odom_.rotation(), last_odom_.translation()*estimated_scale_);
        }
        else
        {
            pose = gtsam::Pose3(last_odom_.rotation(), last_odom_.translation());
        }
        graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(X(key_index_-1), X(key_index_), pose, odom_noise_));

        compose_odom_ = false;
    }

    // save timestamp, increment key
    last_gps_time_ = timestamp_f;
    last_gps_ = gps; 
    key_index_++;

    return;
}

void FactorManager::addOdometryFactor(int64_t timestamp, const gtsam::Pose3& pose)
{
    if (!initialized_) return;

    if (compose_odom_)
    {
        last_odom_ = last_odom_.compose(pose);
    }
    else
    {
        last_odom_ = pose;
        compose_odom_ = true;
    }
}


void FactorManager::addImuFactor(int64_t timestamp, const Eigen::Vector3d& accel, const Eigen::Vector3d& gyro, const Eigen::Vector4d& orient) 
{
    if (!initialized_) 
    {
        last_optimize_time_ = nanosecInt2Float(timestamp);
        last_imu_time_ = nanosecInt2Float(timestamp);
        imuInitialize(accel, gyro, orient);
        return;
    }
   
    double timestamp_f = nanosecInt2Float(timestamp);
    gtsam::Quaternion q(orient(0), orient(1), orient(2), orient(3));

    static std::mutex imu_mutex;
    std::lock_guard<std::mutex> lock(imu_mutex);
    //imu_buffer_.add(timestamp_f, accel, gyro, orient);

    //Eigen::Vector3d accel_meas = accel;
    //Eigen::Vector3d gyro_meas = gyro;
    orient_ = orient;
    double dt = nanosecInt2Float(timestamp) - last_imu_time_;
    if (dt <=0) return;
    
    pim_copy_->integrateMeasurement(accel, gyro, dt);
    gyro_ = gyro;
    last_imu_time_ = nanosecInt2Float(timestamp);

    return;
}

Odometry FactorManager::predict(int64_t timestamp)
{
    if (!pim_ || !current_state_.isInitialized())
    {
        return Odometry::Uninitialized();
    }
    gtsam::NavState result = pim_copy_->predict(current_state_.getNavState(), bias_);

    Odometry ret(result, gyro_);

    return ret; 
}

void FactorManager::initializeGraph() 
{
    if (!initialized_) 
    {
        return;
    }
    initials_ = gtsam::InitializePose3::initialize(graph_);
}

gtsam::Values FactorManager::optimize() 
{
    if (!initialized_) 
    {
        return gtsam::Values();
    }
     
    if (graph_.size() == 0 || initials_.size() == 0)
    {
        std::cout << "[GLIDER-MONO] No factors or initials, skipping optimization" << std::endl;

        initials_.clear();
        smoother_timestamps_.clear();
        graph_.resize(0);
        
        try
        {
            return smoother_.calculateEstimate();
        }
        catch (const std::exception& e)
        {
            std::cerr << "[GLIDER-MONO] No current estimate" << std::endl;
            return gtsam::Values();
        }
    }
    gtsam::Values result;
    if (key_index_ > 1)
    {
        //isam_.update(graph_, initials_);
        //result = isam_.calculateEstimate();
        
        smoother_.update(graph_, initials_, smoother_timestamps_);
        result = smoother_.calculateEstimate();

    }
    return result;
}

State FactorManager::runner() 
{
    if (!initialized_) 
    {
        State::Uninitialized();
    }
    if (key_index_ > 1)
    {
        //graph_.print();
        gtsam::Values result = optimize();
        last_state_ = current_state_;
       
        gtsam::Matrix pose_cov = smoother_.marginalCovariance(X(key_index_-1));
        gtsam::Matrix vel_cov = smoother_.marginalCovariance(V(key_index_-1));

        current_state_ = State(result, key_index_-1, estimated_scale_, pose_cov, vel_cov, true);
        
        pim_->resetIntegration();
        pim_copy_->resetIntegration();
 
        if (start_odom_ == 1)
        {
            std::cout << "[GLIDER-MONO] Starting to fuse odometry" << std::endl;
            initial_pose_for_odom_ = current_state_.getPose<gtsam::Pose3>();
            start_odom_ = 2;
        }
        
        initials_.clear();
        smoother_timestamps_.clear();
        graph_.resize(0);
        return current_state_;
    }
    else
    {
        return State::Uninitialized();
    }
}

gtsam::ExpressionFactorGraph FactorManager::getGraph()
{
    return graph_;
}

bool FactorManager::isInitialized()
{
    return initialized_;
}

double FactorManager::getInitialHeading() const
{
    return initial_orientation_.yaw();
}   
