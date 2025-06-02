/*
*
*/
#include <mutex>

#include "glider/core/factor_manager.hpp"
#include "glider/core/imu_buffer.hpp"
#include "glider/utils/geodetics.hpp"
#include "glider/utils/gps_heading.hpp"

#include <gtsam/slam/expressions.h>

using namespace glider;

Eigen::Vector3d vector3(double x, double y, double z)
{
    return Eigen::Vector3d(x, y, z);
}

double nanosecInt2Float(int64_t timestamp)
{
    return timestamp * 1e-9;
}

FactorManager::FactorManager(const std::map<std::string, double>& config, int64_t start_time)
{   
    this->config_ = config;
    for (const auto& kv : config) 
    {
        this->config_[kv.first] = kv.second;
    }

    this->NED2ENU << 0.0, 1.0, 0.0, 
                     1.0, 0.0, 0.0, 
                     0.0, 0.0, -1.0;


    this->gravity_vec_ = Eigen::Vector3d(0.0, 0.0, this->config_["gravity"]);
    this->bias_estimate_vec_ = Eigen::MatrixXd::Zero(static_cast<int>(this->config_["bias_num_measurements"]), 6);
    this->init_counter_ = 0;
    this->imu2body_ = Eigen::Matrix3d::Identity();
   
    this->heading_count_ = 0;
    this->initialized_ = false;
    this->key_index_ = 0;
    this->last_optimize_time_ = 0.0;
    this->last_imu_time_ = 0.0;
    this->params_ = this->defaultParams(this->config_["gravity"]);

    this->prior_noise_ = gtsam::noiseModel::Isotropic::Sigma(6, this->config_["gps_noise"]);
    this->odom_noise_ = gtsam::noiseModel::Isotropic::Sigma(6, this->config_["gps_noise"]);
    this->gps_noise_ = gtsam::noiseModel::Isotropic::Sigma(3, this->config_["gps_noise"]);
    this->heading_noise_ = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(M_PI/2, M_PI/2, this->config_["heading_noise"]));

    this->graph_ = gtsam::ExpressionFactorGraph();
    this->initials_ = gtsam::Values();

    this->parameters_ = gtsam::ISAM2Params();
    this->parameters_.setRelinearizeThreshold(0.1);
    this->parameters_.relinearizeSkip = 1;
   
    this->smoother_ = gtsam::IncrementalFixedLagSmoother(this->config_["lag_time"], parameters_);
    this->isam_ = gtsam::ISAM2(this->parameters_);
    this->current_optimized_pose_ = gtsam::Pose3();
    this->last_velocity_ = gtsam::Point3(0, 0, 0);

    this->imu_buffer_ = ImuBuffer(1000);
    this->compose_odom_ = false;

    std::cout << "[GLIDER] Factor Manager Initialized" << std::endl;
}

boost::shared_ptr<gtsam::PreintegrationCombinedParams> FactorManager::defaultParams(double g)
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
    if (init_counter_ < static_cast<int>(config_["bias_num_measurements"])) {
        bias_estimate_vec_.row(init_counter_).head(3) = accel_meas + gravity_vec_;
        bias_estimate_vec_.row(init_counter_).tail(3) = gyro_meas;
        init_counter_++;
    }

    if (init_counter_ == static_cast<int>(config_["bias_num_measurements"])) {
        Eigen::VectorXd bias_mean = bias_estimate_vec_.colwise().mean();
        bias_ = gtsam::imuBias::ConstantBias(
            Eigen::Vector3d(bias_mean.head(3)),
            Eigen::Vector3d(bias_mean.tail(3))
        );
        
        pim_ = boost::make_shared<gtsam::PreintegratedCombinedMeasurements>(params_, bias_);
        pim_copy_ = boost::make_shared<gtsam::PreintegratedCombinedMeasurements>(params_, bias_);
        
        // GTSAM expects quaternion as w,x,y,z
        gtsam::Rot3 rot = gtsam::Rot3::Quaternion(orient(0), orient(1), orient(2), orient(3));
        //rot = NED2ENU * rot;
        Eigen::Matrix3d init_orient_matrix = imu2body_ * rot.matrix();
        initial_orientation_ = gtsam::Rot3(init_orient_matrix);
        current_heading_ = initial_orientation_.yaw(); 
        double heading_deg = ((current_heading_*180.0) / M_PI) + 360.0;
        std::cout << "[GLIDER] Initialized IMU with heading: " << heading_deg << std::endl;
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
    
    Eigen::Vector3d meas = Eigen::Vector3d::Zero();
    
    double easting, northing;
    char zone[4];
    geodetics::LLtoUTM(gps(0), gps(1), northing, easting, zone);
    meas.head(2) << easting, northing;
    meas(2) = gps(2);

    double timestamp_f = nanosecInt2Float(timestamp);


    if (key_index_ == 0) 
    {
        current_navstate_pose_ = gtsam::Pose3(initial_orientation_, gtsam::Point3(meas(0), meas(1), meas(2)));
        
        initial_navstate_ = gtsam::NavState(current_navstate_pose_, gtsam::Point3(0, 0, 0));
        last_nav_state_ = initial_navstate_;
        
        initials_.insert(X(key_index_), current_navstate_pose_);
        initials_.insert(V(key_index_), gtsam::Point3(0, 0, 0));
        initials_.insert(R(key_index_), initial_orientation_);
        initials_.insert(B(key_index_), bias_);

        smoother_timestamps_[X(key_index_)] = timestamp_f;
        smoother_timestamps_[V(key_index_)] = timestamp_f;
        smoother_timestamps_[R(key_index_)] = timestamp_f;
        smoother_timestamps_[B(key_index_)] = timestamp_f;

        graph_.add(gtsam::PriorFactor<gtsam::Pose3>(X(key_index_), current_navstate_pose_, gtsam::noiseModel::Isotropic::Sigma(6, 0.001)));
        graph_.add(gtsam::PriorFactor<gtsam::Point3>(V(key_index_), gtsam::Point3(0, 0, 0), gtsam::noiseModel::Isotropic::Sigma(3, 0.001)));
        graph_.add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(B(key_index_), bias_, gtsam::noiseModel::Isotropic::Sigma(6, 0.001)));
        key_index_++;

        return;
    }
                                  
    if (key_index_ > 0) 
    { 
        std::lock_guard<std::mutex> lock(std::mutex);
        
        graph_.add(gtsam::CombinedImuFactor(X(key_index_),
                                            V(key_index_),
                                            X(key_index_-1),
                                            V(key_index_-1),
                                            B(key_index_),
                                            B(key_index_-1),
                                            *pim_copy_));
        pim_copy_->resetIntegration();
    
        initials_.insert(X(key_index_), current_optimized_pose_);
        initials_.insert(V(key_index_), current_velocity_);
        initials_.insert(B(key_index_), bias_);

        smoother_timestamps_[X(key_index_)] = timestamp_f;
        smoother_timestamps_[V(key_index_)] = timestamp_f;
        smoother_timestamps_[B(key_index_)] = timestamp_f;
        smoother_timestamps_[R(key_index_)] = timestamp_f;

    }

    if (heading_count_ == 4)
    {    
        // add differential gps heading in ENU frame
        double heading = geodetics::gpsHeading(last_gps_(0), last_gps_(1), gps(0), gps(1));
        double enu_heading = geodetics::geodeticToENU(heading);
        gtsam::Rot3 heading_rot = gtsam::Rot3::Yaw(enu_heading);     
        graph_.addExpressionFactor(gtsam::rotation(X(key_index_)), heading_rot, heading_noise_);
        last_gps_ = gps;
        
        gtsam::Quaternion q_temp(last_imu_orientation_(0), last_imu_orientation_(1), last_imu_orientation_(2), last_imu_orientation_(3));
        gtsam::Rot3 rot(q_temp);
        std::cout << "IMU heading: " << geodetics::headingRadiansToDegrees(rot.yaw()) << std::endl;
        std::cout << "Differential GPS Heading: " << geodetics::headingRadiansToDegrees(heading_rot.yaw()) << std::endl;
        heading_count_ = 0;
    }
    else
    {
        heading_count_++;
    }
    
    // Add gps factor in utm frame
    graph_.add(gtsam::GPSFactor(X(key_index_), gtsam::Point3(meas(0), meas(1), meas(2)), gps_noise_));

    // add odom as between factor
    double bearing = last_odom_.rotation().yaw();
    double distance = last_odom_.translation().norm();
    //graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(X(key_index_-1), X(key_index_), last_odom_, odom_noise_));
    
    last_gps_time_ = timestamp_f;
    compose_odom_ = false;
    
    key_index_++;

    return;
}


void FactorManager::addOdometryFactor(int64_t timestamp, const Eigen::Vector3d& pose, const Eigen::Vector4d& quat) 
{
    if (!initialized_ || key_index_ == 0) 
    {
        return;
    }

    gtsam::Rot3 rotation = gtsam::Rot3::Quaternion(quat(0), quat(1), quat(2), quat(3));
    //gtsam::Pose3 meas(rotation, gtsam::Point3(pose(0), pose(1), pose(2)));
    gtsam::Point3 local_t(pose(0), pose(1), pose(2));
    gtsam::Point3 enu_t = current_optimized_pose_.rotation() * local_t;

    gtsam::Pose3 meas(rotation, enu_t);

    if (compose_odom_)
    {
        last_odom_ = last_odom_.compose(meas);
    }
    else
    {
        last_odom_ = meas;
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

    std::lock_guard<std::mutex> lock(std::mutex);
    imu_buffer_.add(timestamp_f, accel, gyro, orient);

    Eigen::Vector3d accel_meas = accel;
    Eigen::Vector3d gyro_meas = gyro;
    double dt = nanosecInt2Float(timestamp) - last_imu_time_;
    if (dt <=0) return;
    
    pim_copy_->integrateMeasurement(accel_meas, gyro_meas, dt);
    last_imu_time_ = nanosecInt2Float(timestamp);
    last_accel_meas_ = accel;
    last_gyro_meas_ = gyro;
    last_imu_orientation_ = orient;

    return;
}

std::tuple<Eigen::Vector3d, Eigen::Vector4d> FactorManager::predict(int64_t timestamp)
{
    if (!pim_)
    {
        return std::make_tuple(Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector4d(0.0, 0.0, 0.0, 0.0));
    }
    gtsam::NavState navstate(current_optimized_pose_, last_velocity_);
    gtsam::NavState result = pim_copy_->predict(navstate, bias_);

    gtsam::Rot3 rotation = result.attitude();
    gtsam::Point3 translation = result.position();
    gtsam::Quaternion quat = rotation.toQuaternion();
    Eigen::Vector4d quaternion;
    quaternion << quat.w(), quat.x(), quat.y(), quat.z();

    return std::make_tuple(Eigen::Vector3d(translation.x(), translation.y(), translation.z()), quaternion);
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
    
    //last_marginal_covariance = _isam.marginalCovariance(X(_key_index-1));
    
    //std::cout << "=== DEBUGGING SMOOTHER KEYS ===" << std::endl;
    //
    //// Debug initial values
    //std::cout << "Keys in _initials: ";
    //for (const auto& key_value : initials_) {
    //    std::cout << gtsam::DefaultKeyFormatter(key_value.key) << " ";
    //}
    //std::cout << std::endl;
    //
    //// Debug timestamps
    //std::cout << "Keys with timestamps: ";
    //for (const auto& kv : smoother_timestamps_) {
    //    std::cout << gtsam::DefaultKeyFormatter(kv.first) << " ";
    //}
    //std::cout << std::endl;
    //
    //// Debug factor keys
    //std::cout << "Keys referenced in factors: ";
    //std::set<gtsam::Key> factor_keys;
    //for (const auto& factor : graph_) {
    //    for (gtsam::Key key : factor->keys()) {
    //        factor_keys.insert(key);
    //        std::cout << gtsam::DefaultKeyFormatter(key) << " ";
    //    }
    //}
    //std::cout << std::endl;
    //
    //// Check for missing keys
    //std::cout << "Missing keys (in factors but not in initials): ";
    //for (gtsam::Key key : factor_keys) {
    //    if (!initials_.exists(key)) {
    //        std::cout << gtsam::DefaultKeyFormatter(key) << " ";
    //    }
    //}
    //std::cout << std::endl;
    //
    //// Check for missing timestamps
    //std::cout << "Missing timestamps (in factors but no timestamp): ";
    //for (gtsam::Key key : factor_keys) {
    //    if (smoother_timestamps_.find(key) == smoother_timestamps_.end()) {
    //        std::cout << gtsam::DefaultKeyFormatter(key) << " ";
    //    }
    //}
    //std::cout << std::endl;

    //graph_.print();
    //std::cout << "graph size: " << graph_.size() << " initials size: " << initials_.size() << std::endl;
    //// end debug output
    
    if (graph_.size() == 0 || initials_.size() == 0)
    {
        std::cout << "[GLIDER] No factors or initials, skipping optimization" << std::endl;

        initials_.clear();
        smoother_timestamps_.clear();
        graph_.resize(0);
        
        try
        {
            return smoother_.calculateEstimate();
        }
        catch (const std::exception& e)
        {
            std::cout << "[GLIDER] No current estimate" << std::endl;
            return gtsam::Values();
        }
    }
    gtsam::Values result;
    if (key_index_ > 1)
    {
        isam_.update(graph_, initials_);
        result = isam_.calculateEstimate();
        
        //smoother_.update(graph_, initials_, smoother_timestamps_);
        //gtsam::Values result = smoother_.calculateEstimate();
        //last_marginal_covariance_ = smoother_.marginalCovariance(X(key_index_-1));

        initials_.clear();
        smoother_timestamps_.clear();
        graph_.resize(0);
    }
    return result;
}

State FactorManager::runner() 
{
    if (!initialized_) 
    {
        State::Zero();
    }
    if (key_index_ > 1)
    {
        gtsam::Values result = optimize();
        
        last_state_ = current_state_;
        
        gtsam::Matrix pose_cov = isam_.marginalCovariance(X(key_index_-1));
        gtsam::Matrix vel_cov = isam_.marginalCovariance(V(key_index_-1));
        current_state_ = State(result, key_index_-1, pose_cov, vel_cov);
        
        pim_->resetIntegration();
        pim_copy_->resetIntegration();
        gtsam::Pose3 optimized_pose = result.at<gtsam::Pose3>(X(key_index_-1));
        this->current_optimized_pose_ = optimized_pose;
        this->current_velocity_ = result.at<gtsam::Point3>(V(key_index_-1));    
        //std::cout << "here1" << std::endl;
        if (key_index_ > 1)
        {
            //std::cout<<"getting -2 pose" << std::endl;
            this->last_optimized_pose_ = result.at<gtsam::Pose3>(X(key_index_-2));
            this->last_velocity_ = result.at<gtsam::Point3>(V(key_index_-2));
        }
        else
        {
            this->last_optimized_pose_ = gtsam::Pose3(); //optimized_pose;
            this->last_velocity_ = gtsam::Point3(0.0, 0.0, 0.0);//current_velocity_;
        }
        //std::cout<<"here2"<<std::endl;
        gtsam::Rot3 rotation = optimized_pose.rotation();
        gtsam::Point3 translation = optimized_pose.translation();

        gtsam::Quaternion quat = rotation.toQuaternion();
        Eigen::Vector4d quaternion;
        quaternion << quat.w(), quat.x(), quat.y(), quat.z();
        
        orientation_ = quaternion;
        translation_ = translation;
        current_heading_ = rotation.yaw();

        return current_state_;
    }
    else
    {
        return State::Zero();
    }
}

gtsam::ExpressionFactorGraph FactorManager::getGraph()
{
    return graph_;
}

template <typename T>
T FactorManager::getKeyIndex()
{
    if (typeid(T) == typeid(int))
    {
        int index_as_int = static_cast<int>(gtsam::symbolIndex(key_index_));
        return index_as_int;
    }
    else
    {
        return key_index_;
    }
}

bool FactorManager::isInitialized()
{
    return initialized_;
}

template int FactorManager::getKeyIndex<int>();
template gtsam::Key FactorManager::getKeyIndex<gtsam::Key>();
