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

const Eigen::Matrix3d NED2ENU = (Eigen::Matrix3d() << 
    0.0, 1.0, 0.0,
    1.0, 0.0, 0.0,
    0.0, 0.0, -1.0
).finished();

double nanosecInt2Float(int64_t timestamp)
{
    return timestamp * 1e-9;
}

FactorManager::FactorManager(const std::map<std::string, double>& config)
{    
    for (const auto& kv : config) 
    {
        this->config[kv.first] = kv.second;
    }

    this->gravity_vec = Eigen::Vector3d(0.0, 0.0, this->config["gravity"]);
    this->bias_estimate_vec = Eigen::MatrixXd::Zero(static_cast<int>(this->config["bias_num_measurements"]), 6);
    this->init_counter = 0;
    this->imu2body = Eigen::Matrix3d::Identity();
    
    this->_initialized = false;
    this->_gps_initialized = false;
    this->_key_index = 0;
    this->_lastOptimizeTime = 0.0;
    this->_lastImuTime = 0.0;
    this->params = this->defaultParams(this->config["gravity"]);

    this->_prior_noise = gtsam::noiseModel::Isotropic::Sigma(6, this->config["gps_noise"]);
    this->_odom_noise = gtsam::noiseModel::Isotropic::Sigma(6, this->config["odom_noise"]);
    this->_gps_noise = gtsam::noiseModel::Isotropic::Sigma(3, this->config["gps_noise"]);
    this->_heading_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(M_PI/2, M_PI/2, this->config["heading_noise"]));

    this->_graph = gtsam::ExpressionFactorGraph();
    this->_params = gtsam::GaussNewtonParams();
    this->_initials = gtsam::Values();
    this->_parameters = gtsam::ISAM2Params();
    this->_parameters.setRelinearizeThreshold(0.1);
    this->_parameters.relinearizeSkip = 1;
    this->_isam = gtsam::ISAM2(this->_parameters);
    this->_orientation = Eigen::Vector4d::Zero();
    this->_smoother = gtsam::IncrementalFixedLagSmoother(this->config["lag_time"], _parameters);

    this->optimized_pose = gtsam::Pose3();
    this->last_velocity = gtsam::Point3(0, 0, 0);

    _imu_buffer = ImuBuffer(1000);

    std::cout << "[GLIDER] Factor Manager Initialized" << std::endl;
}

boost::shared_ptr<gtsam::PreintegrationCombinedParams> FactorManager::defaultParams(double g)
{
    auto params = gtsam::PreintegrationCombinedParams::MakeSharedD(g);
    double kGyroSigma = (0.5 * M_PI / 180.0) / 60.0;  // 0.5 degree ARW
    double kAccelSigma = 0.001; // / 60.0;  // 10 cm VRW
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    params->setGyroscopeCovariance(std::pow(kGyroSigma, 2) * I);
    params->setAccelerometerCovariance(std::pow(kAccelSigma, 2) * I);
    params->setIntegrationCovariance(std::pow(0.0000001, 2) * I);

    return params;
}

void FactorManager::imuInitialize(const Eigen::Vector3d& accel_meas, const Eigen::Vector3d& gyro_meas, const Eigen::Vector4d& orient) 
{
    if (init_counter < static_cast<int>(config["bias_num_measurements"])) {
        bias_estimate_vec.row(init_counter).head(3) = accel_meas + gravity_vec;
        bias_estimate_vec.row(init_counter).tail(3) = gyro_meas;
        init_counter++;
    }

    if (init_counter == static_cast<int>(config["bias_num_measurements"])) {
        Eigen::VectorXd bias_mean = bias_estimate_vec.colwise().mean();
        bias = gtsam::imuBias::ConstantBias(
            Eigen::Vector3d(bias_mean.head(3)),
            Eigen::Vector3d(bias_mean.tail(3))
        );
        
        pim = boost::make_shared<gtsam::PreintegratedCombinedMeasurements>(params, bias);
        pim_copy = boost::make_shared<gtsam::PreintegratedCombinedMeasurements>(params, bias);
        
        // GTSAM expects quaternion as w,x,y,z
        gtsam::Rot3 rot = gtsam::Rot3::Quaternion(orient(0), orient(1), orient(2), orient(3));
        Eigen::Matrix3d x = rot.matrix();
        
        Eigen::Matrix3d init_orient_matrix = imu2body * rot.matrix();
        initial_orientation = gtsam::Rot3(init_orient_matrix);
        //std::cout << initial_orientation << std::endl;
        _initialized = true;
    }
}

bool FactorManager::addGpsFactor(int64_t timestamp, const Eigen::Vector3d& gps) 
{
    if (!_initialized) 
    {
        prev_gps_ = gps;
        return 0;
    }
    
    Eigen::Vector3d meas = Eigen::Vector3d::Zero();
    
    double easting, northing;
    char zone[4];
    geodetics::LLtoUTM(gps(0), gps(1), northing, easting, zone);
    meas.head(2) << easting, northing;
    meas(2) = gps(2);

    double timestamp_f = nanosecInt2Float(timestamp);

    std::cout << "adding keys in gps" << std::endl; 
    if (_key_index == 0) 
    {
        navstate_pose = gtsam::Pose3(initial_orientation, gtsam::Point3(meas(0), meas(1), meas(2)));
        
        init_navstate = gtsam::NavState(navstate_pose, gtsam::Point3(0, 0, 0));
        lastNavState = init_navstate;
        
        _initials.insert(X(_key_index), navstate_pose);
        _initials.insert(V(_key_index), gtsam::Point3(0, 0, 0));
        _initials.insert(R(_key_index), initial_orientation);
        _initials.insert(B(_key_index), bias);

        _smoother_timestamps[X(_key_index)] = timestamp_f;
        _smoother_timestamps[V(_key_index)] = timestamp_f;
        _smoother_timestamps[R(_key_index)] = timestamp_f;
        _smoother_timestamps[B(_key_index)] = timestamp_f;

        _graph.add(gtsam::PriorFactor<gtsam::Pose3>(X(_key_index), navstate_pose, gtsam::noiseModel::Isotropic::Sigma(6, 0.001)));
        _graph.add(gtsam::PriorFactor<gtsam::Point3>(V(_key_index), gtsam::Point3(0, 0, 0), gtsam::noiseModel::Isotropic::Sigma(3, 0.001)));
        _graph.add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(B(_key_index), bias, gtsam::noiseModel::Isotropic::Sigma(6, 0.001)));
    }
                                  
    if (_key_index > 0) 
    {
	    //auto prediction = this->predict(timestamp);
        //auto diff = (std::get<0>(prediction) - meas).norm();
        //auto maxEllipseVal = last_marginal_covariance.diagonal().maxCoeff();
        // Ensure that if GPS is greater than 5m away from prediction we discard it.
        // If the covariance is very large, we will always include it
        //if (diff > std::max(2.0 * maxEllipseVal, this->config["_gps_noise"])) 
        //{
        //    std::cout << "[GLIDER] Rejecting GPS within " << maxEllipseVal << " " << diff << std::endl;
        //    _last_gps_time = nanosecInt2Float(timestamp);
        //    pim->resetIntegration();
        //    //_key_index++;
        //    return 0;
	    //}
        
        std::lock_guard<std::mutex> lock(std::mutex);

        std::vector<std::pair<double, ImuData>> measurements = _imu_buffer.get(_last_gps_time, nanosecInt2Float(timestamp)); 
        size_t num_measurements = measurements.size();
        if (num_measurements < 5)
        {
            std::cout << "error" << std::endl; 
            return false;
        }

        for (size_t i = 0; i < measurements.size(); ++i)
        {
            double dt;
            if (i == 0) 
            {
                dt = measurements[i].first - _last_gps_time;
            }
            else
            {
                dt = measurements[i].first - measurements[i-1].first;
            }
            ImuData data = measurements[i].second;
	        if (dt <= 0.) continue;

            pim->integrateMeasurement(data.accel, data.gyro, dt);
        }

        _graph.add(gtsam::CombinedImuFactor(X(_key_index),
                                              V(_key_index),
                                              X(_key_index-1),
                                              V(_key_index-1),
                                              B(_key_index),
                                              B(_key_index-1),
                                              *pim));
                
        _initials.insert(X(_key_index), optimized_pose);
        _initials.insert(V(_key_index), last_velocity);
        _initials.insert(B(_key_index), bias);
        _initials.insert(R(_key_index), optimized_pose.rotation());

        _smoother_timestamps[X(_key_index)] = timestamp_f;
        _smoother_timestamps[V(_key_index)] = timestamp_f;
        _smoother_timestamps[R(_key_index)] = timestamp_f;
        _smoother_timestamps[B(_key_index)] = timestamp_f;


    }

    double heading = geodetics::gpsHeading(gps(0), gps(1), prev_gps_(0), prev_gps_(1));
    gtsam::Rot3 heading_rot = gtsam::Rot3::Yaw(heading);
    
    _graph.add(gtsam::GPSFactor(X(_key_index), gtsam::Point3(meas(0), meas(1), meas(2)), _gps_noise));
    _graph.add(gtsam::PriorFactor<gtsam::Rot3>(R(_key_index), heading_rot, _heading_noise));
    _last_gps_time = timestamp_f;
    
    _key_index++;

    return true;
}

void FactorManager::addOdometryFactor(int64_t timestamp, const Eigen::Vector3d& pose, const Eigen::Vector4d& quat) 
{
    if (!_initialized || _key_index == 0) 
    {
        return;
    }
    std::cout << "adding keys in odom" << std::endl; 
    double timestamp_f = nanosecInt2Float(timestamp);

    gtsam::Rot3 rotation = gtsam::Rot3::Quaternion(quat(0), quat(1), quat(2), quat(3));
    gtsam::Pose3 meas(rotation, gtsam::Point3(pose(0), pose(1), pose(2)));

    if (!_initials.exists(X(_key_index)))
    {
        gtsam::Pose3 prev_pose;
        if (optimized_pose.translation().norm() > 0)
        {
            prev_pose = optimized_pose;
        }
        else
        {
            prev_pose = gtsam::Pose3();
        }

        gtsam::Pose3 predicted_pose = prev_pose.compose(meas);
        _initials.insert(X(_key_index), predicted_pose);
        _initials.insert(V(_key_index), last_velocity);
        _initials.insert(B(_key_index), bias);
        
        _smoother_timestamps[X(_key_index)] = timestamp_f;
        _smoother_timestamps[V(_key_index)] = timestamp_f;
        _smoother_timestamps[B(_key_index)] = timestamp_f;
    }

    _graph.add(gtsam::BetweenFactor<gtsam::Pose3>(X(_key_index-1), X(_key_index), meas, _odom_noise));
    _key_index++;
}

void FactorManager::addHeadingFactor(int64_t timestamp, const double& delta_heading)
{
    // pass
}

void FactorManager::addImuFactor(int64_t timestamp, const Eigen::Vector3d& accel, const Eigen::Vector3d& gyro, const Eigen::Vector4d& orient) 
{
    if (!_initialized) 
    {
        _lastOptimizeTime = nanosecInt2Float(timestamp);
        _lastImuTime = nanosecInt2Float(timestamp);
        imuInitialize(accel, gyro, orient);
        return;
    }
   
    double timestamp_f = nanosecInt2Float(timestamp);

    std::lock_guard<std::mutex> lock(std::mutex);
    _imu_buffer.add(timestamp_f, accel, gyro, orient);

    Eigen::Vector3d accel_meas = accel;
    Eigen::Vector3d gyro_meas = gyro;
    double dT = nanosecInt2Float(timestamp) - _lastImuTime;
    if (dT <=0) return;
    
    pim_copy->integrateMeasurement(accel_meas, gyro_meas, dT);
    _lastImuTime = nanosecInt2Float(timestamp);
    _last_accel_meas = accel;
    _last_gyro_meas = gyro;
    _orient = orient;

    return;
}

std::tuple<Eigen::Vector3d, Eigen::Vector4d> FactorManager::predict(int64_t timestamp)
{
    if (!pim)
    {
        return std::make_tuple(Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector4d(0.0, 0.0, 0.0, 0.0));
    }
    double dt = nanosecInt2Float(timestamp) - _lastImuTime; 
    gtsam::NavState navstate(optimized_pose, last_velocity);
    gtsam::NavState result = pim_copy->predict(navstate, bias);

    // reset the graph
    _initials.clear();
    _smoother_timestamps.clear();
    _graph.resize(0);
    
    gtsam::Rot3 rotation = result.attitude();
    gtsam::Point3 translation = result.position();
    gtsam::Quaternion quat = rotation.toQuaternion();
    Eigen::Vector4d quaternion;
    quaternion << quat.w(), quat.x(), quat.y(), quat.z();

    return std::make_tuple(Eigen::Vector3d(translation.x(), translation.y(), translation.z()), quaternion);
}

void FactorManager::initializeGraph() 
{
    if (!_initialized) 
    {
        return;
    }
    _initials = gtsam::InitializePose3::initialize(_graph);
}

gtsam::Values FactorManager::optimize() 
{
    if (!_initialized) 
    {
        return gtsam::Values();
    }
     
    //_isam.update(_graph, _initials);
    //gtsam::Values result = _isam.calculateEstimate();
    //last_marginal_covariance = _isam.marginalCovariance(X(_key_index-1));
    
    std::cout << "=== DEBUGGING SMOOTHER KEYS ===" << std::endl;
    
    // Debug initial values
    std::cout << "Keys in _initials: ";
    for (const auto& key_value : _initials) {
        std::cout << gtsam::DefaultKeyFormatter(key_value.key) << " ";
    }
    std::cout << std::endl;
    
    // Debug timestamps
    std::cout << "Keys with timestamps: ";
    for (const auto& kv : _smoother_timestamps) {
        std::cout << gtsam::DefaultKeyFormatter(kv.first) << " ";
    }
    std::cout << std::endl;
    
    // Debug factor keys
    std::cout << "Keys referenced in factors: ";
    std::set<gtsam::Key> factor_keys;
    for (const auto& factor : _graph) {
        for (gtsam::Key key : factor->keys()) {
            factor_keys.insert(key);
            std::cout << gtsam::DefaultKeyFormatter(key) << " ";
        }
    }
    std::cout << std::endl;
    
    // Check for missing keys
    std::cout << "Missing keys (in factors but not in initials): ";
    for (gtsam::Key key : factor_keys) {
        if (!_initials.exists(key)) {
            std::cout << gtsam::DefaultKeyFormatter(key) << " ";
        }
    }
    std::cout << std::endl;
    
    // Check for missing timestamps
    std::cout << "Missing timestamps (in factors but no timestamp): ";
    for (gtsam::Key key : factor_keys) {
        if (_smoother_timestamps.find(key) == _smoother_timestamps.end()) {
            std::cout << gtsam::DefaultKeyFormatter(key) << " ";
        }
    }
    std::cout << std::endl;
    // end debug output
    std::cout << "graph size: " << _graph.size() << " initials size: " << _initials.size() << std::endl;
    bool graph_is_zero = _graph.size() == 0;
    std::cout << "graph is zero: " << graph_is_zero << std::endl;
    if (_graph.size() == 0 || _initials.size() == 0)
    {
        std::cout << "[GLIDER] No factors or initials, skipping optimization" << std::endl;

        _initials.clear();
        _smoother_timestamps.clear();
        _graph.resize(0);
        
        try
        {
            return _smoother.calculateEstimate();
        }
        catch (const std::exception& e)
        {
            std::cout << "No current estimate" << std::endl;
            return gtsam::Values();
        }
    }
    std::cout << "optimizing" << std::endl;
    _smoother.update(_graph, _initials, _smoother_timestamps);
    gtsam::Values result = _smoother.calculateEstimate();
    last_marginal_covariance = _smoother.marginalCovariance(X(_key_index-1));

    _initials.clear();
    _smoother_timestamps.clear();
    _graph.resize(0);

    return result;
}

std::tuple<Eigen::Vector3d, Eigen::Vector4d, Eigen::Matrix3d> FactorManager::runner() 
{
    if (!_initialized) 
    {
        return std::make_tuple(Eigen::Vector3d::Zero(), Eigen::Vector4d::Zero(), Eigen::Matrix3d::Zero());
    }
    
    gtsam::Values result = optimize();
    pim->resetIntegration();
    pim_copy->resetIntegration();

    gtsam::Pose3 optimized_pose = result.at<gtsam::Pose3>(X(_key_index-1));
    this->optimized_pose = optimized_pose;
    
    gtsam::Rot3 rotation = optimized_pose.rotation();
    gtsam::Point3 translation = optimized_pose.translation();
    
    this->last_velocity = result.at<gtsam::Point3>(V(_key_index-1));
    
    gtsam::Quaternion quat = rotation.toQuaternion();
    Eigen::Vector4d quaternion;
    quaternion << quat.w(), quat.x(), quat.y(), quat.z();
    
    _orientation = quaternion;
    _translation = translation;

    return std::make_tuple(Eigen::Vector3d(translation.x(), translation.y(), translation.z()), quaternion, rotation.matrix());
}

gtsam::ExpressionFactorGraph FactorManager::getGraph()
{
    return _graph;
}

template <typename T>
T FactorManager::getKeyIndex()
{
    if (typeid(T) == typeid(int))
    {
        int index_as_int = static_cast<int>(gtsam::symbolIndex(_key_index));
        return index_as_int;
    }
    else
    {
        return _key_index;
    }
}

bool FactorManager::isInitialized()
{
    return _initialized;
}

template int FactorManager::getKeyIndex<int>();
template gtsam::Key FactorManager::getKeyIndex<gtsam::Key>();
