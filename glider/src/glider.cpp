/*
* Jason Hughes
* July 2025
*
* Take care of all necessary transforms.
*/

#include "glider/core/glider.hpp"

namespace Glider
{

Glider::Glider(const std::string& path)
{
    Parameters params = Parameters::Load(path);
    initializeLogging(params);
    factor_manager_.initialize(params);

    frame_ = params.frame;
    t_body_imu_ = params.t_body_imu;
    r_body_imu_ = params.r_body_imu;
    t_body_gps_ = params.t_body_gps;
    gps_heading_offset_ = params.gps_heading_offset;
    r_enu_ned_ = Eigen::Matrix3d::Zero();
    r_enu_ned_ << 0.0, 1.0, 0.0,
                  1.0, 0.0, 0.0,
                  0.0, 0.0, -1.0;

    LOG(INFO) << "[GLIDER] Using IMU frame: " << frame_;
    LOG(INFO) << "[GLIDER] Using Fixed Lag Smoother: " << std::boolalpha << params.smooth;
    LOG(INFO) << "[GLIDER] Logging to: " << params.log_dir;
    use_dgpsfm_ = params.use_dgpsfm;
    dgps_ = Geodetics::DifferentialGpsFromMotion(params.frame, params.dgpsfm_threshold);

    current_odom_ = OdometryWithCovariance::Uninitialized();
    utm_zone_ = "";

    LOG(INFO) << "[GLIDER] Using IMU frame: " << frame_;
    LOG(INFO) << "[GLIDER] Using Fixed Lag Smoother: " << std::boolalpha << params.smooth;
    LOG(INFO) << "[GLIDER] Using DGPS From Motion: " << std::boolalpha << params.use_dgpsfm;
    LOG(INFO) << "[GLIDER] Using DGPS: " << std::boolalpha << params.use_dgps;
    assert(!(params.use_dgpsfm && params.use_dgps) && "Both DGPS and DGPS From Motion are set to true, this is not allowed");
    LOG(INFO) << "[GLIDER] Glider initialized";
}

void Glider::initializeLogging(const Parameters& params) const
{
    // initialize GLog
    google::InitGoogleLogging("Glider");

    FLAGS_log_dir = params.log_dir;
    if (params.log) FLAGS_alsologtostderr = 1;
}


void Glider::addGps(int64_t timestamp, Eigen::Vector3d& gps, const double sigma)
{
    // route the
    if (use_dgpsfm_)
    {
        addGpsWithHeading(timestamp, gps);
        return;
    }

    // transform from lat lon To UTM
    Eigen::Vector3d meas = Eigen::Vector3d::Zero();

    double easting, northing;
    char zone[4];
    geodetics::LLtoUTM(gps(0), gps(1), northing, easting, zone);
    utm_zone_ = std::string(zone);

    // keep everything in the enu frame
    meas.head(2) << easting, northing;
    meas(2) = gps(2);

    // Convert the antenna position to the body origin.
    if (current_odom_.isInitialized())
        meas -= current_odom_.getOrientation<Eigen::Quaterniond>().toRotationMatrix() * t_body_gps_;

    factor_manager_.addGpsFactor(timestamp, meas, sigma);
}

void Glider::addGpsWithHeading(int64_t timestamp, Eigen::Vector3d& gps, Eigen::Vector2d& heading, const double sigma)
{
    // transform from lat lon To UTM
    Eigen::Vector3d meas = Eigen::Vector3d::Zero();

    double easting, northing;
    char zone[4];
    geodetics::LLtoUTM(gps(0), gps(1), northing, easting, zone);
    utm_zone_ = std::string(zone);

    // keep everything in the enu frame
    meas.head(2) << easting, northing;
    meas(2) = gps(2);

    const double body_heading = heading.x() + gps_heading_offset_;
    const Eigen::Matrix3d r_enu_body =
        Eigen::AngleAxisd(body_heading, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    meas -= r_enu_body * t_body_gps_;

    if (factor_manager_.isSystemInitialized())
    {
        factor_manager_.addGpsFactor(timestamp, meas, body_heading, true, sigma, heading.y());
    } else {
        factor_manager_.addGpsFactor(timestamp, meas, 0.0, false, sigma);
    }
}

void Glider::addGpsWithHeading(int64_t timestamp, Eigen::Vector3d& gps, const double sigma)
{
    // transform from lat lon To UTM
    Eigen::Vector3d meas = Eigen::Vector3d::Zero();

    double easting, northing;
    char zone[4];
    geodetics::LLtoUTM(gps(0), gps(1), northing, easting, zone);
    utm_zone_ = std::string(zone);

    // keep everything in the enu frame
    meas.head(2) << easting, northing;
    meas(2) = gps(2);

    if (current_odom_.isInitialized())
        meas -= current_odom_.getOrientation<Eigen::Quaterniond>().toRotationMatrix() * t_body_gps_;

    if(factor_manager_.isSystemInitialized() && current_odom_.isMovingFasterThan(dgps_.getVelocityThreshold()))
    {
        double heading = dgps_.getHeading(gps);
        factor_manager_.addGpsFactor(timestamp, meas, heading, true, sigma);
    }
    else
    {
        dgps_.setLastGps(gps);
        factor_manager_.addGpsFactor(timestamp, meas, 0.0, false, sigma);
    }
}

void Glider::addImu(int64_t timestamp, Eigen::Vector3d& accel, Eigen::Vector3d& gyro, Eigen::Vector4d& quat)
{
    Eigen::Vector3d accel_sensor = accel;
    Eigen::Vector3d gyro_sensor = gyro;
    Eigen::Vector4d quat_enu_sensor = quat;

    if (frame_ == "ned")
    {
        accel_sensor = r_enu_ned_ * accel;
        gyro_sensor = r_enu_ned_ * gyro;
        quat_enu_sensor = rotateQuaternion(r_enu_ned_, quat);
    }
    else if (frame_ != "enu")
    {
        LOG(FATAL) << "[GLIDER] IMU Frame, not supported use ENU or NED";
    }

    // Transform IMU measurements and orientation into the body frame.
    Eigen::Vector3d accel_body = r_body_imu_ * accel_sensor;
    Eigen::Vector3d gyro_body = r_body_imu_ * gyro_sensor;
    const Eigen::Quaterniond q_enu_sensor(quat_enu_sensor(0), quat_enu_sensor(1),
                                         quat_enu_sensor(2), quat_enu_sensor(3));
    const Eigen::Matrix3d r_enu_body = q_enu_sensor.normalized().toRotationMatrix() * r_body_imu_.transpose();
    const Eigen::Quaterniond q_enu_body(r_enu_body);
    Eigen::Vector4d quat_enu_body(q_enu_body.w(), q_enu_body.x(), q_enu_body.y(), q_enu_body.z());
    factor_manager_.addImuFactor(timestamp, accel_body, gyro_body, quat_enu_body);
}

bool Glider::addOdom(int64_t timestamp, const Eigen::Isometry3d& pose)
{
    return factor_manager_.addOdomFactor(timestamp, pose);
}

bool Glider::addOdom(int64_t timestamp, const Eigen::Isometry3d& pose,
                     const Eigen::Vector3d& velocity, double velocity_sigma)
{
    return factor_manager_.addOdomFactor(timestamp, pose, velocity, velocity_sigma);
}

void Glider::addLandmark(int64_t timestamp, size_t lid, const Eigen::Vector3d& utm, const Eigen::Matrix3d& cov)
{
    factor_manager_.addLandmarkFactor(timestamp, lid, utm, cov);
}

PointWithCovariance Glider::getLandmark(size_t lid)
{
    return factor_manager_.getLandmarkPoint(lid);
}

Eigen::Vector3d Glider::getGpsOffset() const
{
    return factor_manager_.getGpsOffset();
}

Odometry Glider::interpolate(int64_t timestamp)
{
    try
    {
        Odometry odom = factor_manager_.predict(timestamp);
        return odom;
    }
    catch (const std::exception& e)
    {
        LOG(ERROR) << "[GLIDER] Interpolation Error: " << e.what();
        return Odometry::Uninitialized();
    }
}

OdometryWithCovariance Glider::optimize(int64_t timestamp)
{
    try
    {
        current_odom_ = factor_manager_.runner(timestamp);
        return current_odom_;
    }
    catch (const std::exception& e)
    {
        LOG(ERROR) << "[GLIDER] Optimization Error: " << e.what();
        return OdometryWithCovariance::Uninitialized();
    }
}

Eigen::Vector4d Glider::rotateQuaternion(const Eigen::Matrix3d& rot, const Eigen::Vector4d& quat) const
{
    Eigen::Quaterniond q_ned(quat(0), quat(1), quat(2), quat(3));
    Eigen::Quaterniond q_ned_enu(rot);

    Eigen::Quaterniond q_enu = q_ned_enu * q_ned;

    return Eigen::Vector4d(q_enu.w(), q_enu.x(), q_enu.y(), q_enu.z());
}
}
