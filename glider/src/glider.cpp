/*
* Jason Hughes
* July 2025
*
* Take care of all necessary transforms.
*/

#include "glider/core/factor_manager.hpp"
#include "glider/core/glider.hpp"

namespace Glider
{

Glider::Glider(const std::string& path) 
{
    Parameters params = Parameters::Load(path);
    factor_manager_ = FactorManager(params);

    origin_x_ = params.origin_x;
    origin_y_ = params.origin_y;
    frame_ = params.frame;

    // IMU transformations
    ned_to_enu_rot_ << 0.0, 1.0, 0.0, 
                       1.0, 0.0, 0.0, 
                       0.0, 0.0, -1.0;
    ned_to_enu_quat_ = Eigen::Quaterniond(ned_to_enu_rot_);

    prev_pose_ = Eigen::Isometry3d::Identity();

    current_heading_ = 0.0;
    initial_heading_ = 0.0;
    set_initial_heading_ = true;

    std::cout << "[GLIDER] Using IMU frame: " << frame_ << std::endl;
    std::cout << "[GLIDER] Glider initialized" << std::endl;
}

double Glider::northEastToEastNorth(double heading_ne)
{
    // convert heading from NE to EN
    return std::fmod(M_PI/2 - heading_ne, 2*M_PI);
}

Eigen::Vector4d Glider::correctImuOrientation(const Eigen::Vector4d orient)
{
    Eigen::Quaterniond quat_enu(orient(0), orient(1), orient(2), orient(3));
    Eigen::Vector3d euler = quat_enu.toRotationMatrix().eulerAngles(2, 1, 0);
    double yaw = euler[0];
    if (std::abs(yaw - current_heading_) > 0.18) 
    {
        // IMU orientation yaw is off by more than 10 degrees
        // therefore we need to rotate it
        double yaw_error = current_heading_ - yaw;
        while (yaw_error > M_PI) yaw_error -= 2.0 * M_PI;
        while (yaw_error < -M_PI) yaw_error += 2.0 * M_PI;

        Eigen::Quaterniond q_correction(Eigen::AngleAxisd(yaw_error, Eigen::Vector3d::UnitZ()));
        Eigen::Quaterniond q_corrected = q_correction * quat_enu;
        Eigen::Vector4d q_corr_enu(q_corrected.w(), q_corrected.x(), q_corrected.y(), q_corrected.z());
        return q_corr_enu;
    }
    else return orient;
}

void Glider::addGPS(int64_t timestamp, Eigen::Vector3d& gps)
{
    // transform from GPS To UTM
    Eigen::Vector3d meas = Eigen::Vector3d::Zero();
    
    double easting, northing;
    char zone[4];
    geodetics::LLtoUTM(gps(0), gps(1), northing, easting, zone);
    
    easting = easting - origin_x_;
    northing = northing - origin_y_;
    
    meas.head(2) << easting, northing;
    meas(2) = gps(2);

    factor_manager_.addGpsFactor(timestamp, meas);
}

void Glider::addIMU(int64_t timestamp, Eigen::Vector3d& accel, Eigen::Vector3d& gyro, Eigen::Vector4d& quat)
{
    if (frame_ == "ned")
    {
        // transfrom to enu
        Eigen::Quaterniond quat_ned(quat(0), quat(1), quat(2), quat(3));
        Eigen::Quaterniond quat_enu = ned_to_enu_quat_ * quat_ned;
        Eigen::Vector4d vec_enu;
        vec_enu << quat_enu.w(), quat_enu.x(), quat_enu.y(), quat_enu.z();

        Eigen::Vector4d quat_corr = correctImuOrientation(vec_enu);

        Eigen::Vector3d accel_enu = ned_to_enu_rot_ * accel;
        Eigen::Vector3d gyro_enu = ned_to_enu_rot_ * gyro;

        factor_manager_.addImuFactor(timestamp, accel_enu, gyro_enu, quat_corr);
    }
    else if (frame_ == "enu")
    {
        Eigen::Vector4d quat_corr = correctImuOrientation(quat);
        factor_manager_.addImuFactor(timestamp, accel, gyro, quat_corr);
    }
    else
    {
        throw std::runtime_error("IMU Frame, not supported use ned or enu");
        return;
    }
}  

void Glider::addMagnetometer(int64_t timestamp, double heading)
{
    // convert from NE to EN
    heading = northEastToEastNorth(heading);

    // normalize between -pi and pi
    while (heading > M_PI) heading -= 2.0 * M_PI;
    while (heading < -M_PI) heading += 2.0 * M_PI;

    // save in the correct spots
    current_heading_ = heading;
    if (set_initial_heading_)
    {
        initial_heading_ = heading;
        set_initial_heading_ = false;
    }
}

void Glider::addOdom(int64_t timestamp, Eigen::Isometry3d& pose)
{
    if (factor_manager_.isInitialized() && !set_initial_heading_)
    {
        // Convert orb pose to ENU
        Eigen::Matrix3d odom_to_enu;
        double h = initial_heading_;
        odom_to_enu << std::cos(h), -std::sin(h), 0.0d, 
                      std::sin(h), std::cos(h), 0.0d, 
                      0.0d, 0.0d, 1.0d;
        Eigen::Isometry3d enu_pose = pose.rotate(odom_to_enu);
        
        // calculate the relative pose in ENU frame
        Eigen::Isometry3d rel_pose = prev_pose_.inverse() * enu_pose;
        
        // convert to gtsam for pose graph
        gtsam::Pose3 gtpose = isometryToPose(rel_pose);
        factor_manager_.addOdometryFactor(timestamp, gtpose);
        prev_pose_ = enu_pose;
    }
    else
    {
        prev_pose_ = pose;
    }
}

Odometry Glider::interpolate(int64_t timestamp)
{
    Odometry odom = factor_manager_.predict(timestamp);
    return odom;
}

State Glider::optimize()
{
    return factor_manager_.runner();
}

gtsam::Pose3 Glider::isometryToPose(const Eigen::Isometry3d& iso)
{
    Eigen::Matrix3d rot = iso.rotation();
    Eigen::Vector3d t = iso.translation();

    gtsam::Rot3 gt_rot(rot);
    gtsam::Point3 gt_t(t);

    return gtsam::Pose3(gt_rot, gt_t);
}
}
