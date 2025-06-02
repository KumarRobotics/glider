/*
* Jason Hughes
* May 2025
*
* Struct to keep track of the robots state
*/

#include "glider/core/state.hpp"
#include "glider/utils/geodetics.hpp"

using namespace glider;

State::State(gtsam::Values& vals, gtsam::Key key, gtsam::Matrix& pose_cov, gtsam::Matrix& velocity_cov)
{
    gtsam::Pose3 pose = vals.at<gtsam::Pose3>(X(key));
    gtsam::Quaternion quat = pose.rotation().toQuaternion();
    gtsam::Point3 vel = vals.at<gtsam::Point3>(V(key));
    gtsam::imuBias::ConstantBias bias = vals.at<gtsam::imuBias::ConstantBias>(B(key));

    this->key_index = key;

    this->position = Eigen::Vector3d(pose.translation().x(), pose.translation().y(), pose.translation().z());
    this->orientation = Eigen::Vector4d(quat.w(), quat.x(), quat.y(), quat.z());
    this->velocity = Eigen::Vector3d(vel.x(), vel.y(), vel.z());
    this->accelerometer_bias = bias.accelerometer();
    this->gyroscope_bias = bias.gyroscope();

    this->pose_covariance = pose_cov;
    this->velocity_covariance = velocity_cov;
    this->position_covariance = pose_cov.block<3,3>(0,0);

    this->altitude = pose.translation().z();
    this->heading = pose.rotation().yaw();
}

State State::Zero()
{
    State state;
    state.key_index = 0;

    state.position = Eigen::Vector3d::Zero();
    state.orientation = Eigen::Vector4d(1.0, 0.0, 0.0, 0.0);
    state.velocity = Eigen::Vector3d::Zero(3);

    state.accelerometer_bias = Eigen::Vector3d::Zero();
    state.gyroscope_bias = Eigen::Vector3d::Zero();

    state.pose_covariance = Eigen::MatrixXd::Zero(6,6);
    state.position_covariance = Eigen::MatrixXd::Zero(3,3);
    state.velocity_covariance = Eigen::MatrixXd::Zero(3,3);

    state.altitude = 0.0;
    state.heading = 0.0;

    state.setMovingStatus(false);
    state.setInitializedStatus(false);

    state.setLatitude(0.0);
    state.setLongitude(0.0);

    return state;
}

double State::getLatitude(const char* zone)
{
    double temp;
    geodetics::UTMtoLL(this->position(1), this->position(0), zone, this->latitude, temp);

    return latitude;
}

double State::getLongitude(const char* zone)
{
    double temp;
    geodetics::UTMtoLL(this->position(1), this->position(0), zone, temp, this->longitude);

    return latitude;
}

std::pair<double, double> State::getLatLon(const char* zone)
{
    geodetics::UTMtoLL(this->position(1), this->position(0), zone, this->latitude, this->longitude);
    return std::make_pair(latitude, longitude);
}

bool State::isMoving() const
{
    return is_moving;
}

bool State::isInitialized() const
{
    return is_initialized;
}

void State::setLatitude(const double lat)
{
    this->latitude = lat;
}

void State::setLongitude(const double lon)
{
    this->longitude = lon;
}

void State::setMovingStatus(const bool status)
{
    this->is_moving = status;
}

void State::setInitializedStatus(const bool status)
{
     this->is_initialized = status;
}

double State::getHeadingDegrees() const
{
    double heading_deg = (heading * 180.0) / M_PI;
    if (heading_deg < 0.0) 
    {
        heading_deg += 360.0;
    }
    else if (heading_deg > 360.0)
    {
        heading_deg -= 360.0;
    }

    return heading_deg;
}
