#include "glider/gekf/global_ekf.hpp"

#include <cmath>

using namespace Glider;

namespace
{

constexpr double kSemiMajor = 6378137.0;
constexpr double kFlattening = 1.0 / 298.257223563;
constexpr double kEccSq = kFlattening * (2.0 - kFlattening);
constexpr double kDegToRad = M_PI / 180.0;
constexpr double kRadToDeg = 180.0 / M_PI;

double wrapPi(double angle)
{
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}

double yawFromQuaternion(const Eigen::Quaterniond& q)
{
    return std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                      1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}

}  // namespace

GlobalEKF::GlobalEKF(const GlobalEKFConfig& config)
: config_(config),
  gate_(config.gps_gate)
{
}

void GlobalEKF::reset()
{
    gate_.reset();

    position_.setZero();
    position_variance_.setZero();
    heading_ = 0.0;
    heading_variance_ = 0.0;
    last_heading_time_ = 0;
    position_initialized_ = false;
    heading_initialized_ = false;
}

void GlobalEKF::updateFixStatus(int64_t timestamp, int8_t status)
{
    gate_.update(timestamp, status);
}

void GlobalEKF::checkGpsTimeout(int64_t now)
{
    gate_.checkTimeout(now);
}

void GlobalEKF::updatePosition(const Eigen::Vector3d& lla, double horizontal_sigma, double vertical_sigma)
{
    position_(0) = lla(0) * kDegToRad;
    position_(1) = lla(1) * kDegToRad;
    position_(2) = lla(2);

    const double s = std::sin(position_(0));
    const double meridian = kSemiMajor * (1.0 - kEccSq) / std::pow(1.0 - kEccSq * s * s, 1.5);
    const double transverse = kSemiMajor / std::sqrt(1.0 - kEccSq * s * s);
    const double c = std::max(std::cos(position_(0)), 1.0e-6);

    const double lat_sigma = horizontal_sigma / (meridian + position_(2));
    const double lon_sigma = horizontal_sigma / ((transverse + position_(2)) * c);

    position_variance_(0) = lat_sigma * lat_sigma;
    position_variance_(1) = lon_sigma * lon_sigma;
    position_variance_(2) = vertical_sigma * vertical_sigma;

    position_initialized_ = true;
}

void GlobalEKF::initHeading(int64_t timestamp, double z, double r)
{
    heading_ = z;
    heading_variance_ = r;
    last_heading_time_ = timestamp;
    heading_initialized_ = true;
}

void GlobalEKF::propagate(int64_t timestamp)
{
    double dt = static_cast<double>(timestamp - last_heading_time_) * config_.seconds_per_tick;
    last_heading_time_ = timestamp;

    if (dt <= 0.0) {
        return;
    }
    if (dt > config_.max_dt) {
        dt = config_.max_dt;
    }

    heading_variance_ += config_.sigma_heading * config_.sigma_heading * dt;
}

bool GlobalEKF::updateHeading(double z, double r)
{
    const double y = wrapPi(z - heading_);
    const double s = heading_variance_ + r;

    if (s <= 0.0 || (y * y / s) > config_.nis_threshold) {
        return false;
    }

    const double k = heading_variance_ / s;

    heading_ = wrapPi(heading_ + k * y);
    heading_variance_ = (1.0 - k) * (1.0 - k) * heading_variance_ + k * k * r;
    return true;
}

bool GlobalEKF::updateCompass(int64_t timestamp, double heading)
{
    const double z = wrapPi(heading + config_.mag_declination);
    const double r = config_.sigma_compass * config_.sigma_compass;

    if (!heading_initialized_) {
        initHeading(timestamp, z, r);
        return true;
    }

    propagate(timestamp);
    return updateHeading(z, r);
}

bool GlobalEKF::updateGpsHeading(int64_t timestamp, const Eigen::Quaterniond& orientation)
{
    if (!gate_.trusted()) {
        return false;
    }

    const double z = wrapPi(-yawFromQuaternion(orientation) + config_.gps_heading_offset);
    const double r = config_.sigma_gps_heading * config_.sigma_gps_heading;

    if (!heading_initialized_) {
        initHeading(timestamp, z, r);
        return true;
    }

    propagate(timestamp);
    return updateHeading(z, r);
}

Eigen::Vector3d GlobalEKF::position() const
{
    return Eigen::Vector3d(position_(0) * kRadToDeg, position_(1) * kRadToDeg, position_(2));
}

Eigen::Matrix3d GlobalEKF::positionCovariance() const
{
    return Eigen::Matrix3d(position_variance_.asDiagonal());
}

template <GlobalFrame F>
GlobalState GlobalEKF::state() const
{
    GlobalState state;
    state.latitude = position_(0) * kRadToDeg;
    state.longitude = position_(1) * kRadToDeg;
    state.altitude = position_(2);

    if constexpr (F == GlobalFrame::NED) {
        state.heading = heading_;
    } else {
        state.heading = wrapPi(M_PI_2 - heading_);
    }
    state.heading_deg = kRadToDeg * state.heading;

    return state;
}

template GlobalState GlobalEKF::state<GlobalFrame::ENU>() const;
template GlobalState GlobalEKF::state<GlobalFrame::NED>() const;
