#include "glider/gekf/global_ekf.hpp"

#include <cmath>
#include <utility>

//namespace Glider
//{
using namespace Glider;

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

GlobalEKF::GlobalEKF(const GlobalEKFConfig& config) : config_(config), gate_(config.gps_gate)
{
    reset();
}

void GlobalEKF::reset()
{
    gate_.reset();

    x_.setZero();
    P_.setZero();
    P_(3, 3) = config_.init_heading_sigma * config_.init_heading_sigma;
    P_(4, 4) = config_.init_gyro_bias_sigma * config_.init_gyro_bias_sigma;
    P_(5, 5) = config_.init_mag_bias_sigma * config_.init_mag_bias_sigma;

    last_predict_time_ = 0;
    has_predict_time_ = false;
    heading_trusted_ = false;
    position_initialized_ = false;
    heading_initialized_ = false;
}

// radians of latitude per meter north
double GlobalEKF::latScale() const
{
    const double s = std::sin(x_(0));
    const double meridian = kSemiMajor * (1.0 - kEccSq) / std::pow(1.0 - kEccSq * s * s, 1.5);
    return 1.0 / (meridian + x_(2));
}

// radians of longitude per meter east
double GlobalEKF::lonScale() const
{
    const double s = std::sin(x_(0));
    const double transverse = kSemiMajor / std::sqrt(1.0 - kEccSq * s * s);
    const double c = std::max(std::cos(x_(0)), 1.0e-6);
    return 1.0 / ((transverse + x_(2)) * c);
}

void GlobalEKF::predict(int64_t timestamp, const Eigen::Vector3d& gyro)
{
    if (!has_predict_time_) {
        last_predict_time_ = timestamp;
        has_predict_time_ = true;
        return;
    }

    const double dt = static_cast<double>(timestamp - last_predict_time_) * config_.seconds_per_tick;
    last_predict_time_ = timestamp;

    if (dt <= 0.0 || dt > config_.max_dt) {
        return;
    }

    x_(3) = wrapPi(x_(3) + (gyro.z() - x_(4)) * dt);

    Matrix6d F = Matrix6d::Identity();
    F(3, 4) = -dt;

    const double sg = config_.sigma_gyro;
    const double sb = config_.sigma_gyro_bias;

    Matrix6d Q = Matrix6d::Zero();
    Q(3, 3) = sg * sg * dt + sb * sb * dt * dt * dt / 3.0;
    Q(3, 4) = -sb * sb * dt * dt / 2.0;
    Q(4, 3) = Q(3, 4);
    Q(4, 4) = sb * sb * dt;

    // The mag bias is only separable while the GPS heading is available.
    if (heading_trusted_) {
        Q(5, 5) = config_.sigma_mag_bias * config_.sigma_mag_bias * dt;
    }

    P_ = F * P_ * F.transpose() + Q;
    P_ = 0.5 * (P_ + P_.transpose()).eval();
}

void GlobalEKF::updateFixStatus(int64_t timestamp, int8_t status)
{
    gate_.update(timestamp, status);
    setHeadingTrusted(gate_.trusted());
}

void GlobalEKF::checkGpsTimeout(int64_t now)
{
    gate_.checkTimeout(now);
    setHeadingTrusted(gate_.trusted());
}

void GlobalEKF::updatePosition(const Eigen::Vector3d& lla, double horizontal_sigma, double vertical_sigma)
{
    x_(0) = lla(0) * kDegToRad;
    x_(1) = lla(1) * kDegToRad;
    x_(2) = lla(2);

    const double lat_sigma = horizontal_sigma * latScale();
    const double lon_sigma = horizontal_sigma * lonScale();
    P_.block<3, 3>(0, 0) = Eigen::Vector3d(lat_sigma * lat_sigma,
                                           lon_sigma * lon_sigma,
                                           vertical_sigma * vertical_sigma).asDiagonal();

    position_initialized_ = true;
}


bool GlobalEKF::updateGpsHeading(const Eigen::Quaterniond& orientation, double sigma)
{
    if (!gate_.trusted()) {
        return false;
    }

    const double z = wrapPi(yawFromQuaternion(orientation) + config_.gps_heading_offset);

    if (!heading_initialized_) {
        x_(3) = z;
        P_(3, 3) = sigma * sigma;
        heading_initialized_ = true;
        return true;
    }

    Eigen::Matrix<double, 1, 6> H = Eigen::Matrix<double, 1, 6>::Zero();
    H(0, 3) = 1.0;

    const double y = wrapPi(z - x_(3));
    return updateScalar(H, y, sigma * sigma);
}

bool GlobalEKF::updateCompass(double heading)
{
    const double z = wrapPi(heading + config_.mag_declination);

    if (!heading_initialized_) {
        x_(3) = z;
        x_(5) = 0.0;
        P_(3, 3) = config_.sigma_compass * config_.sigma_compass;
        heading_initialized_ = true;
        return true;
    }

    Eigen::Matrix<double, 1, 6> H = Eigen::Matrix<double, 1, 6>::Zero();
    H(0, 3) = 1.0;
    H(0, 5) = 1.0;

    const double y = wrapPi(z - (x_(3) + x_(5)));
    return updateScalar(H, y, config_.sigma_compass * config_.sigma_compass);
}

bool GlobalEKF::updateScalar(const Eigen::Matrix<double, 1, 6>& H, double innovation, double r)
{
    const double s = (H * P_ * H.transpose())(0, 0) + r;
    if (s <= 0.0) {
        return false;
    }

    if (innovation * innovation / s > config_.nis_threshold) {
        return false;
    }

    const Vector6d K = P_ * H.transpose() / s;

    x_ += K * innovation;
    x_(3) = wrapPi(x_(3));
    x_(5) = wrapPi(x_(5));

    const Matrix6d IKH = Matrix6d::Identity() - K * H;
    P_ = IKH * P_ * IKH.transpose() + r * K * K.transpose();
    P_ = 0.5 * (P_ + P_.transpose()).eval();
    return true;
}

void GlobalEKF::setHeadingTrusted(bool trusted)
{
    if (trusted && !heading_trusted_) {
        P_(3, 3) = std::max(P_(3, 3), config_.retrust_heading_sigma * config_.retrust_heading_sigma);
        P_(5, 5) = std::max(P_(5, 5), config_.retrust_mag_bias_sigma * config_.retrust_mag_bias_sigma);
    }
    heading_trusted_ = trusted;
}

Eigen::Vector3d GlobalEKF::position() const
{
    return Eigen::Vector3d(x_(0) * kRadToDeg, x_(1) * kRadToDeg, x_(2));
}

template <GlobalFrame F>
GlobalState GlobalEKF::state()
{
    GlobalState state;
    state.latitude = x_(0);
    state.longitude = x_(1);
    state.altitude = x_(2);
    if constexpr (F == GlobalFrame::NED) {
        state.heading = x_(3);
        state.heading_deg = kRadToDeg * x_(3);
    } else {
        state.heading = wrapPi(M_PI_2 - x_(3));
        state.heading_deg = kRadToDeg * state.heading;
    }

    return state;
}

template GlobalState GlobalEKF::state<GlobalFrame::ENU>();
template GlobalState GlobalEKF::state<GlobalFrame::NED>();

//}  // namespace Glider
