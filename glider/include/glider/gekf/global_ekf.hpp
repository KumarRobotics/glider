#pragma once

#include <cstdint>

#include <Eigen/Dense>

#include "gps_state_machine.hpp"

namespace Glider
{

struct GlobalEKFConfig
{
    double sigma_pos = 0.05;
    double sigma_alt = 0.05;
    double sigma_gyro = 0.005;
    double sigma_gyro_bias = 1.0e-5;
    double sigma_mag_bias = 1.0e-5;
    double sigma_compass = 0.14;
    double mag_declination = 0.0;
    double gps_heading_offset = 90.0;
    double nis_threshold = 6.63;

    double seconds_per_tick = 1.0e-9;
    double max_dt = 0.5;

    double init_heading_sigma = 3.15;
    double init_gyro_bias_sigma = 0.02;
    double init_mag_bias_sigma = 0.35;

    double retrust_heading_sigma = 0.5;
    double retrust_mag_bias_sigma = 0.35;

    GpsStateMachineConfig gps_gate;
};

enum GlobalFrame
{
    NED,
    ENU
};

struct GlobalState 
{
    double latitude;
    double longitude;
    double altitude;
    double heading;
    double heading_deg;
};

class GlobalEKF
{
    public:
        using Vector6d = Eigen::Matrix<double,6,1>;
        using Matrix6d = Eigen::Matrix<double,6,6>;

        GlobalEKF() = default;
        explicit GlobalEKF(const GlobalEKFConfig& config);

        void predict(int64_t timestamp, const Eigen::Vector3d& gyro);

        void updateFixStatus(int64_t timestamp, int8_t status);
        void checkGpsTimeout(int64_t now);

        void updatePosition(const Eigen::Vector3d& lla, double horizontal_sigma, double vertical_sigma);
        bool updateCompass(double heading);
        bool updateGpsHeading(const Eigen::Quaterniond& orientation, double sigma);

        void reset();

        template <GlobalFrame F>
        GlobalState state();

        Eigen::Vector3d position() const;

        double heading() const { return x_(3); }
        double gyroBias() const { return x_(4); }
        double magBias() const { return x_(5); }

        double headingVariance() const { return P_(3, 3); }
        const Matrix6d& covariance() const { return P_; }

        bool headingTrusted() const { return heading_trusted_; }
        const GpsStateMachine& gate() const { return gate_; }
        bool positionInitialized() const { return position_initialized_; }
        bool headingInitialized() const { return heading_initialized_; }

    private:
        void setHeadingTrusted(bool trusted);
        bool updateScalar(const Eigen::Matrix<double, 1, 6>& H, double innovation, double r);

        double latScale() const;
        double lonScale() const;

        GlobalEKFConfig config_;
        GpsStateMachine gate_;
        Vector6d x_ = Vector6d::Zero();
        Matrix6d P_ = Matrix6d::Zero();
        int64_t last_predict_time_ = 0;
        bool has_predict_time_ = false;
        bool heading_trusted_ = false;
        bool position_initialized_ = false;
        bool heading_initialized_ = false;
};

}  // namespace Glider
