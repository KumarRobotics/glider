#pragma once

#include <cstdint>

#include <Eigen/Dense>

#include "gps_state_machine.hpp"

namespace Glider
{

struct GlobalEKFConfig
{
    double sigma_heading = 0.1;

    double sigma_compass = 0.1;
    double sigma_gps_heading = 0.02;
    double mag_declination = 0.0;
    double gps_heading_offset = 0.0;
    double nis_threshold = 1e9;

    double seconds_per_tick = 1.0e-9;
    double max_dt = 1.0;

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
        GlobalEKF() = default;
        explicit GlobalEKF(const GlobalEKFConfig& config);

        void updateFixStatus(int64_t timestamp, int8_t status);
        void checkGpsTimeout(int64_t now);

        void updatePosition(const Eigen::Vector3d& lla, double horizontal_sigma, double vertical_sigma);
        bool updateCompass(int64_t timestamp, double heading);
        bool updateGpsHeading(int64_t timestamp, const Eigen::Quaterniond& orientation);

        template <GlobalFrame F>
        GlobalState state() const;

        void reset();

        // degrees, degrees, meters
        Eigen::Vector3d position() const;
        Eigen::Matrix3d positionCovariance() const;

        double heading() const { return heading_; }
        double headingVariance() const { return heading_variance_; }

        bool headingTrusted() const { return gate_.trusted(); }
        const GpsStateMachine& gate() const { return gate_; }
        bool positionInitialized() const { return position_initialized_; }
        bool headingInitialized() const { return heading_initialized_; }

    private:
        void propagate(int64_t timestamp);
        bool updateHeading(double z, double r);
        void initHeading(int64_t timestamp, double z, double r);

        GlobalEKFConfig config_;
        GpsStateMachine gate_;

        Eigen::Vector3d position_ = Eigen::Vector3d::Zero();
        Eigen::Vector3d position_variance_ = Eigen::Vector3d::Zero();

        double heading_ = 0.0;
        double heading_variance_ = 0.0;

        int64_t last_heading_time_ = 0;
        bool position_initialized_ = false;
        bool heading_initialized_ = false;
};

}  // namespace Glider
