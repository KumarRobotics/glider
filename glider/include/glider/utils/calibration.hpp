/*!
* Jason Hughes
* September 2025
*
*/

#pragma once

#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <yaml-cpp/yaml.h>
#include <iostream>

#include "glider/utils/geodetics.hpp"
#include "glider/utils/gps_heading.hpp"

namespace Glider
{
enum class AngleUnit
{
    Degrees,
    Radians
};

class HeadingCalibrator
{
    public:
        HeadingCalibrator() = default;
        HeadingCalibrator(const std::string& path);

        void addGPSMeasurement(const int64_t timestamp, const Eigen::Vector3d meas);
        void addMagMeasurement(const int64_t timestamp, const Eigen::Vector3d mag);
        void addIMUMeasurement(const int64_t timestamp, const Eigen::Vector4d quat);
        Eigen::Vector4d applyCalibration(const Eigen::Vector4d& input) const;

        bool isCalibrated() const;
        template<AngleUnit U>
        double getHeadingDifference() const;

    private:

        struct GPSStamped
        {
            GPSStamped(const int64_t ts, const Eigen::Vector3d meas)
            {
                timestamp = ts;
                gps = meas;
            }
            
            GPSStamped(const int64_t ts, const Eigen::Vector3d meas, const double h) 
            {
                timestamp = ts;
                gps = meas;
                heading = h;
            }

            Eigen::Vector3d gps;
            int64_t timestamp;
            double heading{0.0};
        };

        struct MagStamped
        {
            MagStamped(const int64_t ts, const Eigen::Vector3d meas)
            {
                timestamp = ts;
                mag = meas;
                heading = std::atan2(meas(1), meas(0));
            }
            int64_t timestamp;
            Eigen::Vector3d mag;
            double heading;
        };

        struct IMUStamped
        {
            IMUStamped() = default;
            IMUStamped(const int64_t ts, const Eigen::Vector4d meas)
            {
                timestamp = ts;
                quaternion = meas;
                Eigen::Quaterniond quat(meas(0), meas(1), meas(2), meas(2));
                Eigen::Vector3d euler = quat.toRotationMatrix().eulerAngles(0, 1, 2);
                heading = euler(2);
            }

            int64_t timestamp;
            Eigen::Vector4d quaternion;
            double heading;
        };

        struct Parameters
        {
            Parameters() = default;
            Parameters(const std::string& path);
            static Parameters Load(const std::string& path);

            size_t num_measurements;
            double min_distance;
        };

        double calibrate();
        double measureDistance(const Eigen::Vector3d& meas) const;
        IMUStamped getProximalIMUMeasurement(int64_t timestamp) const;
        
        Parameters params_;
        std::vector<GPSStamped> gps_measurements_;
        std::vector<IMUStamped> imu_measurements_;
        std::vector<MagStamped> mag_measurements_;

        double heading_diff_{0.0};
        bool calibrated_{false};
        bool print_{true};
};
}
