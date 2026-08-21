/*
* Jason Hughes
* July 20205
*
* glider-mono parameters
*/

#include "glider/utils/parameters.hpp"

#include <cmath>

namespace
{
Eigen::Vector3d loadTranslation(const YAML::Node& node)
{
    return {node["x"].as<double>(), node["y"].as<double>(), node["z"].as<double>()};
}

Eigen::Matrix3d loadRpyDegrees(const YAML::Node& node)
{
    const double scale = M_PI / 180.0;
    const Eigen::AngleAxisd roll(node["roll"].as<double>() * scale, Eigen::Vector3d::UnitX());
    const Eigen::AngleAxisd pitch(node["pitch"].as<double>() * scale, Eigen::Vector3d::UnitY());
    const Eigen::AngleAxisd yaw(node["yaw"].as<double>() * scale, Eigen::Vector3d::UnitZ());
    return (yaw * pitch * roll).toRotationMatrix();
}
}


Glider::Parameters::Parameters(const std::string& path)
{
    try
    {
        YAML::Node config = YAML::LoadFile(path);

        // covaraiances
        accel_cov = config["imu"]["covariances"]["accelerometer"].as<double>();
        gyro_cov = config["imu"]["covariances"]["gyroscope"].as<double>();
        heading_cov = config["imu"]["covariances"]["heading"].as<double>();
        roll_pitch_cov = config["imu"]["covariances"]["roll_pitch"].as<double>();
        integration_cov = config["imu"]["covariances"]["integration"].as<double>();
        bias_cov = config["imu"]["covariances"]["bias"].as<double>();
        gps_noise = config["gps"]["covariance"].as<double>();
        odom_cov = config["odom"]["covariance"].as<double>();

        // constants
        gravity = config["constants"]["gravity"].as<double>();
        bias_num_measurements = config["constants"]["bias_num_measurements"].as<int>();
        initial_num_measurements = config["constants"]["initial_num_measurements"].as<uint64_t>();

        frame = config["imu"]["frame"].as<std::string>();

        log = config["logging"]["stdout"].as<bool>();
        log_dir = config["logging"]["directory"].as<std::string>();

        smooth = config["optimizer"]["smooth"].as<bool>();
        lag_time = config["optimizer"]["lag_time"].as<double>();

        use_dgpsfm = config["dgpsfm"]["enable"].as<bool>();
        dgpsfm_threshold = config["dgpsfm"]["integration_threshold"].as<double>();
        dgpsfm_cov = config["dgpsfm"]["covariance"].as<double>();

        use_dgps = config["dgps"]["enable"].as<bool>();
        dgps_cov = config["dgps"]["covariance"].as<double>();

        const YAML::Node extrinsics = config["extrinsics"];
        const Eigen::Vector3d t_reference_body = loadTranslation(extrinsics["body"]["translation"]);
        const Eigen::Matrix3d r_reference_body = loadRpyDegrees(extrinsics["body"]["rotation_rpy_deg"]);
        const auto toBodyTranslation = [&](const YAML::Node& sensor) {
            return r_reference_body.transpose() * (loadTranslation(sensor["translation"]) - t_reference_body);
        };
        const auto toBodyRotation = [&](const YAML::Node& sensor) {
            return r_reference_body.transpose() * loadRpyDegrees(sensor["rotation_rpy_deg"]);
        };

        body_frame = extrinsics["body"]["frame"].as<std::string>();
        t_body_imu = toBodyTranslation(extrinsics["imu"]);
        r_body_imu = toBodyRotation(extrinsics["imu"]);
        t_body_gps = toBodyTranslation(extrinsics["gps"]);
        gps_heading_offset = extrinsics["gps"]["heading_offset_deg"].as<double>() * M_PI / 180.0;
    }
    catch (const YAML::Exception& e)
    {
        throw std::runtime_error("Error loading YAML File at : " + path + " : " + std::string(e.what()));
    }
    catch (const std::exception& e)
    {
        throw std::runtime_error("Error parsing YAML configuration at: " + path + " : " + std::string(e.what()));
    }
}

Glider::Parameters Glider::Parameters::Load(const std::string& path)
{
    Parameters params(path);
    return params;
}
