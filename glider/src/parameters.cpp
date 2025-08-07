/*
* Jason Hughes
* July 20205
*
* glider-mono parameters
*/

#include "glider/utils/parameters.hpp"


Glider::Parameters::Parameters(const std::string& path)
{
    try
    {
        YAML::Node config = YAML::LoadFile(path);

        accel_cov = config["accelerometer_covariance"].as<double>();
        gyro_cov = config["gyroscope_covariance"].as<double>();
        integration_cov = config["integration_covariance"].as<double>();
        bias_cov = config["bias_covariance"].as<double>();
        use_second_order = config["use_second_order"].as<double>();
        origin_x = config["origin_x"].as<double>();
        origin_y = config["origin_y"].as<double>();
        gravity = config["gravity"].as<double>();
        gps_noise = config["gps_noise"].as<double>();
        heading_noise = config["heading_noise"].as<double>();
        odom_noise = config["odom_noise"].as<double>();
        lag_time = config["lag_time"].as<double>();
        odom_orientation_noise = config["odom_orientation_noise"].as<double>();
        odom_translation_noise = config["odom_translation_noise"].as<double>();
        odom_scale_noise = config["odom_scale_noise"].as<double>();

        bias_num_measurements = config["bias_num_measurements"].as<int>();

        frame = config["imu_frame"].as<std::string>();
        scale_odom = config["scale_odom"].as<bool>();
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
