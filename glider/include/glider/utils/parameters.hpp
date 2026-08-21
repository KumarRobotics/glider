/*
* Jason Hughes
* July 2025
*
* Glider parameters loaded from a yaml file.
* These parameters are for configuring the factor graph
* and glider usage.
*/

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <string>

namespace Glider
{

struct Parameters
{
    /* @brief default constructor for Parameters
    */
    Parameters() = default;
    /* @brief main constructor that will load parameters from
    *  yaml config file
    *  @param path: path to the yaml file
    */
    Parameters(const std::string& path);
    /* @brief static method that calls the constructor to load
    *  the parameters, for readability.
    *  @param path: path to the yaml file
    */
    static Parameters Load(const std::string& path);

    // @brief accelerometer covariance of the IMU
    double accel_cov;
    // @brief gyroscope covariance of the IMU
    double gyro_cov;
    // @brief covariance of the compass of the IMU
    double heading_cov;
    // @brief covariance of the roll and pitch orientation
    // estimate from the IMU
    double roll_pitch_cov;
    // @brief covariance of IMU integration
    double integration_cov;
    // @brief covariance of the IMU's bias estimate
    double bias_cov;
    // @brief covariance of the GPS position estimate
    // TODO make this gps_cov to match
    double gps_noise;
    // @brief covariance of the odometry position estimate
    double odom_cov;

    // @brief gravity as read from your IMU
    double gravity;
    // @brief number of IMU measurements to initially measure the IMUs bias
    int bias_num_measurements;
    // @brief number of GPS measurements to add to and optimize over
    // before we start publishing results
    uint64_t initial_num_measurements;

    // @brief the frame of the IMU, we only handle ENU and NED
    std::string frame;

    // @brief if true this logs to stdout and log file otherwise it logs
    // just to a file
    bool log;
    // @brief the directory to save the log file, glog needs an
    // absolute path
    std::string log_dir;

    // @brief if true optimize using a fixed lag smoother, otherwise
    // optimize with iSAM2
    bool smooth;
    // @brief amount of time in seconds for the fixed lag smoother to
    // smooth over.
    double lag_time;

    // @brief wheather or not to integrate differential gps from motion heading,
    // if false orientation from the IMU will be integrated
    bool use_dgpsfm;
    // @brief velocity in m/s that the robot should be moving at to integrate
    // dgpsfm
    double dgpsfm_threshold;
    // @brief heading noise for differential gps from motion
    double dgpsfm_cov;

    bool use_dgps;
    double dgps_cov;
    // Sensor poses transformed from the configured reference frame to body.
    Eigen::Vector3d t_body_imu;
    Eigen::Matrix3d r_body_imu;
    Eigen::Vector3d t_body_gps;
    double gps_heading_offset;
    std::string body_frame;
};
}
