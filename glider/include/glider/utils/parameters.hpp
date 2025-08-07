/*
* Jason Hughes
* July 2025
*
* glider-mono parameters loaded from a yaml file.
*/

#include <yaml-cpp/yaml.h>
#include <string>

namespace Glider
{

struct Parameters
{
    Parameters() = default;
    Parameters(const std::string& path);
    static Parameters Load(const std::string& path);

    double accel_cov;
    double gyro_cov;
    double integration_cov;
    double bias_cov;
    double use_second_order;
    double origin_x;
    double origin_y;
    double gravity;
    double gps_noise;
    double heading_noise;
    double odom_noise;
    double lag_time; 
    double odom_orientation_noise;
    double odom_translation_noise;
    double odom_scale_noise;

    int bias_num_measurements;

    std::string frame;

    bool scale_odom;
};
}
