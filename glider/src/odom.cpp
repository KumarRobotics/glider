/*
* Jason Hughes
* May 2025
*
*/

#include "glider/utils/odom.hpp"

using namespace glider;

Odometry::Odometry(gtsam::Pose3 p, bool init)
{
    this->pose = p;
    this->is_initialized = init;
}

bool Odometry::isInitialized() const
{
    return is_initialized;
}
