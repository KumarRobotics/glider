/*
* Jason Hughes
* May 2025
*
*
* track the odometry and its initialization status
*/

#pragma once

#include <gtsam/geometry/Pose3.h>

namespace glider
{

struct Odometry
{
    public:
        Odometry() = default;
        Odometry(gtsam::Pose3 p, bool init);

        gtsam::Pose3 pose;

        bool isInitialized() const;

    private:
        bool is_initialized;
};
}
