/*
* Jason Hughes
* May 2025
*
* Struct to keep track of robots odometry
*/

#pragma once

#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/navigation/ImuBias.h>

#include <Eigen/Dense>
#include <utility>
#include <type_traits>

using gtsam::symbol_shorthand::V; // Velocity
using gtsam::symbol_shorthand::X; // Pose

namespace glider
{


class Odometry
{
    public:
        Odometry() = default;
        Odometry(gtsam::Values& val, gtsam::Key key, bool init = true);
        Odometry(gtsam::NavState& ns, bool init = true);

        template<typename T>
        T getPose() const;
        template<typename T>
        T getPosition() const;
        template<typename T>
        T getOrientation() const;
        template<typename T>
        T getVelocity() const;

        static Odometry Uninitialized();

        gtsam::NavState getNavState() const;
        double getAltitude() const;
        double getHeading() const;
        double getHeadingDegrees() const;
        bool isInitialized() const;

        double getLatitude(const char* zone);
        double getLongitude(const char* zone);
        std::pair<double, double> getLatLon(const char* zone);

        void setInitializedStatus(bool init);

    protected:
        template<typename TF, typename TS>
        std::pair<TF,TS> getEigenPose() const;

        double latitude_;
        double longitude_;
                
        gtsam::Vector3 velocity_;
        gtsam::Point3 position_;
        gtsam::Rot3 orientation_;
        gtsam::Pose3 pose_;

        double altitude_;
        double heading_;

        bool initialized_;
};
} // namespace glider
