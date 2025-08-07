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
#include <gtsam/geometry/Similarity3.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/navigation/ImuBias.h>

#include <Eigen/Dense>
#include <utility>
#include <type_traits>

using gtsam::symbol_shorthand::V; // Velocity
using gtsam::symbol_shorthand::X; // Pose
using gtsam::symbol_shorthand::S; // Similarity

namespace Glider
{


class Odometry
{
    public:
        Odometry() = default;
        Odometry(gtsam::Values& val, gtsam::Key key, double scale, int64_t timestamp, bool init = true);
        Odometry(gtsam::NavState& ns, bool init = true);
        Odometry(gtsam::NavState& ns, Eigen::Vector3d& gyro, bool init = true);

        template<typename T>
        T getPose() const;
        template<typename T>
        T getPosition() const;
        template<typename T>
        T getOrientation() const;
        template<typename T>
        T getVelocity() const;
        template<typename T>
        T getPoseWithScale() const;
        template<typename T>
        T getRelative() const;
        template<typename T>
        T getGyroscope() const;

        static Odometry Uninitialized();

        gtsam::NavState getNavState() const;
        double getAltitude() const;
        double getHeading() const;
        double getHeadingDegrees() const;
        double getScale() const;
        bool isInitialized() const;

        double getLatitude(const char* zone);
        double getLongitude(const char* zone);
        std::pair<double, double> getLatLon(const char* zone);
        int64_t getTimestamp() const;

        void setInitializedStatus(bool init);
        void offsetPose(double x, double y);

    protected:
        template<typename TF, typename TS>
        std::pair<TF,TS> getEigenPose() const;

        double latitude_;
        double longitude_;
                
        gtsam::Vector3 velocity_;
        gtsam::Point3 position_;
        gtsam::Rot3 orientation_;
        gtsam::Pose3 pose_;
        gtsam::Similarity3 sim_;
        Eigen::Vector3d gyro_;

        double altitude_;
        double heading_;

        double scale_;
        
        int64_t timestamp_;

        bool initialized_;
};
} // namespace glider
