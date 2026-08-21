/*!
 *
 * Jason Hughes
 * May 2025
 *
 * Struct to keep track of the odometry output
 * from the factor graph. This keeps track of
 * everything the gtsam NavState does but adds
 * a timestamp, gyroscope reading and initilization
 * status.
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
        /*! @brief default constructor of Odometry object, note that this
         *  sets initialized_ to false */
        Odometry() = default;
        /*! @brief initialize the Odometry from the result of optimization, designed to used
         *  by child class upon inheritance
         *  @param val: results from gtsam optimization
         *  @param timestamp: timestamp passed to the optimizer
         *  @param key: current key_index to get current results from val
         *  @param init: should this constuctor call initialize the odometry */
        Odometry(gtsam::Values& val, int64_t timestamp, gtsam::Key key, bool init = true);
        /*! @brief initialize the Odometry from a NavState, likely from calling the the pim
         *  predict
         *  @param ns: the current NavState from gtsam
         *  @param timestamp: the current timestamp
         *  @param init: should the constructor call initialize the odometry */
        Odometry(gtsam::NavState& ns, int64_t timestamp, bool init = true);

        /*! @brief gets the full pose from odometry
         *  @return the 3D position in UTM frame and orientation in ENU frame
         *  @type T: gtsam::Pose3
         *  @type T: Eigen::Isometry3d
         *  @type T: std::pair<Eigen::Vector3d, Eigen::Vector4d>
         *  @type T: std::pair<Eigen::Vector3d, Eigen::Quaterniond> */
        template<typename T>
        T getPose() const;
        /*! @brief gets the 3D positon from odometry
         *  @return position in 3D in UTM ENU frame (easting, northing, altitude)
         *  @type T: gtsam::Point3
         *  @type T: Eigen::Vector3d*/
        template<typename T>
        T getPosition() const;
        /*! @brief gets the 3D orientation from odometry
         *  @return the orientation in ENU frame as rotation matrix or Quaternion
         *  @type T: gtsam::Rot3
         *  @type T: gtsam::Quaternion
         *  @type T: Eigen::Vector4d
         *  @type T: Eigen::Quaterniond */
        template<typename T>
        T getOrientation() const;
        /* @brief gets the 3D velocity from odometry
         * @return the 3D velocity in FLU frame
         * @type T: gtsam::Vector3
         * @type T: Eigen::Vector3d */
        template<typename T>
        T getVelocity() const;

        /*! @brief static method to return an uninitialized Odometry
         *  same as calling the default constructor but good for readability
         *  @return uninitialized Odometry */
        static Odometry Uninitialized();

        /*! @brief get the full gtsam NavState, that is pose and velocity
         *  @return gtsam NavState */
        gtsam::NavState getNavState() const;
        /*! @brief get the altitude from odometry
         *  @return altitude in meters in whatever frame is input */
        double getAltitude() const;
        /*! @brief get the heading from odometry
         *  @return heading in radians in ENU frame */
        double getHeading() const;
        /*! @brief get the heading from odometry
         *  @return heading in degrees in ENU frame */
        double getHeadingDegrees() const;
        /*! @brief check if the odometry is initialized
         *  @return true if odometry is initialized otherwise false */
        bool isInitialized() const;

        /*! @brief check if the gps offset is initialized
         *  @return true if it is otherwise false */
        bool isGpsOffsetInitialized() const;

        /*! @brief get the latitude of the current position
         *  @param zone: the utm zone ex "18S"
         *  @return the latitude in degrees decimal from the UTM position */
        double getLatitude(const char* zone);
        /*! @brief get the longitude of the current position
         *  @param zone: the utm zone, ex "18S"
         *  @return the longitude in degrees decimal from the UTM position */
        double getLongitude(const char* zone);
        /*! @brief get the latitude and longitude as a pair
         *  @param zone: the utm zone, ex "18S"
         *  @return latitude and longitude in degrees decimal as a pair
         *  where lat is first and lon is second */
        std::pair<double, double> getLatLon(const char* zone, const Eigen::Vector3d& offset = Eigen::Vector3d::Zero());
        /*! @brief get the timestamp of the odometry
         *  @return nanosec time in integer format */
        int64_t getTimestamp() const;

        /*! @brief set the initalization status
         *  @param init: true if you want the odometry to be initialized
         *  otherwise false */
        void setInitializedStatus(bool init);

        /*! @brief set the gps offset initalization status
         *  @param init: true if you want the gps offset to be initialized
         *  otherwise false */
        void setGpsOffsetInitialized(bool init);

    protected:
        /*! @brief a helper function to convert gtsam Pose3 to a pair
         *  of Eigen objects
         *  @return a pair of Eigen objects where first is position and second in orientation as a quaternion
         *  @type TF: Eigen::Vector3d
         *  @type TS: Eigen::Vector4d
         *  @type TS: Eigen::Quaterniond */
        template<typename TF, typename TS>
        std::pair<TF,TS> getEigenPose() const;

        // @brief the latitude from the input UTM pose
        double latitude_;
        // @brief the longitude from the input UTM pose
        double longitude_;

        // @brief the 3D velocity in m/s
        gtsam::Point3 velocity_;
        // @brief the 3D position in UTM and ENU frames
        gtsam::Point3 position_;
        // @brief the 3D orientation in ENU frame
        gtsam::Rot3 orientation_;
        // @brief the full 3D pose in ENU frame
        gtsam::Pose3 pose_;

        // @brief the altitude in meters in the input frame
        double altitude_;
        // @brief the heading in radians in ENU frame
        double heading_;

        // @breif nanosec timestamp
        int64_t timestamp_;
        // @brief is this initialized, default to false
        bool initialized_{false};
        // @brief is the gps offset initialized, default to false
        bool is_gps_offset_initialized_{false};
};
} // namespace glider
