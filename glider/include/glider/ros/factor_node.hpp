/*
*
*
* ROS node header
*/
#pragma once

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Time.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "glider/core/factor_manager.hpp"
#include "glider/utils/params.hpp"
#include "glider/utils/quaternion.hpp"
#include "glider/core/state.hpp"
#include "glider/core/odometry.hpp"

class FactorNode
{
    public:
        FactorNode() = default;
        FactorNode(ros::NodeHandle& nh);

    private:
        glider::FactorManager factor_manager_;

        void interpolationCallback(const ros::TimerEvent& event);
        void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
        void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
        void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
       
        int64_t getTime(const ros::Time& stamp) const;

        ros::NodeHandle nh_;

        ros::Subscriber odom_sub_;
        ros::Subscriber imu_sub_;
        ros::Subscriber gps_sub_;

        ros::Publisher odom_pub_;
        ros::Timer timer_;

        bool initialized_;
        bool use_sim_time_;
        Params params;

        glider::Quaternion prev_quat_;
        Eigen::Vector3d prev_position_;

        Eigen::Matrix3d ned_to_enu_;
};
