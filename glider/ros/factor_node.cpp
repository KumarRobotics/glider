/*
*
* ROS node logic
*/

#include "glider/ros/factor_node.hpp"
#include "glider/ros/node_utils.hpp"

FactorNode::FactorNode(ros::NodeHandle& nh) : nh_(nh)
{
    double freq;
    ned_to_enu_ = (Eigen::Matrix3d() << 
                 0.0, 1.0, 0.0,
                 1.0, 0.0, 0.0,
                 0.0, 0.0, -1.0).finished();

    nh_.getParam("/glider_node/rate", freq);
    ROS_DEBUG_STREAM("Useing prediction rate: "<< freq);
    
    std::string path;
    nh_.getParam("/glider_node/path", path);
    ROS_DEBUG_STREAM("Loading graph params from: "<< path);
    
    nh_.getParam("/use_sim_time", use_sim_time_);
    bool use_odom;
    nh_.getParam("/use_odom", use_odom);

    std::map<std::string, double> config;
    config = params.load<double>(path);
    int64_t time_now = this->getTime(ros::Time::now());
    factor_manager_ = glider::FactorManager(config, time_now);

    gps_sub_ = nh_.subscribe("/gps", 1, &FactorNode::gpsCallback, this);
    imu_sub_ = nh_.subscribe("/imu", 10, &FactorNode::imuCallback, this);
    if (use_odom)
    {
        odom_sub_ = nh_.subscribe("/odom", 1, &FactorNode::odomCallback, this);
    }

    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/glider/odom", 10);

    ros::Duration d = rosutil::hzToDuration(freq);
    timer_ = nh_.createTimer(d, &FactorNode::interpolationCallback, this);
    ROS_INFO("[GLIDER] Factor Manager Node Initialized");
}

int64_t FactorNode::getTime(const ros::Time& stamp) const
{
   return (static_cast<int64_t>(stamp.sec) * 1000000000LL) + static_cast<int64_t>(stamp.nsec);
}

void FactorNode::interpolationCallback(const ros::TimerEvent& event)
{
    if (!initialized_) return;
    
    int64_t timestamp = getTime(ros::Time::now());
    auto [position, orientation] = factor_manager_.predict(timestamp);

    if (position.norm() > 0)
    {
        nav_msgs::Odometry odom_msg;
        
        odom_msg.pose.pose.position.x = position(0);
        odom_msg.pose.pose.position.y = position(1);
        odom_msg.pose.pose.position.z = position(2);

        odom_msg.pose.pose.orientation.w = orientation(0);
        odom_msg.pose.pose.orientation.x = orientation(1);
        odom_msg.pose.pose.orientation.y = orientation(2);
        odom_msg.pose.pose.orientation.z = orientation(3);

        odom_pub_.publish(odom_msg);
    }
}

void FactorNode::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    if (!initialized_)
    {
        prev_position_ << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
        prev_quat_ = glider::Quaternion(msg->pose.pose.orientation);
    }
    // calculate the difference between current and prev
    Eigen::Vector3d current_position(msg->pose.pose.position.x,
                                     msg->pose.pose.position.y,
                                     msg->pose.pose.position.z);
    glider::Quaternion current_quat(msg->pose.pose.orientation);

    Eigen::Vector3d between_position = (current_position - prev_position_);
    glider::Quaternion between_quat = current_quat - prev_quat_;

    int64_t timestamp;
    if (use_sim_time_)
    {
        timestamp = getTime(ros::Time::now());
    }
    else
    {
        timestamp = getTime(msg->header.stamp);
    }

    factor_manager_.addOdometryFactor(timestamp, between_position, between_quat.toEigen());

    prev_position_ = current_position;
    prev_quat_ = current_quat;
}

void FactorNode::gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{

    Eigen::Vector3d gps_factor;
    gps_factor(0) = msg->latitude;
    gps_factor(1) = msg->longitude;
    gps_factor(2) = msg->altitude;
   
    int64_t timestamp;
    if (use_sim_time_)
    {
        std::cout << "using sim time" << std::endl;
        timestamp = getTime(ros::Time::now());
    }
    else
    {
        timestamp = getTime(msg->header.stamp);
    }
    //std::cout << "GPS timestamp " << timestamp << std::endl;
    factor_manager_.addGpsFactor(timestamp, gps_factor);
    //std::cout << "optimizing with time " << timestamp << std::endl;
    
    glider::State state = factor_manager_.runner();
    //auto [position, quaternion, rotation] = factor_manager_.runner();
    //std::cout << "[GLIDER] Heading: " << state.getHeadingDegrees() << std::endl;    
    if (!initialized_) initialized_ = true;
}

void FactorNode::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    if (!initialized_) return;

    Eigen::Vector3d gyro(msg->angular_velocity.x,
                         msg->angular_velocity.y,
                         msg->angular_velocity.z);
    
    Eigen::Vector3d accel(msg->linear_acceleration.x,
                          msg->linear_acceleration.y,
                          msg->linear_acceleration.z);
    
    //glider::Quaternion quat(msg->orientation.w,
    //                        msg->orientation.x,
    //                        msg->orientation.y,
    //                        msg->orientation.z);
    Eigen::Quaterniond quat(msg->orientation.w,
                            msg->orientation.x,
                            msg->orientation.y,
                            msg->orientation.z);

    Eigen::Matrix3d rot_enu = ned_to_enu_ * quat.toRotationMatrix() * ned_to_enu_.transpose();
    quat = Eigen::Quaterniond(rot_enu);
    quat.normalize();

    int64_t timestamp;
    if (use_sim_time_)
    {
        timestamp = getTime(ros::Time::now());
    }
    else
    {
        timestamp = getTime(msg->header.stamp);
    }
    //std::cout << "IMU timestamp " << timestamp << std::endl;
    Eigen::Vector4d q_vec(quat.w(), quat.x(), quat.y(), quat.z());
    factor_manager_.addImuFactor(timestamp, accel, gyro, q_vec);
}


