# Glider
![Humble CI](https://github.com/KumarRobotics/glider/actions/workflows/humble-ci.yml/badge.svg?branch=ros2)

![Jazzy CI](https://github.com/KumarRobotics/glider/actions/workflows/jazzy-ci.yml/badge.svg?branch=ros2)

Glider is a G-INS system built on [GTSAM](https://github.com/borglab/gtsam). It accepts GPS, a 9-DOF IMU, and optional local odometry, and provides a full
state estimate at up to the IMU rate. Glider is designed to be configured for a specific sensor suite.

## Building Glider
To run glider you can use the provided docker images, ROS2 jazzy and humble are both supported, simply use the `build.bash` and `run.bash` files. You can mounted volumes in the run files if necessary. If you want to inlcude this in another ROS2 workspace, you may need to install the following dependencies:
```
apt install ros-$ROS_DISTRO-gtsam libgoogle-glog-dev libgest-dev
```
Glider can be build with colcon as a ROS2 package with:
```
colcon build --packages-select glider
```
If you only want the api you can build with:
```
cmake -S . -B build -DBUILD_ROS=OFF
cmake --build build
```

## Running Glider
To run glider with ros use `ros2 run glider glider-node.launch.py`. If you are running with a bag file use:
```
ros2 bag run <bag_file_name> --clock
ros2 launch glider glider-node.launch.py use_sim_time:=true
```

## Hardware Setup
You're setup needs a GPS and a 9-DOF IMU, that is an IMU that provides a full orientation. The IMU orientation should be provided in the IMU's frame
as this is standard for robotics, but we are working on supporting the NED frame. We use a VectorNav VN100T IMU. It is important make sure your IMU magnetometer is calibrated, if it is not aligned correctly the heading output of glider will be incorrect.

## ROS2 Setup
We recommend using Glider with ROS2, you can configure the ros parameters in `config/ros-params.yaml`. Here's more detail about what
the parameters mean:
 - `publishers.rate`: the rate at which odometry is published in hz, setting this to 0 publishes at IMU rate.
 - `publishers.nav_sat_fix`: if true will publish the odometry as a `NavSatFix` msg, the default is an `Odometry` msg.
 - `publishers.viz.use`: if true will publish an `Odometry` topic for visualization centered around the origin.
 - `publishers.viz.origin_easting`: the easting value you want to viz odometry to center around.
 - `publishers.viz.origin_northing`: the northing value you want the viz odometry to center around.
 - `subscribers.imu_topic`, `gps_topic`, `dgps_topic`, `odom_topic`: input topic names.
 - `subscribers.use_gps`, `use_dgps`, `use_odom`: enable each aiding source. Avoid enabling both GPS inputs when they represent the same receiver fix.
 - `subscribers.gps_rejection_variance`: hard GPS variance ceiling in m². Measurements above it are rejected while local odometry remains active.

The checked-in ROS parameters use a VectorNav IMU on `/vectornav/imu`, LIO on
`/rko_lio/odometry`, and the ENU DGPS fix on `/sept/enu/dfix`. LIO remains active
while GPS is absent or rejected.

## Glider Setup
You can configure glider itself in `config/glider-params.yaml`, this is where you can specify the parameters for the factor graph. Here's more detail on each parameter:
#### IMU Parameters
 - `covariances.accelerometer`: covariance of the accelerometer.
 - `covariances.gyroscope`: covariance of the gyroscope.
 - `covariances.integration`: covariance of the IMU preintegration.
 - `covariances.heading`: covariance of the IMU's magnetometer heading in radians, 0.09 radians is about 5 degrees.
 - `covariances.roll_pitch`: covariance of the roll and pitch angles in radians.
 - `covariances.bias`: covariance of the bias estimate.
 - `frame`: What frame the IMU is in, either `enu` or `ned`.
#### GPS Parameters
 - `gps.covariance`: covariance of the gps position estimate.
#### Other Parameters
 - `constants.gravity`: gravity in your IMU's frame.
 - `constants.bias_num_measurements`: number of IMU measurements to use to initially estimate the bias.
 - `constants.initial_num_measurements`: number of times to let the factor graph optimize before glider starts reporting odometry.
 - `logging.stdout`: output log statements to terminal in addition to the logfile
 - `optimizer.smooth`: if true the factor graph will optimize using a fixed lag smoother, otherwise it will use iSAM2.
 - `optimizer.lag_time`: period of time the fixed lag smoother should look at in seconds.
 - `extrinsics`: the single hardware-calibration section. Sensor poses are entered relative to the LiDAR using ROS FLU axes (+X forward, +Y left, +Z up), metres, and XYZ roll/pitch/yaw degrees. Glider converts them to its body frame internally.

### Building and Running Unit Tests
We use GTest to run unit tests. You can build the tests with
```
cd glider
cmake -S . -B build -DBUILD_TESTS=ON
cmake --build build
```
and run with:
```
cd build
ctest
```
Note these tests are run on PR's and pushes to the `ros2` branch.
