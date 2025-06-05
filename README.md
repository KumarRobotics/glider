## Glider

Glider is a GPS-Odometry-INS system, it is designed to fuse GPS and IMU (a GINS system) with any odometry you provide. This can also be run as a standard GINS system by setting `use_odom:=false`. Noise parameters can be configured in `config/graph_params.yaml`. There is also a ROS2 branch which is still under active development. 

### Running
Glider can be built as a ros packages in your ros workspace with `catkin build`.
Run glider with:
```
roslaunch glider glider.launch
```
You can set the argument: `use_sim_time:=true` if you are using a bag file and use the `--clock` flag when starting your bag file.

Everything will be rotated into the ENU frame, includeing the IMU orientation and the Odometry pose. Since GPS is the main prior, we publish state estimates in the ENU frame with UTM coordinates as an `Odometry` ros message or a `NavSatFix` message. This can be configured in `config/ros_params.yaml` with the `publish_nav_sat_fix` parameter.

### Building and Running Unit Tests
We use GTest to run unit tests. You can build the tests with 
``` 
cmake -S . -B build
cmake --build build
```
and run with:
```
cd build 
ctest
```

#### Authors
 - **Corresponding:** Jason Hughes jasonah.at.seas.upenn.edu
 - Varun Murali
