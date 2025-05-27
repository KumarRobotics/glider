## Glider

Glider is a GPS-LiDAR-INS system, it is designed to fuse GPS and IMU (an GINS system) with any odometry you provide where the position is in meters. This can also be run as a standard GINS system by setting `use_odom:=false`. Noise parameters can be configured in `config/graph_params.yaml`. There is also a ROS2 branch which is still under active development. 

### Running
Glider can be built as ros packages in your ros workspace with `catkin build`.
Run glider with:
```
roslaunch glider glider.launch
```
You can set the argument: `use_sim_time:=true` if you are using a bag file and use the `--clock` flag when starting your bag file.

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
