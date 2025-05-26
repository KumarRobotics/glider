## Glider

Glider is a GPS-LiDAR-INS system. It offers extremely accurate localization without drift and at high rate.

### Running
Glider can be built as ros packages in your ros workspace with `catkin build`.
Run Tanqueray with:
```
roslaunch glider glider.launch
```
You can set the argument: `use_sim_time:=true` if you are using a bag file or simulated sensor.

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

### Authors
Jason Hughes and Varun Murali
