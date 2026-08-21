#!/bin/bash

source /opt/ros/jazzy/setup.bash
source /home/dtc/ws/install/setup.bash

if [ "$RUN" = "true" ]; then
    echo "[GLIDER] Launching glider..."
    ros2 launch glider glider-node.launch.py use_sim_time:="${USE_SIM_TIME:-false}"
else
    echo "[GLIDER] RUN=false, keeping container alive..."
fi

exec "$@"
