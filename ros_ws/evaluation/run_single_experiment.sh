#!/bin/bash

# Check if `nvidia-smi` command is available
if ! command -v nvidia-smi &> /dev/null; then
    echo "NVIDIA drivers are not installed."
    exit 1
fi

# Check if NVIDIA drivers are currently running
nvidia_smi_output=$(nvidia-smi)
if [[ $? -ne 0 ]]; then
    echo "NVIDIA drivers are not running."
    exit 1
fi

#  set args
SCENARIO_FILE=empty 
if [ $# -gt 0 ]
  then
    SCENARIO_FILE=$1
fi

RULE_SET=full_rules 
if [ $# -gt 0 ]
  then
    RULE_SET=$2
fi

#START_TIME=$((1 + $RANDOM % 250))
#python3 ./evaluation/create_random_timestamp.py $START_TIME
source /ws/install/setup.bash
source /bt_workspace/install/setup.bash
# make sure to read the changes
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug --symlink-install

source ./install/setup.bash

echo "Managing system configuration: CONSIDER_DEPENDENCIES=$CONSIDER_DEPENDENCIES, CONSIDER_CRITICALITY_LEVEL=$CONSIDER_CRITICALITY_LEVEL, CONSIDER_COST_FUNCTION=$CONSIDER_COST_FUNCTION"
nvidia-smi

export START_TIME=$START_TIME
export SCENARIO_FILE=$SCENARIO_FILE
export RULE_SET=$RULE_SET
export CONSIDER_DEPENDENCIES=$CONSIDER_DEPENDENCIES
export CONSIDER_CRITICALITY_LEVEL=$CONSIDER_CRITICALITY_LEVEL
export CONSIDER_COST_FUNCTION=$CONSIDER_COST_FUNCTION

# Start experiment setup
ros2 launch experiment_setup experiment.launch.py &
PID1=$!
PGID1=$(ps -o pgid= -p $PID1 | tr -d ' ')

# Wait for the system to come to live
sleep 10
rate=0.2
# Start the ros bag
ros2 bag play .data/SynDrone_t01_h50.bag --clock  --rate=$rate&
PID3=$!
PGID3=$(ps -o pgid= -p $PID3 | tr -d ' ')

sleep 2

# Start the managing subsystem
ros2 run bt_mape_k bt_executor --ros-args --remap use_sim_time:=true &
#ros2 run bt_mape_k bt_executor -p use_sim_time:=true &
PID2=$!
PGID2=$(ps -o pgid= -p $PID2 | tr -d ' ')

sleep 2

# Run scenario executor individually with specific scenario file
ros2 run experiment_setup scenario_executor seams_eval_example.yaml --ros-args -r __ns:=/experiment_setup -p use_sim_time:=true &
PID4=$!
PGID4=$(ps -o pgid= -p $PID4 | tr -d ' ')

# Define a function to kill the background processes
cleanup() {
    echo "Cleaning up..."
    # Attempt to kill gracefully first
    kill -TERM -- -$PGID1
    kill -TERM -- -$PGID2
    kill -TERM -- -$PGID3
    kill -TERM -- -$PGID4
}

# Set a trap to call the cleanup function on script exit
trap cleanup EXIT

# Simulate script work
echo "All processes have started..."
# 5 seconds init, 30 sec experiment, 5 buffer
sleep 60

echo "Experiment finished."
