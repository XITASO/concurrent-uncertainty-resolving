#!/bin/bash

# Number of repetitions
REPEAT=54

if [ "$1" == "gpu" ] || [ "$1" == "cpu" ];
then
    used_device=$1
else
    echo "Please provide the device you want to run this script on: gpu or cpu"
    exit 1
fi

if [ "$1" = "cpu" ]; then
    USE_CPU=true
else
    USE_CPU=false
fi

# Optional: Control whether success rates and probabilities should be updated
# Values: 1 or true to enable (default), 0 or false to disable
export MAPEK_UPDATE_RATES_PROBABILITIES=1
managing_system_combinations=(
    "false false false"  # CONSIDER_DEPENDENCIES CONSIDER_CRITICALITY_LEVEL CONSIDER_COST_FUNCTION
    "false false true"
    "false true false"
    "false true true"
    "true false false"
    "true false true"
    "true true false"
    "true true true"
)

echo "Starting experiment with all managing system combinations..."

for combination in "${managing_system_combinations[@]}"; do
    # Parse the combination into individual variables
    read -r deps crit cost <<< "$combination"
    
    echo "Running with CONSIDER_DEPENDENCIES=$deps, CONSIDER_CRITICALITY_LEVEL=$crit, CONSIDER_COST_FUNCTION=$cost"
    for i in $(seq 1 $REPEAT); do
        echo "Running experiment set $i..."

        # Run the experiment in a new session to isolate it
        CONSIDER_DEPENDENCIES=$deps CONSIDER_CRITICALITY_LEVEL=$crit CONSIDER_COST_FUNCTION=$cost MAPEK_UPDATE_RATES_PROBABILITIES=$MAPEK_UPDATE_RATES_PROBABILITIES\
        USE_CPU=$USE_CPU \
        setsid ./ros_ws/evaluation/docker_single_experiment.sh
        echo "experiment $exp of set $i finished. Preparing for next experiment..."
        sleep 5
    done
done

echo "All experiments completed."
