#!/bin/bash

# Number of repetitions
REPEAT=40
managing_system_combinations=(
    "false false false"  # CONSIDER_DEPENDENCIES CONSIDER_CRITICALITY_LEVEL CONSIDER_COST_FUNCTION
    "false false true"
    "false true false"
    "false true true"
    #"true false false"
    #"true false true"
    #"true true false"
    #"true true true"
)

echo "Starting experiment with all managing system combinations..."

for combination in "${managing_system_combinations[@]}"; do
    # Parse the combination into individual variables
    read -r deps crit cost <<< "$combination"
    
    echo "Running with CONSIDER_DEPENDENCIES=$deps, CONSIDER_CRITICALITY_LEVEL=$crit, CONSIDER_COST_FUNCTION=$cost"
    for i in $(seq 1 $REPEAT); do
        echo "Running experiment set $i..."

        # Run the experiment in a new session to isolate it
        CONSIDER_DEPENDENCIES=$deps CONSIDER_CRITICALITY_LEVEL=$crit CONSIDER_COST_FUNCTION=$cost \
        setsid ./ros_ws/evaluation/docker_single_experiment.sh
        echo "experiment $exp of set $i finished. Preparing for next experiment..."
        sleep 5
    done
done

echo "All experiments completed."
