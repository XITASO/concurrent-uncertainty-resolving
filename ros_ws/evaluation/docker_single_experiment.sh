#!/bin/bash

#  set args
RULE_SET=test_rules.txt 
if [ $# -gt 0 ]
  then
    RULE_SET=$1
fi

docker build . -t mapek_bt -f ./Dockerfile.cuda

echo "Build is done"

docker run -v ./log_dump:/home/dockuser/ros_ws/log_dump -e CONSIDER_DEPENDENCIES=$CONSIDER_DEPENDENCIES -e CONSIDER_CRITICALITY_LEVEL=$CONSIDER_CRITICALITY_LEVEL -e CONSIDER_COST_FUNCTION=$CONSIDER_COST_FUNCTION --rm --name mapek_bt_evaluation mapek_bt evaluation/run_single_experiment.sh --gpus "device=$SLURM_JOB_GPUS"

echo "Fininshed docker run"
