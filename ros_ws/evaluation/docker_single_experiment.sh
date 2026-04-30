#!/bin/bash

#  set args
RULE_SET=test_rules.txt 
if [ $# -gt 0 ]
  then
    RULE_SET=$1
fi

if [ "$USE_CPU" = "false" ];
then
    docker build . -t mapek_bt -f ./Dockerfile.cuda
    if [ -n "${SLURM_JOB_GPUS:-}" ]; then
        docker_arg="--gpus device=${SLURM_JOB_GPUS}"
    else
        docker_arg="--gpus=all"
    fi

    echo "Build cuda is done"
else
    docker build . -t mapek_bt -f ./Dockerfile
    docker_arg=""
    echo "Build cpu is done"
fi

echo "Updating success probabilities enabled:" $MAPEK_UPDATE_RATES_PROBABILITIES
docker run $docker_arg -v ./log_dump:/home/dockuser/ros_ws/log_dump -e CONSIDER_DEPENDENCIES=$CONSIDER_DEPENDENCIES -e CONSIDER_CRITICALITY_LEVEL=$CONSIDER_CRITICALITY_LEVEL -e CONSIDER_COST_FUNCTION=$CONSIDER_COST_FUNCTION -e MAPEK_UPDATE_RATES_PROBABILITIES=$MAPEK_UPDATE_RATES_PROBABILITIES --rm --name mapek_bt_evaluation mapek_bt evaluation/run_single_experiment.sh 

echo "Finished docker run"
