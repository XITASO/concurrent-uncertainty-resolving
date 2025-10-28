#!/bin/bash

#START=$((1 + $RANDOM % 250))
#
#echo $START
#
#python3 evaluation/create_random_timestamp.py $START
#
#sleep 1

setsid CONSIDER_DEPENDENCIES=true CONSIDER_CRITICALITY_LEVEL=true CONSIDER_COST_FUNCTION=false evaluation/run_single_experiment.sh