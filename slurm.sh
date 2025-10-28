#!/bin/bash

# ------- Example resources --- #
#SBATCH --gres=shard:30
#SBATCH --job-name=mapek-bt
# --- End Example resources --- #

handle_kill() {
    docker stop mapek_bt_evaluation
    docker rm mapek_bt_evaluation
    echo "Docker container stopped and removed."
    exit 0
}
trap 'handle_kill' SIGINT SIGTERM

./ros_ws/evaluation/run_multi_experiment.sh

# Wait for the container to exit or a signal to be caught
docker wait mapek_bt_evaluation

# Cleanup after container finishes running normally
docker rm mapek_bt_evaluation
