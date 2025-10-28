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

echo "NVIDIA drivers are running successfully."
