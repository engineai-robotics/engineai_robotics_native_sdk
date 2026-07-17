#!/bin/bash

# Exits on error
set -e

# Gets the source directory
SCRIPT_DIR="$(cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd)"
if [ -d "$SCRIPT_DIR/simulation" ]; then
    ROOT_DIR="$SCRIPT_DIR"
else
    ROOT_DIR="$(dirname "$SCRIPT_DIR")"
fi

echo "[INFO] Exports the environment variables:"
export ENGINEAI_ROBOTICS_DIR="$ROOT_DIR"
echo "[INFO] ENGINEAI_ROBOTICS_DIR=$ENGINEAI_ROBOTICS_DIR"

export ENGINEAI_ROBOTICS_ASSETS="$ENGINEAI_ROBOTICS_DIR/assets"
echo "[INFO] ENGINEAI_ROBOTICS_ASSETS=$ENGINEAI_ROBOTICS_ASSETS"

export ENGINEAI_ROBOTICS_THIRD_PARTY="/opt/engineai_robotics_third_party"
echo "[INFO] ENGINEAI_ROBOTICS_THIRD_PARTY=$ENGINEAI_ROBOTICS_THIRD_PARTY"

export ENGINEAI_ROBOTICS_HARDWARE="/opt/engineai_robotics_hardware"
echo "[INFO] ENGINEAI_ROBOTICS_HARDWARE=$ENGINEAI_ROBOTICS_HARDWARE"

export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$ENGINEAI_ROBOTICS_THIRD_PARTY/lib:$ENGINEAI_ROBOTICS_HARDWARE/lib:$ENGINEAI_ROBOTICS_DIR/core/lib"
echo "[INFO] LD_LIBRARY_PATH=$LD_LIBRARY_PATH"

echo "[INFO] Run the executor:"
# Opens the executor path
mujoco_dir="$ROOT_DIR/simulation/mujoco"
install_dir="$mujoco_dir/build"
cd $install_dir

# Runs executable files
if [ $# -gt 0 ]; then
    ./engineai_robotics_simulation_mujoco "$1"
else
    ./engineai_robotics_simulation_mujoco
fi
