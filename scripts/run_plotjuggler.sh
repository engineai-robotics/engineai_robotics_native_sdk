#!/bin/bash

for arg in "$@"; do
    if [ "$arg" == "remote" ]; then
        export ROS_DOMAIN_ID=69
        export ROS_LOCALHOST_ONLY=0
        echo "Starting PlotJuggler in remote mode"
        break
    fi
done

ROOT_DIR="$(realpath -s $(cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd)/..)"
ROS_ENV_DIR="$ROOT_DIR/build/ros2_env/install"

cd $ROOT_DIR
if [ ! -d "$ROS_ENV_DIR" ]; then
    echo "Error: ROS environment directory not found: $ROS_ENV_DIR, please run ./build.sh first"
    exit 1
fi

source $ROS_ENV_DIR/local_setup.bash
ros2 run plotjuggler plotjuggler -n
