#!/bin/bash

# Exits on error
set -e

# Gets the source directory
root_dir="$(realpath -s $(cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd)/..)"
msg_dir="$root_dir/src/protocol"

cd $msg_dir
source /opt/ros/humble/setup.bash
colcon --log-base "$root_dir/build/ros2_env/log" build \
  --base-paths interface_protocol \
  --build-base "$root_dir/build/ros2_env/build" \
  --install-base "$root_dir/build/ros2_env/install" \
  --test-result-base "$root_dir/build/ros2_env/test_results" \
  --merge-install
