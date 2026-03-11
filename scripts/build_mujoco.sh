#!/bin/bash

# Exits on error
set -e

# Gets the source directory
root_dir="$(realpath -s $(cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd)/..)"
mujoco_dir="$root_dir/simulation/mujoco"

# Builds the project
build_dir="$mujoco_dir/build"
mkdir -p $build_dir && cd $build_dir
cmake -DBUILD_RELEASE=ON ..

# Compiles with 2 threads less than the number of cores
num_cores=$(expr $(nproc) - 2)
if [ $num_cores -lt 1 ]; then
  num_cores=$(nproc)
fi
make -j$num_cores

