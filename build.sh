#!/bin/bash

# Exits on error
set -e

# Gets the source directory
source_dir=$(cd $(dirname $0) && pwd)

# Builds the project
build_dir="$source_dir/build"
# build_type: release, debug, releasewithdebinfo
build_type="releasewithdebinfo"

build_tests="OFF"
module_name=""
while getopts ":j:t:m:T" opt; do
  case $opt in
    j)
      num_cores=$OPTARG
      ;;
    t)
      build_type=$OPTARG
      ;;
    m)
      module_name=$OPTARG
      ;;
    T)
      build_tests="ON"
      ;;
    \?)
      echo "Invalid option: -$OPTARG" >&2
      echo "Usage: $0 [-j num_cores] [-t build_type] [-m module_name] [-T]" >&2
      echo "  build_type: release, debug, releasewithdebinfo (default: releasewithdebinfo)" >&2
      echo "  -m: Compile specific module only (e.g., runner_imu, runner_rl_basic)" >&2
      echo "  -T: Enable test compilation (default: disabled)" >&2
      exit 1
      ;;
    :)
      echo "Option -$OPTARG requires an argument." >&2
      exit 1
      ;;
  esac
done

# Validates build type
if [[ "$build_type" != "release" && "$build_type" != "debug" && "$build_type" != "releasewithdebinfo" ]]; then
    echo "Invalid build type: $build_type" >&2
    echo "Valid build types: release, debug, releasewithdebinfo" >&2
    exit 1
fi

shift $((OPTIND - 1))

echo "Using $num_cores cores."
echo "Build type: $build_type"

echo "Building ros2 env..."
./scripts/build_ros2_env.sh
if [ $? -ne 0 ]; then
    echo "Failed to build ros2 env."
    exit 1
fi

echo "Building project..."
mkdir -p $build_dir && cd $build_dir
# Checks if only compiling a specific module
if [ -n "$module_name" ]; then
  # Checks if build directory and CMakeCache exist
  if [ ! -f "$build_dir/CMakeCache.txt" ]; then
    echo "Error: CMakeCache.txt not found in build directory." >&2
    echo "Please run a full build first: ./build.sh" >&2
    exit 1
  fi
  echo "Using existing cmake configuration for module compilation."
else
  # Runs cmake configuration for full build
  source /opt/ros/humble/setup.sh
  cmake -DBUILD_TYPE=$build_type \
        -DBUILD_TESTS=$build_tests \
        -DBUILD_ROS2=ON \
        -DBUILD_DCHECK=ON ..
fi

if [ -z "$num_cores" ]; then
  # Compiles with 2 threads less than the number of cores
  num_cores=$(expr $(nproc) - 2)
  if [ $num_cores -lt 1 ]; then
    num_cores=$(nproc)
  fi
fi

# Compiles the project or specific module
if [ -n "$module_name" ]; then
  # Converts module name to CMake target name
  # Example: runner_imu -> src_runner_imu
  cmake_target=$(echo "$module_name" | sed 's/\//_/g')
  if [[ ! "$cmake_target" == src_* ]]; then
    cmake_target="src_runner_${cmake_target}"
  fi

  # Compiles only the specified target
  make -j$num_cores $cmake_target
  
  # Installs only the specified target
  echo "Installing module: $cmake_target"
  make install
  
  echo "Module compilation completed: $cmake_target"
else
  make -j$num_cores
  make install
  echo "Full build completed."
fi
