#!/bin/bash

# Exits on error
set -e

# Gets the source directory
source_dir=$(cd $(dirname $0) && pwd)
cross_compile_env_script="${source_dir}/scripts/cross_compile_env.sh"
original_args=("$@")

# build_type: release, debug, releasewithdebinfo
build_type="releasewithdebinfo"
build_tests="OFF"
module_name=""
ros_distro="humble"
arch="x86_64"

usage() {
  echo "Usage: $0 [-j num_cores] [-t build_type] [-m module_name] [-r ros_distro] [-a arch] [-T] [-h]"
  echo "  -j: Number of cores to use for compilation (default: nproc - 2)"
  echo "  -t: build_type: release, debug, releasewithdebinfo (default: releasewithdebinfo)"
  echo "  -m: Compile specific module only (e.g., runner_imu, runner_rl_basic)"
  echo "  -r: ROS distro: humble, jazzy (default: humble)"
  echo "  -a: Architecture: aarch64, x86_64 (default: x86_64)"
  echo "  -T: Enable test compilation (default: disabled)"
  echo "  -h: Show this help message and exit"
}

while getopts ":j:t:m:r:a:Th" opt; do
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
    r)
      ros_distro=$OPTARG
      ;;
    a)
      arch=$OPTARG
      ;;
    T)
      build_tests="ON"
      ;;
    h)
      usage
      exit 0
      ;;
    \?)
      echo "Invalid option: -$OPTARG" >&2
      usage >&2
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

# For aarch64 builds, re-run this script inside the cross-compile container.
if [ "${arch}" = "aarch64" ] && [ "${ENGINEAI_ROBOTICS_IN_CROSS_COMPILE:-0}" != "1" ]; then
    if [ -f "/.dockerenv" ]; then
        echo "[INFO] Detected cross-compile Docker environment in the current shell."
        export ENGINEAI_ROBOTICS_IN_CROSS_COMPILE=1
    else
        if [ ! -x "${cross_compile_env_script}" ]; then
            echo "[ERROR] cross-compile launcher not found: ${cross_compile_env_script}" >&2
            exit 1
        fi

        printf -v nested_build_cmd '%q ' bash "${source_dir}/build.sh" "${original_args[@]}"
        nested_build_cmd="${nested_build_cmd% }"

        echo "[INFO] Running aarch64 build in cross_compile_env (ROS 2: ${ros_distro})..."
        echo "[INFO] Command: ${nested_build_cmd}"
        exec "${cross_compile_env_script}" --ros "${ros_distro}" run \
            "ENGINEAI_ROS2_VERSION=${ros_distro} ENGINEAI_ROBOTICS_IN_CROSS_COMPILE=1 ${nested_build_cmd}"
    fi
fi

# Lookup table: maps "ros_distro:arch" to the lib folder suffix (distro_arch)
declare -A lib_suffix_map=(
  ["humble:x86_64"]="humble_amd64"
  ["humble:aarch64"]="humble_arm64"
  ["jazzy:aarch64"]="jazzy_arm64"
)

# Looks up the lib folder suffix using the combined ros_distro + arch key
lib_suffix="${lib_suffix_map[$ros_distro:$arch]}"
if [ -z "$lib_suffix" ]; then
    echo "Unsupported combination: ros distro '$ros_distro' + arch '$arch'." >&2
    echo "Valid combinations: ${!lib_suffix_map[*]}" >&2
    exit 1
fi

# Selects the matching lib folder and creates the core/lib symlink
lib_target=$(ls -d "$source_dir"/core/lib_*_"${lib_suffix}" 2>/dev/null | head -n 1)
if [ -z "$lib_target" ]; then
    echo "No lib folder found for ros distro '$ros_distro' and arch '$arch' (expected core/lib_*_${lib_suffix})." >&2
    exit 1
fi
lib_target_name=$(basename "$lib_target")
ln -sfn "$lib_target_name" "$source_dir/core/lib"

shift $((OPTIND - 1))

if [ -z "$num_cores" ]; then
  # Compiles with 2 threads less than the number of cores
  num_cores=$(expr $(nproc) - 2)
  if [ $num_cores -lt 1 ]; then
    num_cores=$(nproc)
  fi
fi

echo "Using $num_cores cores."
echo "Build type: $build_type"
echo "ROS distro: $ros_distro"
echo "Architecture: $arch"
echo "Selecting lib folder: $lib_target_name"

build_dir="${source_dir}/build/${arch}"
install_dir="${build_dir}/_install"
mkdir -p "$build_dir" && cd "$build_dir"

echo "Building ros2 env..."
readonly msg_dir="${source_dir}/src/protocol"
readonly ros_setup="/opt/ros/${ros_distro}/setup.bash"
if [ ! -f "${ros_setup}" ]; then
  echo "[ERROR] ROS setup file not found: ${ros_setup}" >&2
  echo "[ERROR] Is ROS '${ros_distro}' installed?" >&2
  exit 1
fi

cd "${msg_dir}"
set +u
# shellcheck disable=SC1090
source "${ros_setup}"
set -u

colcon --log-base "${build_dir}/ros2_env/log" build \
  --base-paths interface_protocol \
  --build-base "${build_dir}/ros2_env/build" \
  --install-base "${build_dir}/ros2_env/install" \
  --test-result-base "${build_dir}/ros2_env/test_results" \
  --merge-install

if [ $? -ne 0 ]; then
    echo "Failed to build ros2 env."
    exit 1
fi

echo "Building project..."
cd "$build_dir"
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
  cmake -DBUILD_TYPE=$build_type \
        -DBUILD_TESTS=$build_tests \
        -DBUILD_ROS2=ON \
        -DBUILD_DCHECK=ON \
        -DROS2_VERSION=${ros_distro} \
        "$source_dir"
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
