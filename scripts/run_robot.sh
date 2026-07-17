#!/bin/bash

# Exits on error
set -e

RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to check if motor_service_server process exists
check_motor_service() {
    # Use pgrep to find processes matching motor_service_server
    if pgrep -f motor_service_server > /dev/null; then
        echo -e "${RED}⚠️ WARNING: ⚠️${NC}"
        echo -e "${YELLOW}The motor drive server process is currently running.${NC}"
        echo -e "${YELLOW}Please IMMEDIATELY close the Robot Management Service webpage${NC}"
        exit 1
    fi
}

check_engine_robotics_process() {
    if pgrep -f src_executor > /dev/null; then
        echo -e "${RED}⚠️ WARNING: ⚠️${NC}"
        echo -e "${YELLOW}The src_executor process is currently running.${NC}"
        echo -e ${YELLOW}PID: $(pgrep -f src_executor)${NC}
        exit 1
    fi
}

# Function to check router connectivity
check_router_connectivity() {
    # Check if 192.168.0.1 is reachable, wait up to 2 minutes
    ROUTER_IP="192.168.0.1"
    MAX_WAIT_SECONDS=60
    ATTEMPT=1
    CONNECTED=false

    echo -e "$(date '+%Y-%m-%d %H:%M:%S') [INFO] Starting router $ROUTER_IP connectivity check..."

    while [ $ATTEMPT -le $MAX_WAIT_SECONDS ] && [ "$CONNECTED" = false ]; do
        if ping -c 1 -W 1 $ROUTER_IP > /dev/null 2>&1; then
            CONNECTED=true
            echo -e "$(date '+%Y-%m-%d %H:%M:%S') [SUCCESS] Router $ROUTER_IP connection successful!"
        else
            if [ $((ATTEMPT % 10)) -eq 0 ]; then
                echo -e "$(date '+%Y-%m-%d %H:%M:%S') [INFO] Waiting for router $ROUTER_IP connection... (waited ${ATTEMPT} seconds)"
            fi
            ATTEMPT=$((ATTEMPT+1))
            sleep 1
        fi
    done

    if [ "$CONNECTED" = false ]; then
        echo -e "$(date '+%Y-%m-%d %H:%M:%S') [WARNING] Still unable to connect to router $ROUTER_IP after waiting ${MAX_WAIT_SECONDS} seconds, continuing with subsequent processes..."
    else
        echo -e "$(date '+%Y-%m-%d %H:%M:%S') [INFO] Router connectivity check completed, continuing to start processes..."
    fi
}

setup_hardware_env() {
    sudo chmod 666 /dev/ttyACM0
}

setup_ros() {    
    source /opt/ros/humble/setup.bash
    export ROS_DOMAIN_ID=69
}


# Run the check function
check_engine_robotics_process

# Set default runtime environment if not specified
if [ -z "$ENGINEAI_ROBOTICS_RUNTIME_ENV" ]; then
    export ENGINEAI_ROBOTICS_RUNTIME_ENV="robot"
fi
# Skip motor service check and hardware setup if running in simulation mode
if [ "$ENGINEAI_ROBOTICS_RUNTIME_ENV" != "sim" ]; then
    check_motor_service
    setup_hardware_env
    # Check router connectivity only if ENGINEAI_ROBOTICS_COPYRIGHT is set
    if [ -n "$ENGINEAI_ROBOTICS_COPYRIGHT" ]; then
        check_router_connectivity
    fi
fi

setup_ros

# Gets the source directory
source_dir=$(cd $(dirname $0) && pwd)

echo "[INFO] Exports the environment variables:"
export ENGINEAI_ROBOTICS_DIR="$PWD"
echo "[INFO] ENGINEAI_ROBOTICS_DIR=$ENGINEAI_ROBOTICS_DIR"

export ENGINEAI_ROBOTICS_ASSETS="$ENGINEAI_ROBOTICS_DIR/assets"
echo "[INFO] ENGINEAI_ROBOTICS_ASSETS=$ENGINEAI_ROBOTICS_ASSETS"

export ENGINEAI_ROBOTICS_CONFIG="$ENGINEAI_ROBOTICS_DIR/assets/config"
echo "[INFO] ENGINEAI_ROBOTICS_CONFIG=$ENGINEAI_ROBOTICS_ASSETS"

export ENGINEAI_ROBOTICS_THIRD_PARTY="/opt/engineai_robotics_third_party"
echo "[INFO] ENGINEAI_ROBOTICS_THIRD_PARTY=$ENGINEAI_ROBOTICS_THIRD_PARTY"

export ENGINEAI_ROBOTICS_HARDWARE="/opt/engineai_robotics_hardware"
echo "[INFO] ENGINEAI_ROBOTICS_HARDWARE=$ENGINEAI_ROBOTICS_HARDWARE"

export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$ENGINEAI_ROBOTICS_THIRD_PARTY/lib:$ENGINEAI_ROBOTICS_THIRD_PARTY/lib/runtime:$ENGINEAI_ROBOTICS_HARDWARE/lib:$ENGINEAI_ROBOTICS_DIR/_install/lib"
echo "[INFO] LD_LIBRARY_PATH=$LD_LIBRARY_PATH"

echo "[INFO] Run the executor:"
# Opens the executor path
install_dir="$source_dir/_install"
cd $install_dir/bin

# Runs executable files
if [ $# -gt 0 ]; then
    ./src_executor "$1"
else
    ./src_executor
fi