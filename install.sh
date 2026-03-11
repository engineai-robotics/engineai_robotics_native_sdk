#!/bin/bash

# Usage: ./install.sh <product_name> <mode>
# Example: ./install.sh pm_v2 robot
# Exits on error
set -e

# !!! Sets the remote user and host
remote_user="user"
remote_host="192.168.0.163"
remote_dir="~/projects/engineai_robotics"

# Sets the remote destination
remote_dest="${remote_user}@${remote_host}"

# Gets the source directory
source_dir=$(cd $(dirname $0) && pwd)
local_install_dir="$source_dir/build/_install"
local_scripts=("$source_dir/scripts/run_robot.sh" "$source_dir/scripts/env_robot.sh" "$source_dir/scripts/set_imu_tty.sh")
local_assets_dir="$source_dir/assets"

# Requires product_name and mode; prints red error and exits if missing
if [ $# -lt 2 ] || [ -z "$1" ] || [ -z "$2" ]; then
    echo -e "\033[31mERROR: product name and mode are required.\033[0m"
    echo -e "\033[31mExample: ./install.sh pm01_edu robot\033[0m"
    exit 1
fi
product_name="$1"
active_mode="$2"
echo "Installing for product: $product_name with mode: $active_mode"

temp_dir=$(mktemp -d)
# Reuses a single SSH connection so password is only prompted once
ssh_control_path="${temp_dir}/ssh_%r_%h_%p"
trap "ssh -o ControlPath=${ssh_control_path} -O exit ${remote_dest} 2>/dev/null || true; rm -rf $temp_dir" EXIT

echo "temp_dir: ${temp_dir}"
rsync -av "${local_install_dir}" "${temp_dir}"
rsync -av "${local_scripts[@]}" "${temp_dir}"

# Copies assets for the given product
if [ -d "${local_assets_dir}/config/$product_name" ]; then
    echo "Copying assets for $product_name..."
    mkdir -p "${temp_dir}/assets/config"
    rsync -av "${local_assets_dir}/config/$product_name" "${temp_dir}/assets/config/"

    # urdf
    mkdir -p "${temp_dir}/assets/resource/robot/$product_name"
    rsync -av "${local_assets_dir}/resource/robot/$product_name/urdf" "${temp_dir}/assets/resource/robot/$product_name"
else
    echo "Error: Product directory ${local_assets_dir}/config/$product_name does not exist"
    exit 1
fi

# Add git information if available (兼容普通仓库和 worktree)
git_dir=$( [ -d "$source_dir/.git" ] && echo "$source_dir/.git" || ( [ -f "$source_dir/.git" ] && sed 's/gitdir: //' "$source_dir/.git" ) )
if [ -n "$git_dir" ]; then
    echo "release_note.txt" > "${temp_dir}/release_note.txt"
    git_commit_id=$(git rev-parse HEAD)
    git_branch=$(git rev-parse --abbrev-ref HEAD)
    git_date=$(date "+%Y-%m-%d %H:%M:%S")
    echo "commit_id: ${git_commit_id}" >> "${temp_dir}/release_note.txt"
    echo "branch: ${git_branch}" >> "${temp_dir}/release_note.txt"
    echo "date: ${git_date}" >> "${temp_dir}/release_note.txt"
fi

# Opens a single SSH connection (password prompted once); mkdir and rsync reuse it
echo "Connecting to ${remote_dest} (enter password once)..."
ssh -o ControlMaster=yes -o ControlPath="${ssh_control_path}" -o ControlPersist=60 "${remote_dest}" "echo 'Connected.'"

# Ensures the remote directory exists (reuses connection, no password)
ssh -o ControlPath="${ssh_control_path}" "${remote_dest}" "mkdir -p $remote_dir"

rsync -avz --delete -e "ssh -o ControlPath=${ssh_control_path}" "${temp_dir}/" "${remote_dest}:$remote_dir"

# Updates active_mode in remote mode.yaml (same logic as in scripts/package.sh)
remote_mode_yaml="${remote_dir}/assets/config/${product_name}/mode.yaml"
ssh -o ControlPath="${ssh_control_path}" "${remote_dest}" "sed -i 's/^active_mode:.*$/active_mode: $active_mode/' $remote_mode_yaml"
echo "Updated active_mode to '$active_mode' in remote $remote_mode_yaml"

echo "Congratulations! Installed to ${remote_user}@${remote_host}:${remote_dir}"
