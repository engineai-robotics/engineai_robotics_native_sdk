#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly REPO_ROOT="$(realpath "${SCRIPT_DIR}/..")"
readonly HUMBLE_IMAGE="ghcr.io/engineai-robotics/cross_compile_env:latest"
readonly JAZZY_IMAGE="ghcr.io/engineai-robotics/cross_compile_env_jazzy:latest"
readonly CONTAINER_HOSTNAME="cross-compile-env"
readonly DEFAULT_ROS2_VERSION="humble"

# ROS 2 distro used to pick the cross-compile image. Can be overridden by the
# environment variable ENGINEAI_ROS2_VERSION or the global option -r/--ros.
ros2_version="${ENGINEAI_ROS2_VERSION:-${DEFAULT_ROS2_VERSION}}"

resolve_image() {
    case "${ros2_version}" in
        humble)
            printf '%s\n' "${HUMBLE_IMAGE}"
            ;;
        jazzy)
            printf '%s\n' "${JAZZY_IMAGE}"
            ;;
    esac
}

validate_ros2_version() {
    case "${ros2_version}" in
        humble|jazzy)
            ;;
        *)
            echo "[ERROR] Unknown ROS 2 version: '${ros2_version}' (allowed: humble, jazzy)." >&2
            exit 1
            ;;
    esac
}

resolve_workdir() {
    if [ -n "${PWD:-}" ] && [ -d "${PWD}" ] && { [ "${PWD}" = "${REPO_ROOT}" ] || [[ "${PWD}" == "${REPO_ROOT}/"* ]]; }; then
        printf '%s\n' "${PWD}"
        return 0
    fi

    printf '%s\n' "${REPO_ROOT}"
}

resolve_git_common_dir() {
    local git_common_dir

    if ! git_common_dir="$(git -C "${REPO_ROOT}" rev-parse --git-common-dir 2>/dev/null)"; then
        return 1
    fi

    if [[ "${git_common_dir}" = /* ]]; then
        printf '%s\n' "${git_common_dir}"
        return 0
    fi

    (
        cd "${REPO_ROOT}"
        realpath -m "${git_common_dir}"
    )
}

run_in_container() {
    local user_name workdir command_string git_common_dir
    user_name="${USER:-$(id -un 2>/dev/null || echo user)}"
    workdir="$(resolve_workdir)"
    command_string="$*"

    local image
    image="$(resolve_image)"

    local docker_args=(
        run
        --rm
        --network
        host
        --hostname
        "${CONTAINER_HOSTNAME}"
        --add-host
        "${CONTAINER_HOSTNAME}:127.0.0.1"
        --workdir
        "${workdir}"
        --env
        "ENGINEAI_ROS2_VERSION=${ros2_version}"
        --env
        "HOST_UID=$(id -u)"
        --env
        "HOST_GID=$(id -g)"
        --env
        "HOST_USER=${user_name}"
        --env
        "HOST_HOME=/tmp/${user_name}"
        --volume
        "${REPO_ROOT}:${REPO_ROOT}"
        --volume
        "/etc/localtime:/etc/localtime:ro"
    )

    if git_common_dir="$(resolve_git_common_dir)"; then
        if [[ "${git_common_dir}" != "${REPO_ROOT}" && "${git_common_dir}" != "${REPO_ROOT}/"* ]]; then
            docker_args+=(
                --volume
                "${git_common_dir}:${git_common_dir}"
            )
        fi
    fi

    if [ "$#" -eq 0 ]; then
        if [ -t 0 ] && [ -t 1 ]; then
            docker_args+=(-it)
        fi

        exec docker "${docker_args[@]}" "${image}" /bin/bash -lc 'exec /bin/bash'
    fi

    exec docker "${docker_args[@]}" "${image}" /bin/bash -lc "${command_string}"
}

print_usage() {
    cat <<EOF
Usage: $(basename "$0") [-r|--ros <humble|jazzy>] [shell|run <command>|pull]

Options:
  -r, --ros <version>  ROS 2 distro selecting the image: humble (default) or jazzy.
                       Can also be set via the ENGINEAI_ROS2_VERSION env var.

Commands:
  shell  Start an interactive shell in the cross-compile container.
  run    Run a command in the cross-compile container.
  pull   Pull the cross-compile base image for the selected ROS 2 distro.

Without arguments, 'shell' is used.
EOF
}

# Consume the global ROS 2 distro option before the subcommand.
while [ "$#" -gt 0 ]; do
    case "$1" in
        -r|--ros)
            if [ "$#" -lt 2 ]; then
                echo "[ERROR] '$1' requires a value (humble|jazzy)." >&2
                exit 1
            fi
            ros2_version="$2"
            shift 2
            ;;
        *)
            break
            ;;
    esac
done

validate_ros2_version

case "${1:-}" in
    ""|shell)
        if [ "$#" -gt 0 ]; then
            shift
        fi
        if [ "$#" -ne 0 ]; then
            echo "[ERROR] 'shell' does not accept extra arguments." >&2
            exit 1
        fi
        run_in_container
        ;;
    run)
        shift
        if [ "$#" -eq 0 ]; then
            echo "[ERROR] 'run' requires a command string." >&2
            exit 1
        fi
        run_in_container "$@"
        ;;
    pull)
        shift
        if [ "$#" -ne 0 ]; then
            echo "[ERROR] 'pull' does not accept extra arguments." >&2
            exit 1
        fi
        docker pull "${HUMBLE_IMAGE}"
        docker pull "${JAZZY_IMAGE}"
        ;;
    -h|--help|help)
        print_usage
        exit 0
        ;;
    *)
        echo "[ERROR] Unknown subcommand: $1" >&2
        print_usage >&2
        exit 1
        ;;
esac
