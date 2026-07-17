#!/usr/bin/env bash
# --- 以上变量由 generate.sh 写入，以下为固定逻辑 ---

cd "${PROJECT_DIR}" || exit 1
COMPOSE_CMD="docker compose ${COMPOSE_FILES}"

prepare_x11_auth() {
    if [ -z "${DISPLAY:-}" ]; then
        echo "DISPLAY is not set; GUI applications will not be able to open windows."
        return
    fi

    export DOCKER_XAUTHORITY="${PROJECT_DIR}/.docker.xauth"
    touch "${DOCKER_XAUTHORITY}"
    chmod 600 "${DOCKER_XAUTHORITY}"

    if command -v xauth >/dev/null 2>&1; then
        if ! xauth -i nlist "${DISPLAY}" | sed -e 's/^..../ffff/' | xauth -f "${DOCKER_XAUTHORITY}" nmerge - >/dev/null 2>&1; then
            echo "Warning: failed to copy X11 auth for DISPLAY=${DISPLAY}."
        fi
    else
        echo "Warning: xauth is not installed on the host; X11 authorization may fail."
    fi
}

if [ "${1:-}" == "clean" ]; then
    ${COMPOSE_CMD} down
    echo "Container cleaned up."

    if [ -d "${PROJECT_DIR}" ]; then
        rm -rf "${PROJECT_DIR}"
        echo "Workspace cleaned up."
    fi

    if [ -f "${SELF_PATH}" ]; then
        rm -rf "${SELF_PATH}"
        echo "env script removed."
    fi
    exit 0
fi

prepare_x11_auth

if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    CONTAINER_STATUS=$(docker inspect -f '{{.State.Status}}' "${CONTAINER_NAME}")
    case ${CONTAINER_STATUS} in
        "exited")
            echo "Container exists but stopped, starting it..."
            docker start "${CONTAINER_NAME}"
            ;;
        "running")
            echo "Container is already running"
            ;;
        *)
            echo "Container exists but in state: ${CONTAINER_STATUS}"
            echo "Attempting to start..."
            docker start "${CONTAINER_NAME}" || {
                echo "Failed to start, trying docker compose up -d..."
                ${COMPOSE_CMD} up -d
            }
            ;;
    esac
else
    echo "Container does not exist, creating with docker compose..."
    ${COMPOSE_CMD} up -d
fi

CID=$(${COMPOSE_CMD} ps -q)
user=$USER

if [ "$#" == "0" ]; then
    docker exec -it --user "$user" "$CID" bash -c "source /etc/profile; export XAUTHORITY=/tmp/.docker.xauth; export QT_X11_NO_MITSHM=1; bash"
else
    if [ "$1" == "bash" ]; then
        docker exec -it --user "$user" "$CID" bash -c "source /etc/profile; export XAUTHORITY=/tmp/.docker.xauth; export QT_X11_NO_MITSHM=1; $* "
    else
        docker exec --user "$user" "$CID" bash -c "source /etc/profile; export XAUTHORITY=/tmp/.docker.xauth; export QT_X11_NO_MITSHM=1; $* &"
    fi
fi
