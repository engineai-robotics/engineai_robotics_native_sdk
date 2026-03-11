#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# --- Get user info ---
USER_NAME="$(id -un)"
USER_ID="$(id -u)"
GROUP_ID="$(id -g)"
USER_HOME="$HOME"

# --- Config constants ---
NVIDIA_GPU_AVAILABLE="n"
WORKSPACE_DIR="${SCRIPT_DIR}/../"
PROJECT_NAME="engineai_robotics_env"
SOURCE_IMAGE="ghcr.io/engineai-robotics/engineai_robotics_env:latest"
LOCAL_IMAGE_TAG="humble-desktop-local"

# --- Derive variables ---
CONTAINER_NAME="${PROJECT_NAME}"
PROJECT_DIR="${SCRIPT_DIR}/dockers/${PROJECT_NAME}"
DOCKERFILES_DIR="${SCRIPT_DIR}/dockerfiles"
mkdir -p "${PROJECT_DIR}"

# --- Generate .env file for docker compose ---
cat > "${PROJECT_DIR}/.env" <<EOF
SOURCE_IMAGE=${SOURCE_IMAGE}
USER_NAME=${USER_NAME}
USER_ID=${USER_ID}
GROUP_ID=${GROUP_ID}
USER_HOME=${USER_HOME}
PROJECT_NAME=${PROJECT_NAME}
CONTAINER_NAME=${CONTAINER_NAME}
LOCAL_IMAGE_NAME=${PROJECT_NAME}
LOCAL_IMAGE_TAG=${LOCAL_IMAGE_TAG}
WORKSPACE_DIR=${WORKSPACE_DIR}
HOSTNAME=${HOSTNAME}
EOF

# --- Copy compose files and Dockerfile to project dir ---
cp "${DOCKERFILES_DIR}/docker-compose.yml" "${PROJECT_DIR}/"
cp "${DOCKERFILES_DIR}/Dockerfile" "${PROJECT_DIR}/"
cp "${DOCKERFILES_DIR}/entrypoint.sh" "${PROJECT_DIR}/"
cp "${DOCKERFILES_DIR}/.bashrc" "${PROJECT_DIR}/"

COMPOSE_FILES="-f docker-compose.yml"
if [ "$NVIDIA_GPU_AVAILABLE" = "y" ]; then
  cp "${DOCKERFILES_DIR}/docker-compose.gpu.yml" "${PROJECT_DIR}/"
  COMPOSE_FILES="${COMPOSE_FILES} -f docker-compose.gpu.yml"
fi

# --- Build image ---
echo "Now building the image..."
echo "Starting to pull ${SOURCE_IMAGE} from remote..."
if docker pull "${SOURCE_IMAGE}"; then
  echo "Successfully pulled ${SOURCE_IMAGE} from remote."
elif docker image inspect "${SOURCE_IMAGE}" > /dev/null 2>&1; then
  echo "Remote unreachable, using local image ${SOURCE_IMAGE}."
else
  echo "Error: failed to pull ${SOURCE_IMAGE} and no local image found." >&2
  exit 1
fi
cd "${PROJECT_DIR}" || exit 1
docker compose ${COMPOSE_FILES} build --no-cache

# --- Install dev-env script ---
INSTALL_PATH="${USER_HOME}/.local/bin"
DEV_ENV_SCRIPT="${INSTALL_PATH}/${PROJECT_NAME}"
mkdir -p "${INSTALL_PATH}"

cat > "${DEV_ENV_SCRIPT}" <<EOF
#!/usr/bin/env bash
PROJECT_DIR="${PROJECT_DIR}"
CONTAINER_NAME="${CONTAINER_NAME}"
COMPOSE_FILES="${COMPOSE_FILES}"
ENV_FILE="${PROJECT_DIR}/.env"
SELF_PATH="${DEV_ENV_SCRIPT}"
EOF
cat "${DOCKERFILES_DIR}/env.sh" >> "${DEV_ENV_SCRIPT}"
chmod +x "${DEV_ENV_SCRIPT}"

echo "Installed ${PROJECT_NAME} to ${INSTALL_PATH}/"

if ! echo "$PATH" | grep -qF -- "$INSTALL_PATH"; then
  echo "export PATH=$INSTALL_PATH:\$PATH" >> ~/.bashrc
  export PATH=$INSTALL_PATH:$PATH
fi
