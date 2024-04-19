#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" &>/dev/null && pwd)"
PROJECT_DIR="$(dirname "${SCRIPT_DIR}")"
TAG="fp-ros2"
DOCKERFILE="${SCRIPT_DIR}/dockerfile-${1}"

DOCKER_BUILD_CMD=(docker build "${SCRIPT_DIR}" --tag ${TAG} -f ${DOCKERFILE})

echo -e "\033[0;32m${DOCKER_BUILD_CMD[*]}\033[0m" | xargs

# shellcheck disable=SC2068
exec ${DOCKER_BUILD_CMD[*]}