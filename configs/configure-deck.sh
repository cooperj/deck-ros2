#!/bin/bash

set -e

DECK_ROS2_DIR=${HOME}/deck-ros2
DECK_ROS2_REPO=https://github.com/cooperj/deck-ros2.git
CONFIGS_DIR=${DECK_ROS2_DIR}/configs
ENV_FILE=${CONFIGS_DIR}/.env

if [ -r ${DECK_ROS2_DIR}/.git ]; then
    echo "Deck ROS2 already exists, updating"
    (cd ${DECK_ROS2_DIR} && git pull)
else
    echo "Cloning Deck ROS2"
    git clone ${DECK_ROS2_REPO} ${DECK_ROS2_DIR}
fi

function add_to_env_file {
    echo "ensuring $1=$2 is in .env"
    if grep -q "^$1=" ${ENV_FILE}; then
        echo "$1 already set."
    else
        echo "Setting $1"
        echo "$1=$2" >> ${ENV_FILE}
    fi
}

echo "configure .env file"
if [ -r ${ENV_FILE} ]; then
    echo ".env already exists."
else
    touch ${ENV_FILE}
fi

add_to_env_file "UID" "`id -u`"
add_to_env_file "GID" "`id -g`"
add_to_env_file "HOSTNAME" "`hostname`"
add_to_env_file "ICON_PATH" "${CONFIGS_DIR}/icons"
add_to_env_file "DECK_ROS2_DIR" "${DECK_ROS2_DIR}"
add_to_env_file "DECK_ROS2_DIR" "${DECK_ROS2_DIR}"

echo "configure Desktop shortcuts"
for file in ${CONFIGS_DIR}/*.desktop.in; do
    dest_file="${HOME}/Desktop/$(basename ${file} .in)"
    echo "Configuring ${dest_file} from ${file}"
    export ICON_PATH=${CONFIGS_DIR}/icons
    export LAUNCH_SCRIPT=${CONFIGS_DIR}/connect-robot.sh
    envsubst < ${file} > ${dest_file}
done