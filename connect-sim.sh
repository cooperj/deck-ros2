#!/bin/bash
podman run -it -u 0 --rm --name deck-ros2 --hostname "$(hostnamectl hostname)" --pull=always -e DISPLAY=$DISPLAY -e ROBOT_IP=$IP --user ros -v /tmp/.X11-unix:/tmp/.X11-unix -v $HOME/deck-ros2/:/home/ros/ws -v /dev:/dev --privileged --cap-add=SYS_ADMIN ghcr.io/cooperj/deck-ros2:humble bash -c "\$HOME/.local/bin/tmule --config \$HOME/ws/tmule/sim.tmule.yaml launch; tmux a"
