#!/bin/bash

# File to store previous IPs
IP_FILE="$HOME/deck-ros2/robots.txt"

touch "$IP_FILE"

# Read previous IPs into an array
IP_LIST=$(cat "$IP_FILE")

# Construct options for Zenity list
OPTIONS="NewRobot"
while IFS= read -r ip; do
    OPTIONS+=" $ip"
done < "$IP_FILE"

# Show menu with Zenity
CHOICE=$(zenity --list --title="Connect to Robot" \
    --column="Select or enter a new IP" --width=400 --height=300 \
    $OPTIONS 2>/dev/null)

if [[ -z "$CHOICE" ]]; then
    echo "No selection, exiting..."
    exit 1
fi

if [[ "$CHOICE" == "NewRobot" ]]; then
    IP=$(zenity --entry --title="Enter IP" --text="Enter the IP address:" 2>/dev/null)
else
    IP="$CHOICE"
fi

if [[ -z "$IP" ]]; then
    echo "No IP entered, exiting..."
    exit 1
fi

# Save IP if not already in history
grep -qxF "$IP" "$IP_FILE" || echo "$IP" >> "$IP_FILE"

# -- Connect to robot and launch teleop and RViz --
echo "Connecting to $IP"

echo "Enabling x11 access from podman container"
xhost +local:docker

echo "Launching Deck ROS2 container"
echo "IP: $IP"

podman run -it --rm --name deck-ros2 --hostname "$(hostnamectl hostname)" --pull=always -e DISPLAY=$DISPLAY -e ROBOT_IP=$IP --user ros -v /tmp/.X11-unix:/tmp/.X11-unix -v $HOME/deck-ros2/:/home/ros/ws --net=host --privileged -v /dev/input:/dev/input --annotation run.oci.keep_original_groups=1 --userns=keep-id ghcr.io/cooperj/deck-ros2:humble bash -c "\$HOME/.local/bin/tmule --config \$HOME/ws/tmule/default.tmule.yaml launch; tmux a"
