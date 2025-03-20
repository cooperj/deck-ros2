#!/bin/bash

# File to store previous IPs
IP_FILE="$HOME/deck-ros2/robots.txt"

touch "$IP_FILE"

while true; do
    # Read previous IPs into an array
    IP_LIST=$(cat "$IP_FILE")
    
    # Construct options for Zenity list
    OPTIONS="Enter a new IP"
    while IFS= read -r ip; do
        OPTIONS+="|$ip"
    done < "$IP_FILE"
    
    # Show menu with Zenity
    CHOICE=$(zenity --list --title="Connect to Robot" \
        --column="Select or enter a new IP" --width=400 --height=300 \
        $OPTIONS 2>/dev/null)
    
    if [[ -z "$CHOICE" ]]; then
        echo "No selection, exiting..."
        exit 1
    fi
    
    if [[ "$CHOICE" == "Enter a new IP" ]]; then
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
    
    # Open in Firefox
    firefox "http://$IP" &

done
