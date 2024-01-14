#!/bin/bash

# Prompt for the user's password
read -sp "Please enter your password: " password

# Use sudo to change permissions on /dev/tty* devices
echo "$password" | sudo -S chmod 777 /dev/tty*

# Check if the command was successful
if [ $? -eq 0 ]; then
  echo "Permissions changed successfully."
else
  echo "Failed to change permissions. Please check your password."
fi
