```
#! /bin/bash

 source  /opt/ros/noetic/setup.bash


DIRECTORY="/home/dan/demo"

# Ensure the directory exists
mkdir -p $DIRECTORY

FILENAME="record_$(date +%Y-%m-%d-%H-%M-%S).bag"

# Start recording all topics
rosbag record -a -O $DIRECTORY/$FILENAME




```
