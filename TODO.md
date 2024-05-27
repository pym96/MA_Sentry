```
#!/bin/bash
# Script to automatically start rosbag recording

# Define the directory where the rosbag files will be saved
DIRECTORY="/path/to/your/directory"

# Ensure the directory exists
mkdir -p $DIRECTORY

# Define the filename with timestamp
FILENAME="record_$(date +%Y-%m-%d-%H-%M-%S).bag"

# Start recording all topics
rosbag record -a -O $DIRECTORY/$FILENAME
```
