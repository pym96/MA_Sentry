#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/noetic/setup.bash
set -u
: "${BUILD_JOBS:=2}"
if [[ ! "$BUILD_JOBS" =~ ^[1-9][0-9]*$ ]]; then
  echo "BUILD_JOBS must be a positive integer" >&2
  exit 2
fi
if [[ ! -f /source/README.md ]]; then
  echo "Mount the repository read-only at /source" >&2
  exit 2
fi
mkdir -p /ws/src/MA_Sentry
# Build a private copy: catkin must not write into the host checkout.
tar -C /source --exclude=.git --exclude=build --exclude=devel --exclude=install -cf - . \
  | tar -C /ws/src/MA_Sentry -xf -
# The upstream driver keeps separate ROS 1 and ROS 2 manifests.
driver=/ws/src/MA_Sentry/rm_sensors_driver/livox_ros_driver2
cp "$driver/package_ROS1.xml" "$driver/package.xml"
cd /ws
catkin_make -j"$BUILD_JOBS" -l"$BUILD_JOBS" -DCMAKE_BUILD_TYPE=Release

# Catch a silently skipped driver and missing representative runtime artifacts.
for target in \
  livox_ros_driver2/livox_ros_driver2_node \
  faster_lio/run_mapping_online \
  local_planner/pathFollower \
  rm_decision/rm_decision_final_node \
  ma_serial_port/ma_serial_port_node \
  vehicle_simulator/vehicleSimulator; do
  test -x "/ws/devel/lib/$target" || {
    echo "Missing build artifact: $target" >&2
    exit 1
  }
done
echo "Compile check passed; hardware and runtime behavior were not tested."
