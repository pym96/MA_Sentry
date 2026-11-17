# MA_Sentry

https://github.com/pym96/MA_Sentry/assets/105438207/54a1b719-e381-4962-ab62-0fa8ce5e9e83

![image](https://github.com/pym96/MA_Sentry/assets/105438207/6373742c-4ca8-4d8a-8e34-64be34388b41)

## prerequisite
```
git clone https://github.com/pym96/Livox-SDK2.git
cmake -S Livox-SDK2 -B Livox-SDK2/build -DCMAKE_BUILD_TYPE=Release
cmake --build Livox-SDK2/build -j2
sudo cmake --install Livox-SDK2/build
sudo ldconfig
```

**and invert your lidar**.

## How to clone
```
git clone https://github.com/pym96/MA_Sentry.git
```

## config
```
sudo apt update
sudo apt install libusb-dev
sudo apt install ros-noetic-serial
sudo apt install ros-noetic-tf2-sensor-msgs
sudo apt install ros-noetic-velodyne-gazebo-plugins
sudo apt-get install libgoogle-glog-dev
sudo apt install ros-noetic-behaviortree-cpp-v3
sudo apt install xterm
```

Then get into vehicile_simulator file folder
```
./vehicle_simulator/mesh/download_environments.sh
```

## Native catkin workspace

Use Ubuntu 20.04 / ROS Noetic with the build dependencies listed in
`tools/docker/Dockerfile.noetic` and the Livox SDK installed above. Place this
repository at `~/catkin_ws/src/MA_Sentry`, then run:

```bash
source /opt/ros/noetic/setup.bash
cd ~/catkin_ws/src/MA_Sentry
cp rm_sensors_driver/livox_ros_driver2/package_ROS1.xml rm_sensors_driver/livox_ros_driver2/package.xml
cd ~/catkin_ws
catkin_make -j2 -l2 -DCMAKE_BUILD_TYPE=Release
```

Copying `package_ROS1.xml` to `package.xml` selects the ROS 1 Livox driver.

## Build with Docker (macOS / Linux)

Install and start Docker Desktop on macOS, or Docker Engine on Linux.
On Apple Silicon, use `linux/amd64` to build in an x86-64 Ubuntu 20.04 / ROS Noetic
environment through emulation.

Run from the repository root:

```bash
docker build --platform linux/amd64 -f tools/docker/Dockerfile.noetic -t ma-sentry:noetic tools/docker
docker run --rm --platform linux/amd64 --cpus 4 --memory 8g \
  -e BUILD_JOBS=2 -v "$PWD:/source:ro" \
  ma-sentry:noetic bash /source/tools/docker/compile.sh
```

The first image build downloads ROS/PCL/Gazebo dependencies and builds the pinned
Livox SDK revision. Source is copied into `/ws/src/MA_Sentry` inside the container;
build outputs do not modify the host checkout. Save the command output if needed:
the `--rm` container and its build outputs are removed when it exits.

See [Docker configuration](tools/docker/README.md) for resource settings.
