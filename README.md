# MA_Sentry

https://github.com/pym96/MA_Sentry/assets/105438207/54a1b719-e381-4962-ab62-0fa8ce5e9e83

![image](https://github.com/pym96/MA_Sentry/assets/105438207/6373742c-4ca8-4d8a-8e34-64be34388b41)

## prerequisite
```
git clone https://github.com/pym96/Livox-SDK2.git
cd <file-folder-above>
mkdir build
cd build
make
sudo make install
```

**and invert your lidar**.

## How to clone
```
git clone -b final https://github.com/pym96/MA_Sentry.git
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

## Bugs
If you met the bug below:
![image](https://github.com/pym96/MA_Sentry/assets/105438207/5c0098e9-fdc7-4256-9d44-c7745650a884)

just compile it again, then all things will be ok(normally......)

## Attention
Branch main is chaos, BUT branch final is normal.
