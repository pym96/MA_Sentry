# Package Name

## Overview

This is a package about the referee system, including the sending and receiving of various data and the drawing of the
operation auxiliary interface

**Keywords:** referee, auxiliary, ROS

### License

The source code is released under a [BSD 3-Clause license](LICENSE).

**Author: QiayuanLiao<br />
Affiliation: [Dynamicx]()<br />
Maintainer: QiayuanLiao, liaoqiayuan@gmail.com**

The rm_hw package has been tested under [ROS] Melodic and Noetic on respectively 18.04 and 20.04. This is research code,
expect that it changes often and any fitness for a particular purpose is disclaimed.

[![Build Status](http://rsl-ci.ethz.ch/buildStatus/icon?job=ros_best_practices)](http://rsl-ci.ethz.ch/job/ros_best_practices/)

![Example image](doc/example.jpg)

[comment]: <> (### Publications)

[comment]: <> (If you use this work in an academic context, please cite the following publication&#40;s&#41;:)

[comment]: <> (* P. Fankhauser, M. Bloesch, C. Gehring, M. Hutter, and R. Siegwart: **PAPER TITLE**. IEEE/RSJ International Conference)

[comment]: <> (  on Intelligent Robots and Systems &#40;IROS&#41;, 2015. &#40;[PDF]&#40;http://dx.doi.org/10.3929/ethz-a-010173654&#41;&#41;)

[comment]: <> (        @inproceedings{Fankhauser2015,)

[comment]: <> (            author = {Fankhauser, P\'{e}ter and Hutter, Marco},)

[comment]: <> (            booktitle = {IEEE/RSJ International Conference on Intelligent Robots and Systems &#40;IROS&#41;},)

[comment]: <> (            title = {{PAPER TITLE}},)

[comment]: <> (            publisher = {IEEE},)

[comment]: <> (            year = {2015})

[comment]: <> (        })

## Installation

### Installation from Packages

To install all packages from the this repository as Debian packages use

    sudo apt-get install ros-noetic-...

Or better, use `rosdep`:

	sudo rosdep install --from-paths src

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- roscpp
- sensor_msgs
- roslint
- rm_msgs
- rm_common
- tf2_geometry_msgs
- control_msgs
- serial
- controller_manager_msgs

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package
using

	cd catkin_workspace/src
	git clone http://frp.acmetech.top:8080/dynamicx/rm_manual.git
	cd ../
	rosdep install --from-paths . --ignore-src
	catkin_make

[comment]: <> (### Running in Docker)

[comment]: <> (Docker is a great way to run an application with all dependencies and libraries bundles together. Make sure)

[comment]: <> (to [install Docker]&#40;https://docs.docker.com/get-docker/&#41; first.)

[comment]: <> (First, spin up a simple container:)

[comment]: <> (	docker run -ti --rm --name ros-container ros:noetic bash)

[comment]: <> (This downloads the `ros:noetic` image from the Docker Hub, indicates that it requires an interactive terminal &#40;`-t, -i`&#41;)

[comment]: <> (, gives it a name &#40;`--name`&#41;, removes it after you exit the container &#40;`--rm`&#41; and runs a command &#40;`bash`&#41;.)

[comment]: <> (Now, create a catkin workspace, clone the package, build it, done!)

[comment]: <> (	apt-get update && apt-get install -y git)

[comment]: <> (	mkdir -p /ws/src && cd /ws/src)

[comment]: <> (	git clone http://frp.acmetech.top:8080/dynamicx/rm_manual.git)

[comment]: <> (	cd ..)

[comment]: <> (	rosdep install --from-path src)

[comment]: <> (	catkin_make)

[comment]: <> (	source devel/setup.bash)

[comment]: <> (	roslaunch rm_manual load.launch)

### Unit Tests

Run the unit tests with

	catkin_make text_power_limit.launch

### Static code analysis

Run the static code analysis with

	catkin_make roslint_ros_package_template

## Usage

Describe the quickest way to run this software, for example:

Run the main node with

	roslaunch rm_manual load.launch

## Config files

Config file rm_manual/config

* **engineer.yaml** Shortly explain the content of this config file
