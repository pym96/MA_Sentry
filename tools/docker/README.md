# ROS Noetic compile check

- `Dockerfile.noetic`: Ubuntu 20.04 / ROS Noetic dependencies and pinned Livox SDK.
- `compile.sh`: copy source into an isolated workspace, select the ROS 1 Livox
  manifest, compile with catkin, and check representative executable artifacts.

Run the commands in the repository's main README. This directory is the Docker
build context; source is supplied at container runtime through `/source:ro`.
The container needs neither privileged mode nor access to the Docker socket.

## Validation scope

This is a compilation check, not a ROS integration or hardware test. It does not
launch the robot, connect sensors, download simulation environments, or establish
that planning and control behavior is correct. ROS Noetic / Ubuntu 20.04 are a
legacy compatibility environment, not a migration to a supported ROS release.

Known source issues remain outside this build-only change:

- `rm_control/ma_serial_port/src/ma_serial.cc` uses `while(ros::ok)` rather than
  calling `ros::ok()`.
- `local_planner/launch/local_planner.launch` spells two parameters
  `autonomyMpode` and `autonomySeed`, unlike the names read by the node.
- Decision nodes load behavior-tree XML using author-specific absolute paths.
- `ma_sentry_bringup/src/cloud_filter.cc` produces printf-format warnings;
  a successful compile does not establish correct behavior on that error path.

These should be resolved and tested before claiming a working robot deployment.

## Recorded build: 2026-11-17

Executed by the coding agent on an Apple Silicon Mac with Docker Desktop,
`linux/amd64` emulation, 4 CPUs / 8 GiB container limits and two build jobs.
Application source baseline: `36a389b16af9165958385bf8bdd631f0ae0a967d`;
this change only adds build tooling and documentation.

- The exact image-build command in the root README exited 0.
- The exact compile command ran in a fresh container with no catkin cache and
  exited 0. Catkin traversed 23 packages, including `livox_ros_driver2`.
- All six executable checks in `compile.sh` passed.
- `BUILD_JOBS=0` was rejected with exit 2.
- GCC 9.4.0; CMake 3.16.3; Release build.
- The joy attribute warning and cloud-filter format warnings remain.

Local image ID:
`sha256:3d54054e3fda49da59ff9d8b2801e53f4fa2b1239fe9436fd117888e0dbed713`.
Base image and SDK revisions are pinned in the Dockerfile; apt package versions
are not fully locked, so future builds are not guaranteed to be byte-identical.

Raw local log SHA-256 values (retained by the contributor, not published here):

| Log | SHA-256 |
| --- | --- |
| `docker-build.log` | `34093b5235f87bd9d500d1294fa558fdbeccee12f016134c24e64c1d0e89d0f7` |
| `clean-compile.log` | `296c9e4904b52c05bebc2b94e84368851730859ecde6ff6ae8eac0b67ae15326` |

This is executable build evidence, not an independent code review or runtime
acceptance of the existing robot software.
