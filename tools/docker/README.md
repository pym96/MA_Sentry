# Docker configuration

Use the build and run commands in the [project README](../../README.md).

- `Dockerfile.noetic` installs Ubuntu 20.04 / ROS Noetic dependencies and Livox SDK.
- `compile.sh` copies the source into `/ws/src/MA_Sentry`, selects the ROS 1 Livox
  manifest, and builds the catkin workspace.
- Mount the repository at `/source:ro` to keep the host checkout read-only.
- Set `BUILD_JOBS` to a positive integer to control parallel compilation; the
  default is `2`.
- The example allocates up to 4 CPUs and 8 GiB of memory. Adjust `--cpus` and
  `--memory` for your machine.
- On Apple Silicon, retain `--platform linux/amd64` in both commands.

The first image build downloads dependencies. Subsequent runs reuse the local
image. With `--rm`, the container and its build outputs are removed on exit.
