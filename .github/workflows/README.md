# GitHub Actions CI for Boeing Navigation

This workflow builds the Docker image and runs tests without requiring any USB hardware.

## What it does

1. **Builds the Docker image** using BuildKit with GitHub Actions cache
2. **Tests ROS 2 workspace** - verifies packages are built and sourced
3. **Verifies launch files** - checks all launch files parse correctly with `--show-args`
4. **Verifies config files** - ensures all YAML configs are installed
5. **Verifies entrypoint** - basic ROS 2 command works

## Running locally

```bash
# Build
docker build -t boeing-navigation:humble .

# Test (no hardware needed)
docker run --rm boeing-navigation:humble \
  /bin/bash -c "source /opt/ros/humble/setup.bash && source /ros_ws/install/setup.bash && ros2 pkg list | grep boeing"

# Verify launch files
docker run --rm boeing-navigation:humble \
  /bin/bash -c "source /opt/ros/humble/setup.bash && source /ros_ws/install/setup.bash && \
  ros2 launch boeing_bringup bringup.launch.py --show-args"
```

## Hardware access (local development)

For actual robot testing, use `compose.yaml` which mounts `/dev` and runs privileged:

```bash
docker compose up -d
docker exec -it boeing_ros bash
```

The CI workflow intentionally skips hardware mounts - it only validates the build and ROS 2 workspace integrity.