# BoeingNavigation

## udev rules (Kobuki + RPLIDAR)

The rules in `rules.d/` create fixed device names (`/dev/kobuki`, `/dev/rplidar`).
They must be installed on the **host** (the robot's computer), not inside the container.

### Install

```bash
sudo cp rules.d/*.rules /etc/udev/rules.d/
```

### Apply without rebooting

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Then unplug and replug the Kobuki and the lidar if the symlinks don't appear.

### Verify

```bash
ls -l /dev/kobuki /dev/rplidar
```
