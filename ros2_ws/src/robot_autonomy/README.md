# robot_autonomy

This ROS 2 package provides a fake LaserScan publisher for validating the TF chain
and RViz2 visualization while the real sensor is unavailable.

## Usage

```bash
cd ros2_ws
colcon build --packages-select robot_autonomy
source install/setup.bash
ros2 launch robot_autonomy fake_laser_scan.launch.py
```

## SLAM with fake scan

If you have `slam_toolbox` installed, you can run mapping using the fake scan:

```bash
ros2 launch robot_autonomy slam_fake_laser.launch.py
```

## Switching to real hardware

When the RPLidar is available, replace the `fake_laser_scan_publisher` node in
`launch/fake_laser_scan.launch.py` with the `rplidar_ros` driver node and keep the
static transform (or replace it with the one provided by the driver).
