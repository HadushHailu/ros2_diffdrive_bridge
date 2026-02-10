# diffdrive_serial_bridge

ROS2 Python bridge that converts `/cmd_vel` into serial `M,<L>,<R>\n` motor commands for Arduino.

## Usage

Build:
```bash
cd ~/dev/ros2_diffdrive_bridge/ros2_ws
colcon build --symlink-install
source install/setup.bash

