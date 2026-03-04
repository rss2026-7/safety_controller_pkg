# safety_controller_pkg

ROS 2 safety controller for the MIT RACECAR. Monitors LIDAR and publishes stop commands when obstacles are detected. Use for ALL labs.

## Usage

```bash
# Build
cd ~/racecar_ws && colcon build --packages-select safety_controller_pkg && source install/setup.bash

# Physical car
ros2 run safety_controller_pkg safety_controller_pkg
```
