# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build & Run

```bash
# Build (from racecar_ws)
cd ~/racecar_ws && colcon build --packages-select safety_controller_pkg && source install/setup.bash

# Run safety node (physical car or with racecar_simulator)
ros2 run safety_controller_pkg safety_controller_pkg

# Run example drive node (test safety override)
ros2 run safety_controller_pkg example_forward

# Run standalone 2D simulator (no ROS required)
python3 -m safety_controller_pkg.basic_sim.run_sim
```

## Architecture

ROS 2 Python package for MIT RACECAR safety. Uses the VESC mux architecture:

- **SafetyNode** subscribes to `/scan` (LaserScan), publishes stop commands to `/vesc/low_level/input/safety`
- **Navigation nodes** (wall follower, example_forward) publish to `/vesc/high_level/input/nav_*`
- Low-level mux prioritizes safety channel — when SafetyNode publishes a stop, it overrides navigation

SafetyNode is passive when no obstacle detected (publishes nothing, letting nav commands pass through). It only publishes stop commands, never drive commands.

## Safety Parameters (safety_node.py)

| Parameter | Value | Description |
|-----------|-------|-------------|
| SAFE_DISTANCE | 0.25m | Braking threshold |
| CONE_HALF_ANGLE | 2.007 rad (115°) | Half of 230° front scan cone |
| LASER_TIMEOUT | 0.5s | Stop if no /scan messages received |

## basic_sim/

Standalone pygame simulator that runs the **real** safety_node.py unchanged via mocked ROS interfaces (ros_mock.py). Use for visual debugging without full ROS stack.

Controls: WASD=drive, Space=brake, R=reset, M=toggle lidar rays, Esc=quit

## Versioning

On every commit:
1. Bump the patch version (e.g., `1.0.0` → `1.0.1`) in these files:
   - `setup.py` (version parameter)
   - `package.xml` (version element)

2. When the minor version changes (e.g., `1.0.x` → `1.1.0`), also create a git tag:
   ```
   git tag -a v1.1.0 -m "Release v1.1.0"
   ```

Version format: `MAJOR.MINOR.PATCH` (semantic versioning)
- PATCH: bug fixes, small changes (auto-bump on every commit)
- MINOR: new features, significant changes (manual bump, triggers git tag)
- MAJOR: breaking changes (manual bump, triggers git tag)
