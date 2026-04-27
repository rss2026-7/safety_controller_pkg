# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build & Run

```bash
# Build (from racecar_ws)
cd ~/racecar_ws && colcon build --packages-select safety_controller_pkg && source install/setup.bash

# Run safety node - city mode (default, for Part B)
ros2 run safety_controller_pkg safety_controller_pkg

# Run safety node - race mode (for Part A)
ros2 run safety_controller_pkg safety_controller_pkg --ros-args -p mode:=race

# Run standalone 2D simulator (no ROS required)
python3 -m safety_controller_pkg.basic_sim.run_sim
python3 -m safety_controller_pkg.basic_sim.run_sim --mode race
```

## Architecture

ROS 2 Python package for MIT RACECAR safety. Designed for **Final Challenge 2026**.

### v2.0 Key Features
- **Velocity-dependent stopping distance**: `d = v²/(2a) + margin` — at 4 m/s needs ~1.5m, not 0.25m
- **Dynamic cone angle**: Narrows from 172° to 46° as speed increases (prevents adjacent lane false triggers)
- **Graduated response**: CLEAR → CAUTION (speed limited) → DANGER (stop)
- **External override interface**: `/safety/external_stop` and `/safety/max_speed` for Part B integration
- **Race vs City modes**: Race mode ignores external overrides, uses steering-aware detection

### Topics
- **Subscribes**: `/scan`, `/odom`, `/vesc/odom`, `/safety/external_stop`, `/safety/max_speed`
- **Publishes**: `/vesc/low_level/input/safety`, `/safety/status`

### VESC Mux
- **SafetyNode** publishes to `/vesc/low_level/input/safety` (high priority)
- **Nav nodes** publish to `/vesc/high_level/input/nav_*` (lower priority)
- Low-level mux prioritizes safety channel

## Safety Parameters (safety_node.py)

| Parameter | Default | Description |
|-----------|---------|-------------|
| mode | city | `race` or `city` |
| brake_accel | 6.0 | Braking deceleration (m/s²) |
| min_distance | 0.20 | Minimum stopping threshold (m) |
| max_cone_angle | 1.5 | Cone half-angle at 0 speed (rad) |
| min_cone_angle | 0.4 | Cone half-angle at max speed (rad) |
| laser_timeout | 0.5 | Emergency stop if no /scan (s) |

## basic_sim/

Standalone pygame simulator that runs the **real** safety_node.py via mocked ROS interfaces.

Controls: WASD=drive, Space=brake, R=reset, M=toggle lidar, E=external stop, Q=speed limit, Esc=quit

## Versioning

On every commit:
1. Bump the patch version (e.g., `2.0.0` → `2.0.1`) in these files:
   - `setup.py` (version parameter)
   - `package.xml` (version element)

2. When the minor version changes (e.g., `2.0.x` → `2.1.0`), also create a git tag:
   ```
   git tag -a v2.1.0 -m "Release v2.1.0"
   ```

Current version: **2.3.2**
