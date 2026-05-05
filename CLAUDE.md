# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build & Run

```bash
# Build (from racecar_ws)
cd ~/racecar_ws && colcon build --packages-select safety_controller_pkg && source install/setup.bash

# Run safety node (loads config/safety.yaml from share dir)
ros2 launch safety_controller_pkg safety.launch.py

# Override the config file
ros2 launch safety_controller_pkg safety.launch.py config:=/abs/path/to/foo.yaml
```

## Configuration

All tunables live in `config/safety.yaml`. The YAML is the **ground truth** —
the node has no code-side defaults and refuses to start if any required key
is missing. To change behavior, edit the YAML and rebuild (or pass an
override path via the `config:=` launch arg).

Top-level sections: `cone`, `vehicle`, `physics`, `features`, `caution`,
`critical`, `person_critical`, `laser_timeout`. See the YAML's inline
comments for the meaning and units of every key.

Safety only modifies SPEED. Steering is passed through unchanged from the
latest nav command on every published frame.

## Architecture

ROS 2 Python package for MIT RACECAR safety. Designed for **Final Challenge 2026**.

### Key Features
- **Velocity-dependent stopping distance**: `d = v²/(2a) + margin`
- **Dynamic cone angle**: narrows at high speed to avoid adjacent-lane false triggers
- **Graduated response zones**: CLEAR → CAUTION → CRITICAL
- **External override interface**: `/safety/external_stop` and `/safety/max_speed`
- **Feature flags** (in `config/safety.yaml`): toggle external-override handling,
  steering-aware detection, and reverse-direction scanning

### Topics
- **Subscribes**: `/scan`, `/odom`, `/vesc/odom`, `/vesc/high_level/input/nav_0`,
  `/safety/external_stop`, `/safety/max_speed`
- **Publishes**: `/vesc/low_level/input/safety`, `/safety/status`

### VESC Mux
- **SafetyNode** publishes to `/vesc/low_level/input/safety` (high priority)
- **Nav nodes** publish to `/vesc/high_level/input/nav_*` (lower priority)
- Low-level mux prioritizes safety channel

## Versioning

On every commit:
1. Bump the patch version (e.g., `2.0.0` → `2.0.1`) in these files:
   - `setup.py` (version parameter)
   - `package.xml` (version element)

2. When the minor version changes (e.g., `2.0.x` → `2.1.0`), also create a git tag:
   ```
   git tag -a v2.1.0 -m "Release v2.1.0"
   ```

Current version: **2.5.1**
