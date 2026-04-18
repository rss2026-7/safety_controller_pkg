# Safety Controller Package v2.0

Advanced ROS 2 safety controller for MIT RACECAR, specifically designed for **Final Challenge 2026**.

## What Changed from v1.0

### Old Behavior (v1.0)
- Fixed 0.25m stopping threshold regardless of speed
- Fixed 230° cone (115° half-angle) always on
- Binary stop only (PWM = 0, letting inertia carry forward)
- No velocity awareness
- No integration with external perception
- Would false-trigger on adjacent lane cars during curves

### New Behavior (v2.0)

| Feature | v1.0 | v2.0 |
|---------|------|------|
| Stopping distance | Fixed 0.25m | **Dynamic: v²/(2a) + margin** |
| At 4 m/s | 0.25m (crashes!) | **~1.5m** (physics-correct) |
| At 1 m/s | 0.25m | **~0.25m** |
| Cone angle | Fixed 230° | **Dynamic: 46°-172°** based on speed |
| At high speed | 230° (false triggers) | **~46°** (narrow, race-safe) |
| At low speed | 230° | **~172°** (wide, maneuver-safe) |
| Response | Binary stop | **Graduated: CLEAR→CAUTION→DANGER→CRITICAL** |
| Collision prevention | None | **Active reverse in CRITICAL zone** |
| Direction | Forward only | **Forward + reverse aware** |
| Steering | Ignored | **Path-weighted obstacle detection** |
| External override | None | **Traffic light/pedestrian integration** |
| Modes | None | **Race vs City** |

---

## Quick Start

```bash
# Build
cd ~/racecar_ws && colcon build --packages-select safety_controller_pkg && source install/setup.bash

# Run on physical car (city mode - default)
ros2 run safety_controller_pkg safety_controller_pkg

# Run in race mode (narrow cone, no external overrides)
ros2 run safety_controller_pkg safety_controller_pkg --ros-args -p mode:=race

# Run standalone simulator (no ROS required)
python3 -m safety_controller_pkg.basic_sim.run_sim

# Simulator in race mode
python3 -m safety_controller_pkg.basic_sim.run_sim --mode race
```

---

## Physics-Based Stopping Distance

The critical fix. At speed `v` with braking deceleration `a`:

```
stopping_distance = v² / (2a) + v * t_reaction + margin
```

With default parameters (`a = 6.0 m/s²`, `t_reaction = 0.05s`):

| Speed | v1.0 Threshold | v2.0 Threshold | Would v1.0 Crash? |
|-------|----------------|----------------|-------------------|
| 0.5 m/s | 0.25m | 0.25m | No |
| 1.0 m/s | 0.25m | 0.25m | No |
| 2.0 m/s | 0.25m | 0.47m | **Yes** |
| 3.0 m/s | 0.25m | 0.92m | **Yes** |
| 4.0 m/s | 0.25m | 1.55m | **Yes** |

**At race speed (4 m/s), v1.0 would detect an obstacle when it's 0.25m away but need 1.55m to stop.**

---

## Dynamic Cone Angle

Prevents false triggers from cars in adjacent lanes during curves (Part A race):

```
cone_half_angle = max_angle - (speed / max_speed) * (max_angle - min_angle)
```

| Speed | Cone Half-Angle | Total Cone | Why |
|-------|-----------------|------------|-----|
| 0 m/s | 86° | 172° | Wide for parking/maneuvering |
| 2 m/s | 63° | 126° | Moderate |
| 4 m/s | 23° | 46° | Narrow for straight-line racing |

At high speed you're going mostly straight, so a narrow cone ahead is appropriate. At low speed you might be turning sharply, so scan wider.

---

## Steering-Aware Detection (Race Mode)

When turning, obstacles on the inside of the turn are more dangerous than those on the outside. The safety controller weights obstacles by their proximity to the projected path:

- Turning left → obstacles on left get higher weight (effectively closer)
- Turning right → obstacles on right get higher weight
- Going straight → symmetric weighting

This prevents over-cautious braking when an obstacle is outside your turn radius.

---

## Graduated Response Zones

Instead of binary stop, four zones with proportional response:

| Zone | Distance | Action |
|------|----------|--------|
| **CLEAR** | > 2× stopping_dist | No intervention, nav has full control |
| **CAUTION** | 1-2× stopping_dist | Speed limited proportionally |
| **DANGER** | < stopping_dist | Emergency stop, max deceleration |
| **CRITICAL** | < 0.15m (~6 inches) | **ACTIVE REVERSE** to escape collision |

### CRITICAL Zone: Active Reverse

The CRITICAL zone is the last line of defense. It uses **velocity-dependent thresholds** so it triggers earlier when approaching fast:

| Speed | CRITICAL Threshold | Why |
|-------|-------------------|-----|
| 0 m/s | 0.15m | Base distance (~6 inches) |
| 1 m/s | 0.23m | Need more time to brake |
| 2 m/s | 0.58m | Even more braking distance |
| 4 m/s | 1.88m | High speed = early trigger |

When CRITICAL is triggered:
1. **Active reverse** at 0.5 m/s (fights user input!)
2. **Escape steering** - steers away from obstacle while reversing
3. Loud warnings in console

**The safety controller OVERRIDES user input.** Even if you hold forward at full throttle, the car will reverse away from obstacles. This is the final authority - collision is virtually impossible.

This gives smoother behavior for Part B (city driving) where you approach obstacles intentionally, while making actual collision nearly impossible.

---

## Direction-Aware Scanning

Scans in the direction of travel:
- Moving forward → scan front cone
- Moving backward → scan rear cone

Essential for Part B where you may need to reverse (e.g., 3-point turn, backing out of parking).

---

## External Override Interface (Part B Integration)

For traffic lights, pedestrians, and parking:

```python
# From your traffic light detector:
from std_msgs.msg import Bool
stop_pub = node.create_publisher(Bool, '/safety/external_stop', 10)
stop_pub.publish(Bool(data=True))   # Request stop
stop_pub.publish(Bool(data=False))  # Clear stop

# From your intersection approach logic:
from std_msgs.msg import Float32
limit_pub = node.create_publisher(Float32, '/safety/max_speed', 10)
limit_pub.publish(Float32(data=1.0))  # Limit to 1 m/s
limit_pub.publish(Float32(data=-1.0)) # Clear limit
```

**Note:** External overrides are ignored in `race` mode.

---

## Modes

### City Mode (default)
- Wide cone at low speed for maneuvering
- Graduated response (CAUTION zone active)
- Listens to external stop/speed requests
- Good for Part B (Boating School)

### Race Mode
- Narrow cone at high speed
- Steering-aware obstacle weighting active
- External overrides ignored
- Aggressive, minimal intervention
- Good for Part A (Snail Race)

```bash
# Set via parameter
ros2 run safety_controller_pkg safety_controller_pkg --ros-args -p mode:=race
```

---

## All Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `mode` | `city` | Operating mode: `city` or `race` |
| `brake_accel` | `6.0` | Braking deceleration (m/s²) |
| `min_distance` | `0.20` | Minimum stopping threshold (m) |
| `critical_distance` | `0.15` | Base CRITICAL zone (minimum, scales with speed) |
| `reverse_speed` | `0.5` | Speed to reverse at in CRITICAL zone (m/s) |
| `lidar_to_bumper` | `0.125` | **Distance from LIDAR to front bumper (critical!)** |
| `safety_buffer` | `0.10` | **Configurable gap between bumper and obstacle** |
| `reaction_time` | `0.05` | Sensor/processing delay (s) |
| `max_cone_angle` | `1.5` | Cone half-angle at 0 speed (rad, ~86°) |
| `min_cone_angle` | `0.4` | Cone half-angle at max speed (rad, ~23°) |
| `caution_multiplier` | `2.0` | CAUTION zone = stopping_dist × this |
| `laser_timeout` | `0.5` | Stop if no /scan for this long (s) |
| `max_speed` | `4.0` | Max expected speed (m/s) |
| `wheelbase` | `0.325` | Car wheelbase for path projection (m) |

Example with custom parameters:
```bash
ros2 run safety_controller_pkg safety_controller_pkg --ros-args \
  -p mode:=race \
  -p brake_accel:=5.0 \
  -p min_cone_angle:=0.3
```

---

## Topics

### Subscribed
| Topic | Type | Description |
|-------|------|-------------|
| `/scan` | `sensor_msgs/LaserScan` | LIDAR data |
| `/odom` | `nav_msgs/Odometry` | Velocity for dynamic thresholds |
| `/vesc/odom` | `nav_msgs/Odometry` | Alternate velocity source |
| `/vesc/high_level/input/nav_0` | `ackermann_msgs/AckermannDriveStamped` | Steering intent |
| `/safety/external_stop` | `std_msgs/Bool` | External stop request |
| `/safety/max_speed` | `std_msgs/Float32` | External speed limit |

### Published
| Topic | Type | Description |
|-------|------|-------------|
| `/vesc/low_level/input/safety` | `ackermann_msgs/AckermannDriveStamped` | Stop/limit commands |
| `/safety/status` | `std_msgs/Float32` | Current zone (0=CLEAR, 1=CAUTION, 2=DANGER) |

---

## Standalone Simulator

Test the safety controller without ROS or the physical car:

```bash
python3 -m safety_controller_pkg.basic_sim.run_sim
```

### Controls
| Key | Action |
|-----|--------|
| W/S | Throttle forward/reverse |
| A/D | Steer left/right |
| Space | Handbrake |
| R | Reset position |
| M | Toggle LIDAR ray visualization |
| E | Toggle external stop (simulates traffic light) |
| Q | Cycle speed limit (simulates intersection) |
| Esc | Quit |

### Simulator Options
```bash
# Load a PNG map
python3 -m safety_controller_pkg.basic_sim.run_sim --map path/to/map.png

# Disable LIDAR noise
python3 -m safety_controller_pkg.basic_sim.run_sim --no-noise

# Adjust scan rate
python3 -m safety_controller_pkg.basic_sim.run_sim --scan-hz 20

# Race mode
python3 -m safety_controller_pkg.basic_sim.run_sim --mode race
```

---

## Integration with Final Challenge

### Part A: The Great Snail Race

Use **race mode**:
```bash
ros2 run safety_controller_pkg safety_controller_pkg --ros-args -p mode:=race
```

Key features for Part A:
- Velocity-dependent stopping distance handles 4 m/s racing
- Narrow cone at speed prevents false triggers from adjacent lanes
- Steering-aware detection handles curves without stopping

**Important:** The dynamic cone narrows at high speed, so cars in lanes 3m away won't trigger your safety controller when you're racing straight.

### Part B: Mrs. Puff's Boating School

Use **city mode** (default):
```bash
ros2 run safety_controller_pkg safety_controller_pkg
```

Integration points:
1. **Traffic Light:** Your red light detector publishes `Bool(True)` to `/safety/external_stop`
2. **Pedestrian Crossing:** Same - publish stop request when pedestrian detected
3. **Approaching Intersection:** Publish speed limit to `/safety/max_speed`
4. **Parking:** The graduated CAUTION zone lets you approach the parking meter slowly

Example traffic light integration:
```python
class TrafficLightNode(Node):
    def __init__(self):
        super().__init__('traffic_light_detector')
        self.stop_pub = self.create_publisher(Bool, '/safety/external_stop', 10)
        # ... YOLO setup ...

    def image_callback(self, msg):
        if self.detect_red_light(msg):
            self.stop_pub.publish(Bool(data=True))
        else:
            self.stop_pub.publish(Bool(data=False))
```

---

## Architecture

```
                    ┌─────────────────────────────────────────────┐
                    │              SafetyNode v2.0                │
                    │                                             │
  /scan ──────────► │  ┌─────────────────────────────────────┐   │
                    │  │ 1. Get current speed from /odom      │   │
  /odom ──────────► │  │ 2. Compute stopping_dist = f(speed)  │   │
                    │  │ 3. Compute cone_angle = g(speed)     │   │
  /vesc/.../nav_0 ─►│  │ 4. Apply steering weighting (race)   │   │
                    │  │ 5. Check external overrides          │   │
  /safety/ext_stop ►│  │ 6. Classify zone: CLEAR/CAUTION/DANGER│  │
                    │  │ 7. Publish appropriate command        │   │
  /safety/max_speed►│  └─────────────────────────────────────┘   │
                    │                                             │
                    │  ───────────────────────────────────────►   │
                    │              /vesc/.../safety               │
                    └─────────────────────────────────────────────┘
                                         │
                                         ▼
                              ┌─────────────────────┐
                              │     VESC Mux        │
                              │ (low-level priority)│
                              └─────────────────────┘
                                         │
                                         ▼
                                      Motors
```

---

## Troubleshooting

### "Car still crashes at high speed"
- Check that `/odom` is being published. Without velocity data, the controller can't compute dynamic thresholds.
- Verify with: `ros2 topic echo /odom`

### "False triggers from adjacent lanes during race"
- Ensure you're in race mode: `-p mode:=race`
- Check that steering angle is being tracked (subscribe to nav topic)

### "External stop not working"
- External overrides are ignored in race mode
- Check topic name: `/safety/external_stop` (not `/external_stop`)

### "Simulator won't start"
- Ensure pygame is installed: `pip install pygame`
- For PNG maps, install Pillow: `pip install Pillow`

---

## Files Changed in v2.0

| File | Changes |
|------|---------|
| `safety_node.py` | Complete rewrite with all new features |
| `basic_sim/ros_mock.py` | Added Odometry, Bool, Float32 message types; parameter support |
| `basic_sim/run_sim.py` | Publishes odometry; E/Q keys for external override testing |
| `basic_sim/renderer.py` | Enhanced HUD showing dynamic parameters and zone |
| `README.md` | This documentation |

---

## Changelog

### v2.3.0 (2026-04-18)
- **CRITICAL FIX:** Account for LIDAR-to-bumper offset (0.125m)
- LIDAR is mounted 0.125m BEHIND the front bumper
- All distance calculations now include this offset
- Added `lidar_to_bumper` parameter (default 0.125m)
- Added `safety_buffer` parameter (default 0.10m) - configurable gap
- **Tested:** 8/8 approach scenarios pass - no collisions from any distance
- Added `--test` mode to simulator for autonomous collision testing

### v2.2.0 (2026-04-18)
- **FIX:** CRITICAL zone now **velocity-dependent** - triggers earlier at high speed
- At 2 m/s, CRITICAL triggers at 0.58m (not 0.15m)
- At 4 m/s, CRITICAL triggers at 1.88m
- **Tested:** 10 seconds holding full throttle toward wall = 0 collisions

### v2.1.0 (2026-04-18)
- **NEW:** CRITICAL zone with **active reverse** to prevent collision
- **NEW:** Escape steering - car steers away from obstacle while reversing
- **NEW:** `critical_distance` parameter (default 0.15m base, scales with speed)
- **NEW:** `reverse_speed` parameter (default 0.5 m/s)
- Simulator: Visual indicator for CRITICAL zone (magenta)
- Simulator: Steering override from safety commands

### v2.0.0 (2026-04-18)
- **BREAKING:** Stopping distance is now velocity-dependent
- **BREAKING:** Cone angle is now velocity-dependent  
- Added `mode` parameter (race/city)
- Added graduated response zones (CLEAR/CAUTION/DANGER)
- Added external override interface (/safety/external_stop, /safety/max_speed)
- Added direction-aware scanning (forward/reverse)
- Added steering-aware obstacle weighting (race mode)
- Added /odom subscription for velocity
- Added /safety/status publishing
- Simulator: Added E/Q keys for testing external overrides
- Simulator: Enhanced HUD with dynamic parameter display

### v1.0.0
- Initial release
- Fixed 0.25m threshold
- Fixed 230° cone
- Binary stop only
