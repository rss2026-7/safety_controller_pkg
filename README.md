# Safety Controller Package

Advanced ROS 2 safety controller for MIT RACECAR, designed for **Final Challenge 2026**.

![Demo](.github/demo.gif)

---

## Quick Start

```bash
# Build (from racecar_ws)
cd ~/racecar_ws && colcon build --packages-select safety_controller_pkg && source install/setup.bash

# Run with the default config (config/safety.yaml in the package)
ros2 launch safety_controller_pkg safety.launch.py

# Run with an override config
ros2 launch safety_controller_pkg safety.launch.py config:=/abs/path/to/foo.yaml
```

---

## Configuration

All tunables live in `config/safety.yaml`. **The YAML is the ground truth** —
the node has no code-side defaults and refuses to start if any required key
is missing.

To change behavior, edit the YAML and rebuild (or pass an alternate path
via the `config:=` launch arg). Keys are documented inline in the file.

Top-level sections:

| Section | What it controls |
|---------|------------------|
| `cone` | Speed-dependent detection cone (max/min half-angle, shrink speed) |
| `vehicle` | Wheelbase, lidar-to-bumper offset |
| `physics` | Brake deceleration, reaction time, safety buffer |
| `features` | Toggle external overrides, steering-aware detection, reverse scanning |
| `caution` | CAUTION zone bounds and speed-scaling factors |
| `avoidance` | AVOIDANCE-zone steering behavior |
| `critical` | CRITICAL-zone hard-brake behavior |
| `laser_timeout` | Watchdog (emergency stop if `/scan` stops arriving) |

---

## Physics-Based Stopping Distance

```
stopping_distance = v² / (2·brake_accel) + v·reaction_time
                    + lidar_to_bumper + safety_buffer
```
Floored at `critical.min_distance`.

With `brake_accel = 6.0 m/s²` and `reaction_time = 0.05s`:

| Speed   | Stopping distance |
|---------|-------------------|
| 0.5 m/s | ~0.35 m (floor)   |
| 1.0 m/s | ~0.35 m (floor)   |
| 2.0 m/s | ~0.47 m           |
| 3.0 m/s | ~0.92 m           |
| 4.0 m/s | ~1.55 m           |

---

## Dynamic Cone Angle

```
cone_half_angle = max_angle − (speed / shrink_speed) × (max_angle − min_angle)
```

| Speed | Cone half-angle | Total cone | Why |
|-------|-----------------|------------|-----|
| 0 m/s | 86° | 172° | Wide for parking/maneuvering |
| 2 m/s | 55° | 110° | Moderate |
| 4 m/s | 23° | 46°  | Narrow for straight-line racing |

Wide at low speed (you might be turning sharply); narrow at high speed
(you're going mostly straight, and a wide cone false-triggers on cars in
adjacent lanes during curves).

---

## Graduated Response Zones

| Zone | Distance | Action |
|------|----------|--------|
| **CLEAR** | > `caution.distance_multiplier` × stopping_dist | No intervention; nav passes through |
| **CAUTION** | between stopping_dist and the outer caution edge | Nav speed scaled down linearly (`start_factor` → `end_factor`) |
| **AVOIDANCE** | within CAUTION when factor drops below `escalate_below_factor` | Speed-limited AND actively steering away from the obstacle |
| **CRITICAL** | < stopping_dist | Hard brake at `physics.brake_accel`; steers away if `critical.steer_away` |

---

## Feature Flags

The `features:` block in `safety.yaml` toggles behavior that used to be
mode-dependent. Examples:

```yaml
features:
  respect_external_stop:    true   # listen to /safety/external_stop
  respect_external_speed:   true   # listen to /safety/max_speed
  steering_aware_detection: false  # weight obstacles by projected turn radius
  reverse_scanning:         true   # use rear-FOV-edge rays when v < 0
```

For a Part-A race profile, copy `safety.yaml`, set
`steering_aware_detection: true`, disable `respect_external_*`, narrow the
cone, and pass that file via `config:=...`.

---

## External Override Interface

```python
# From your traffic-light detector:
from std_msgs.msg import Bool
stop_pub = node.create_publisher(Bool, '/safety/external_stop', 10)
stop_pub.publish(Bool(data=True))   # request stop
stop_pub.publish(Bool(data=False))  # clear

# From your intersection-approach logic:
from std_msgs.msg import Float32
limit_pub = node.create_publisher(Float32, '/safety/max_speed', 10)
limit_pub.publish(Float32(data=1.0))   # cap at 1 m/s
limit_pub.publish(Float32(data=-1.0))  # clear cap
```

External overrides are gated by `features.respect_external_stop` and
`features.respect_external_speed`. Disable them for a race profile.

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
| `/vesc/low_level/input/safety` | `ackermann_msgs/AckermannDriveStamped` | Stop / speed-limit commands |
| `/safety/status` | `std_msgs/Float32` | Current zone (0=CLEAR, 1=CAUTION, 2=AVOIDANCE, 3=CRITICAL) |

---

## Integration with Final Challenge

### Part A: The Great Snail Race

Make a race-profile YAML (copy `safety.yaml` and tune):
- Disable `respect_external_stop` and `respect_external_speed`
- Enable `steering_aware_detection`
- Narrow the cone (`min_angle` lower, `shrink_speed` lower)
- Optionally disable `avoidance.enabled` (weaving adds risk at speed)

```bash
ros2 launch safety_controller_pkg safety.launch.py config:=/abs/path/to/race.yaml
```

### Part B: Mrs. Puff's Boating School

Default `safety.yaml` is tuned for city driving. Integration points:

1. **Traffic Light** — publish `Bool(True)` to `/safety/external_stop`
2. **Pedestrian Crossing** — same
3. **Approaching Intersection** — publish speed cap to `/safety/max_speed`
4. **Parking** — graduated CAUTION zone lets you approach slowly

```python
class TrafficLightNode(Node):
    def __init__(self):
        super().__init__('traffic_light_detector')
        self.stop_pub = self.create_publisher(Bool, '/safety/external_stop', 10)

    def image_callback(self, msg):
        red = self.detect_red_light(msg)
        self.stop_pub.publish(Bool(data=red))
```

---

## Architecture

```
                    ┌─────────────────────────────────────────────┐
                    │                 SafetyNode                  │
                    │    (parameters loaded from safety.yaml)     │
  /scan ──────────► │  ┌──────────────────────────────────────┐  │
                    │  │ 1. Read current speed from /odom      │  │
  /odom ──────────► │  │ 2. Compute stopping_dist = f(speed)   │  │
                    │  │ 3. Compute cone_angle = g(speed)      │  │
  /vesc/.../nav_0 ─►│  │ 4. (Opt.) weight by steering          │  │
                    │  │ 5. Check external overrides           │  │
  /safety/ext_stop ►│  │ 6. Classify zone                      │  │
                    │  │ 7. Publish stop / speed-limit cmd     │  │
  /safety/max_speed►│  └──────────────────────────────────────┘  │
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

**"Node refuses to start with `Missing required safety-controller parameter ...`"**
The YAML config is the ground truth. Either you launched without a params
file (use `ros2 launch` rather than `ros2 run`), or a key was deleted from
the YAML. Add it back; there are no defaults.

**"Car still crashes at high speed"**
Check that `/odom` is being published. Without velocity data, the
controller can't compute dynamic thresholds.
```
ros2 topic echo /odom
```

**"False triggers from adjacent lanes during race"**
Use a race profile with a narrower cone (`cone.min_angle` lower) and
`steering_aware_detection: true`.

**"External stop not working"**
Check `features.respect_external_stop` is `true` in the loaded YAML, and
the topic name is `/safety/external_stop` (not `/external_stop`).

---

## Changelog

### v2.4.0 (2026-04-27)
- **BREAKING:** All tunables moved to `config/safety.yaml` (ground truth).
  No more code-side defaults — the node refuses to start if any key is missing.
- **BREAKING:** Removed the `mode` parameter and all race/city branching.
  Behavior that used to depend on mode is now controlled by explicit feature
  flags (`features.respect_external_stop`, `features.steering_aware_detection`,
  `features.reverse_scanning`).
- **BREAKING:** Removed the basic_sim pseudo-simulator (inaccurate vs real sim).
- Added `launch/safety.launch.py` — launch with `ros2 launch …` and pass
  `config:=…` to override.
- Each safety zone (CAUTION / AVOIDANCE / CRITICAL) now has its own dedicated
  block in the YAML with all its tunables.

### v2.3.0 (2026-04-18)
- Account for LIDAR-to-bumper offset (`vehicle.lidar_to_bumper`).
- Added `physics.safety_buffer` for an explicit margin beyond physics distance.

### v2.2.0 (2026-04-18)
- CRITICAL zone is velocity-dependent — triggers earlier at high speed.

### v2.1.0 (2026-04-18)
- Added AVOIDANCE zone — speed-limited steer-away when caution factor drops below threshold.

### v2.0.0 (2026-04-18)
- Velocity-dependent stopping distance and cone angle.
- Graduated response zones (CLEAR / CAUTION / AVOIDANCE / CRITICAL).
- External override interface (`/safety/external_stop`, `/safety/max_speed`).
- Direction-aware scanning, steering-aware obstacle weighting.
- Added `/odom` subscription and `/safety/status` publication.

### v1.0.0
- Initial release. Fixed 0.25m threshold, fixed 230° cone, binary stop.
