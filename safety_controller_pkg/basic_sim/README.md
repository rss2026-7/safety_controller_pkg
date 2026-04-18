# basic_sim — 2D Safety Controller Demo

A lightweight pygame-based simulator that runs the **real** `safety_node.py`
unchanged. Intended for visualizing how the safety controller reacts to
obstacles, without spinning up the full ROS 2 stack or the racecar docker.

No changes are made to `safety_controller_pkg/safety_node.py` or any other
file outside this folder — this subpackage is purely a demo harness.

## How it works

```
  +-------------------+                          +----------------------+
  |  pygame main loop |--- /scan (LaserScan) --->|  safety_node.py      |
  |  (car + lidar)    |                          |  (REAL, unchanged)   |
  |                   |<-- /vesc/.../safety -----|                      |
  +--------+----------+                          +----------------------+
           |
           v
   if safety msg arrived -> override commanded speed to 0
```

1. `ros_mock.py` installs fake `rclpy`, `sensor_msgs`, `ackermann_msgs`
   modules into `sys.modules`. They implement exactly the API surface the
   safety node touches (Node, publisher/subscriber, Time/Duration, Logger).
2. The real `safety_controller_pkg.safety_node.SafetyNode` is imported and
   instantiated — it subscribes to `/scan` and publishes to
   `/vesc/low_level/input/safety` through our in-process broker.
3. Every frame, the sim:
   - reads WASD input,
   - advances `CarPhysics` (Ackermann bicycle + inertia + steering slew),
   - ray-casts a `LidarParams(num_beams=100, fov=4.71 rad, std_dev=0.01 m)`
     scan against the world, matching the real simulator's parameters,
   - publishes the scan, which synchronously fires the safety node's
     callback,
   - if the safety node emitted a stop, the sim clamps the car's target
     speed to 0 (modelling the VESC low-level mux).

## Running

```bash
cd /home/adhoc/Desktop/ROBOTICSTMP/safety_controller_pkg
python3 -m safety_controller_pkg.basic_sim.run_sim
```

Optional flags:
- `--map <path.png>` — load a grayscale occupancy PNG
  (e.g. `../racecar_simulator/maps/building_31.png`)
- `--resolution 0.05` — meters per cell for the PNG map
- `--no-noise` — disable lidar Gaussian noise
- `--scan-hz 40` — lidar publish rate

## Controls

| Key     | Action                          |
|---------|---------------------------------|
| W / S   | Throttle forward / reverse      |
| A / D   | Steer left / right              |
| Space   | Handbrake                       |
| R       | Reset car to starting pose      |
| M       | Toggle lidar-ray rendering      |
| Esc     | Quit                            |

## What you will see

- **Greyscale map** — occupied cells are dark
- **Green rays** — the simulated lidar scan, with orange dots at each hit
- **Translucent green pie** — the safety cone (230 deg), the region the
  safety node scans for obstacles. It turns **red** when the safety node
  fires a stop command.
- **Inner red disc** — the `SAFE_DISTANCE = 0.25 m` braking radius
- **Car** — blue normally, **red** while the safety node is overriding
- **HUD** — speed, steering angle, current throttle command vs. the
  commanded nav speed, pose, minimum front-cone range, FPS, and a big
  `SAFETY OVERRIDE` / `Safety: clear` banner

## Parameters

Tuning mirrors the real racecar. See `car_physics.py::CarParams` and
`lidar.py::LidarParams`.

| Parameter                    | Value      | Source                               |
|------------------------------|------------|--------------------------------------|
| wheelbase                    | 0.325 m    | racecar_simulator/share/params.yaml  |
| max_speed                    | 4.0 m/s    | params.yaml                          |
| max_steering_angle           | 0.34 rad   | params.yaml                          |
| scan_beams                   | 100        | params.yaml                          |
| scan_field_of_view           | 4.71 rad   | params.yaml                          |
| scan_std_dev                 | 0.01 m     | params.yaml                          |
| scan_distance_to_base_link   | 0.325 m    | front-axle midpoint (= wheelbase); params.yaml value is 0.275 |
| SAFE_DISTANCE                | 0.25 m     | safety_node.py                       |
| CONE_HALF_ANGLE              | 2.007 rad  | safety_node.py                       |

## Files

| File           | Purpose                                                                  |
|----------------|--------------------------------------------------------------------------|
| `run_sim.py`   | Main entry point — wires everything together.                             |
| `ros_mock.py`  | Fake rclpy / sensor_msgs / ackermann_msgs stack.                          |
| `car_physics.py` | Ackermann kinematics + inertia + steering slew rate.                    |
| `lidar.py`     | Grid ray casting producing `LaserScan`-compatible ranges.                 |
| `world.py`     | Occupancy grid (demo world + PNG loader).                                 |
| `renderer.py`  | pygame rendering of world, car, lidar, and safety overlays.               |
