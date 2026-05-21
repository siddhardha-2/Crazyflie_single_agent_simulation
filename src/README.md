# Crazyflie BMO — ROS2 Gazebo Simulation

A ROS2-based simulation framework for a swarm of Crazyflie drones using Gazebo. The system implements a full **Guidance → Control → Navigation** pipeline with a PID controller and modular guidance strategies.

---

## Quick Start

1. **Build the workspace:**
   ```bash
   cd ~/bitcraze_ws
   colcon build
   source install/setup.bash
   ```

2. **Launch the full stack (4 terminals):**
   ```bash
   # Terminal 1: Gazebo Simulation
   ros2 launch crazy_sim spawn_crazyflie_gz.launch.py
   
   # Terminal 2: Navigation (State Estimation)
   ros2 launch navigation navigation.launch.py
   
   # Terminal 3: Control (PID Controller)
   ros2 launch control control.launch.py
   
   # Terminal 4: Guidance (Waypoint Planning)
   ros2 launch Guidance guidance.launch.py
   ```

3. **Monitor the drone:**
   ```bash
   ros2 topic echo /cf0/state
   ```

4. **Record flight data:**
   ```bash
   ros2 bag record /cf0/state /cf0/setpoint /cf0/cmd_vel /cf0/imu
   ```

5. **Analyze recorded flight:**
   ```bash
   python3 analyze_flight.py ~/rosbag2_<timestamp>
   ```

---

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                          Guidance                               │
│         Desired Setpoint (Square / BMO / Formation)             │
└───────────────────────────┬─────────────────────────────────────┘
                            │ /cf0/setpoint (PoseStamped)
                            ▼
                    ┌───────────────┐
                    │  error = des  │
                    │   - current   │
                    └───────┬───────┘
                            │
┌───────────────────────────▼─────────────────────────────────────┐
│                          Control                                │
│              PID Controller (position → velocity)               │
└───────────────────────────┬─────────────────────────────────────┘
                            │ /cf0/cmd_vel (Twist)
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│                           Plant                                 │
│              Swarm of Crazyflies (Gazebo / Real)                │
└───────────────────────────┬─────────────────────────────────────┘
                            │ /cf0/odom, /cf0/imu
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│                        Navigation                               │
│         Localization — Odometry + IMU State Estimation          │
└───────────────────────────┬─────────────────────────────────────┘
                            │ /cf0/state (DroneState)
                            └──────────────► back to Control
```

---

## Packages

### `crazy_sim`
Gazebo simulation of the Crazyflie drone.

- **Launch:** `spawn_crazyflie_gz.launch.py` (spawns Gazebo with Crazyflie model)
- Crazyflie SDF model with 4 motors, IMU sensor, and odometry publisher
- Velocity controller plugin (`MulticopterVelocityControl`)
- `ros_gz_bridge` config to bridge Gazebo ↔ ROS2 topics
- Single drone world file (`single_crazy_world.sdf`)
- Located in `crazy_sim/models/` (Crazyflie model)
- Located in `crazy_sim/worlds/` (world definition)

**Components:**
- **Model:** SDF model of Crazyflie with physics properties
- **Plugins:**
  - `gz-sim-multicopter-motor-model` - Motor dynamics
  - `gz-sim-apply-joint-force` - Rotor thrust application
  - `gz-sim-sensors-imu` - IMU sensor
  - `gz-sim-odometry-publisher` - Publishes odometry
- **Ros2-GZ Bridge:** Bridges ROS2 topics to Gazebo topics

**Key Topics Bridged:**

| ROS2 Topic | Direction | Type | Description |
|---|---|---|---|
| `/cf0/cmd_vel` | ROS → Gazebo | `geometry_msgs/Twist` | Velocity commands |
| `/cf0/enable` | ROS → Gazebo | `std_msgs/Bool` | Arm/disarm signal |
| `/cf0/odom` | Gazebo → ROS | `nav_msgs/Odometry` | Drone odometry |
| `/cf0/imu` | Gazebo → ROS | `sensor_msgs/Imu` | IMU data (accel, gyro) |
| `/clock` | Gazebo → ROS | `rosgraph_msgs/Clock` | Simulation time |

---

### `navigation`
State estimation node for the Crazyflie.

- **Node:** `state_estimator` (executable from `navigation/src/state_estimator.cpp`)
- Subscribes to `/cf0/odom` (Gazebo odometry) and `/cf0/imu` (Gazebo IMU)
- Publishes `/cf0/state` as a custom `DroneState` message
- Converts quaternion orientation to roll, pitch, yaw (Euler angles)
- Fuses odometry and IMU data for robust state estimation
- Operates at message-driven rate (publishes when new odometry arrives)

**Custom Message — `DroneState.msg`:**

Located in `navigation/msg/DroneState.msg`:

```
std_msgs/Header header                          # Timestamp and frame
float64 x, y, z                                 # Position (m)
float64 vx, vy, vz                              # Linear velocity (m/s)
geometry_msgs/Quaternion orientation            # Orientation (quaternion)
float64 roll, pitch, yaw                        # Euler angles (rad)
geometry_msgs/Vector3 angular_velocity          # Angular velocity from IMU (rad/s)
geometry_msgs/Vector3 linear_acceleration       # Linear acceleration from IMU (m/s²)
```

**State Fusion:**
- **Odometry Source:** Position (x,y,z), velocity (vx,vy,vz), orientation
- **IMU Source:** Angular velocity, linear acceleration
- **Output:** Combined `DroneState` message with all relevant state information

---

### `control`
PID position controller for the Crazyflie.

- **Node:** `controller_node` (executable from `control/src/controller_node.cpp`)
- Subscribes to `/cf0/state` (current state) and `/cf0/setpoint` (desired position)
- Runs 4 independent PID controllers: X, Y, Z, Yaw
- Publishes velocity commands to `/cf0/cmd_vel`
- Publishes drone enable signal to `/cf0/enable`
- Arms the drone automatically on first setpoint
- Runs at configurable rate (default 50 Hz)
- Output saturation prevents excessive velocity commands

**PID Gains (tunable in `config/pid.yaml`):**

| Axis | Kp | Ki | Kd | Max Output |
|---|---|---|---|---|
| X | 1.0 | 0.0 | 0.2 | ±0.5 m/s |
| Y | 1.0 | 0.0 | 0.2 | ±0.5 m/s |
| Z | 1.5 | 0.05 | 0.3 | ±0.3 m/s |
| Yaw | 1.0 | 0.0 | 0.1 | ±0.5 rad/s |

**Parameters:**

```yaml
controller_node:
  ros__parameters:
    # Position control gains (X, Y axes)
    pid_x:
      kp: 1.0
      ki: 0.0
      kd: 0.2
    
    pid_y:
      kp: 1.0
      ki: 0.0
      kd: 0.2
    
    # Altitude control gains (Z axis)
    pid_z:
      kp: 1.5
      ki: 0.05
      kd: 0.3
    
    # Yaw control gains
    pid_yaw:
      kp: 1.0
      ki: 0.0
      kd: 0.1
    
    # Output limits
    max_vel_xy: 0.5       # max horizontal velocity command (m/s)
    max_vel_z: 0.3        # max vertical velocity command (m/s)
    max_yaw_rate: 0.5     # max yaw rate command (rad/s)
    
    # Control loop rate
    control_rate: 50.0    # Hz
```

---

### `Guidance`
Waypoint-based guidance node (Python).

- **Node:** `pattern_node` (executable in `Guidance/Guidance/pattern_node.py`)
- Subscribes to `/cf0/state` to track current position
- Publishes `/cf0/setpoint` (PoseStamped) to the control package
- Runs at configurable rate (default 20 Hz)
- Advances to next waypoint when drone is within `waypoint_tolerance`
- Handles takeoff sequence with separate `takeoff_tolerance`

**Current mode — Square Pattern:**

```
(0,1) ──► (1,1)
  ▲           │
  │           │
(0,0) ◄── (1,0)
```

Drone takes off to configurable altitude then flies a square pattern continuously.

**Configurable Parameters:**

Create `src/Guidance/config/guidance.yaml`:
```yaml
pattern_node:
  ros__parameters:
    square_size: 1.0              # Side length in meters
    altitude: 1.0                 # Flight altitude in meters
    waypoint_tolerance: 0.15      # Distance threshold to next waypoint
    takeoff_tolerance: 0.1        # Distance threshold for takeoff completion
    guidance_rate: 20.0           # Guidance loop rate in Hz
    loop_pattern: true            # Loop the pattern continuously
```

| Parameter | Default | Description |
|---|---|---|
| `square_size` | 1.0 | Side length in meters |
| `altitude` | 1.0 | Flight altitude in meters |
| `waypoint_tolerance` | 0.15 | Distance threshold to next waypoint |
| `takeoff_tolerance` | 0.1 | Distance threshold for takeoff completion |
| `guidance_rate` | 20.0 | Guidance loop rate in Hz |
| `loop_pattern` | true | Loop the pattern continuously |

---

## Workspace Structure

```
bitcraze_ws/
├── src/                           # ROS2 packages
│   ├── crazy_sim/                # Gazebo simulation package
│   │   ├── models/              # Crazyflie SDF model
│   │   ├── worlds/              # Simulation world files
│   │   ├── launch/              # Launch files
│   │   └── config/              # Gazebo bridge config
│   │
│   ├── navigation/              # State estimation package (C++)
│   │   ├── src/                 # state_estimator.cpp
│   │   ├── msg/                 # Custom DroneState message
│   │   ├── launch/              # Launch files
│   │   └── include/             # Header files
│   │
│   ├── control/                 # PID controller package (C++)
│   │   ├── src/                 # controller_node.cpp, pid.cpp
│   │   ├── config/              # pid.yaml parameters
│   │   ├── launch/              # Launch files
│   │   └── include/             # Header files
│   │
│   └── Guidance/                # Waypoint guidance package (Python)
│       ├── Guidance/            # pattern_node.py
│       ├── config/              # guidance.yaml parameters
│       ├── launch/              # Launch files
│       └── setup.py             # Python package setup
│
├── build/                         # Build artifacts (generated)
├── install/                       # Installation directory (generated)
├── log/                           # Build logs
├── flight_plots/                  # Generated analysis plots
├── rosbag2_*/                     # Recorded flight bags
└── analyze_flight.py              # Flight analysis script
```

---

## Dependencies

### ROS2
- **ROS2 Humble** (distribution)
- `rclcpp` - C++ ROS client library
- `rclpy` - Python ROS client library
- `std_msgs` - Standard message types
- `geometry_msgs` - Geometry message types
- `nav_msgs` - Navigation message types
- `sensor_msgs` - Sensor message types

### Simulation & Physics
- **Gazebo Harmonic** (gz-sim)
- `ros_gz_bridge` - Bridges between ROS2 and Gazebo
- `robot_localization` - For sensor fusion (if used)

### Transforms & Math
- `tf2` - Transform library
- `tf2_geometry_msgs` - TF2 ↔ geometry_msgs conversions

### Python Libraries (for analyze_flight.py)
- `rosbag2-py` - ROS2 bag file reader
- `numpy` - Numerical computing
- `matplotlib` - Plotting and visualization
- `scipy` - Scientific computing (interpolation)

---

## Build

### Full Build

```bash
cd ~/bitcraze_ws
colcon build
source install/setup.bash
```

### Build Specific Packages

```bash
# Build individual packages
colcon build --packages-select crazy_sim
colcon build --packages-select navigation
colcon build --packages-select control
colcon build --packages-select Guidance

# Build a package and its dependencies
colcon build --packages-up-to control

# Build with verbose output
colcon build --event-handlers console_direct+
```

### Clean Build

```bash
# Clean all build artifacts
rm -rf build/ install/ log/
colcon build

# Or use colcon-clean (if installed)
colcon clean workspace
colcon build
```

### Build Troubleshooting

- **"Could not find package":** Ensure all dependencies are installed
- **"Gazebo not found":** Install Gazebo: `sudo apt install gz-sim`
- **"Build fails on C++ errors":** Check compiler version (GCC 9+), update: `sudo apt update && sudo apt upgrade`
- **"Message type not found":** Rebuild navigation package first (custom messages): `colcon build --packages-select navigation --dependencies-only`

---

## Running the Full Stack

Open 4 terminals, source the workspace in each (`source ~/bitcraze_ws/install/setup.bash`), then run:

```bash
# Terminal 1 — Gazebo Simulation
ros2 launch crazy_sim spawn_crazyflie_gz.launch.py

# Terminal 2 — Navigation
ros2 launch navigation navigation.launch.py

# Terminal 3 — Control
ros2 launch control control.launch.py

# Terminal 4 — Guidance
ros2 launch Guidance guidance.launch.py
```

---

## Data Recording & Analysis

### Recording Flight Data

Record the drone's state, setpoints, and sensor data to a ROS2 bag file:

```bash
# Record multiple topics
ros2 bag record /cf0/state /cf0/setpoint /cf0/cmd_vel /cf0/imu

# Record all topics (not recommended, large files)
ros2 bag record -a

# Stop recording
Press Ctrl+C
```

Bags are saved in `~/rosbag2_<timestamp>/` folder.

### Analyzing Flight Data

The `analyze_flight.py` script processes ROS2 bag files and generates visualizations:

```bash
python3 analyze_flight.py ~/rosbag2_<timestamp>
```

**Generates plots:**
- 3D flight trajectory with waypoints
- Position (X, Y, Z) over time
- Velocity (Vx, Vy, Vz) over time
- Control inputs (cmd_vel)
- IMU data (angular velocity, linear acceleration)
- Tracking errors (desired vs actual position)

**Requirements:**
- `rosbag2-py`
- `rclpy`
- `numpy`
- `matplotlib`
- `scipy`

All are installed in the workspace environment.

**Output:**
- Plots saved as PNG files in `flight_plots/` directory
- Multiple subplots for comprehensive flight analysis

---

## Advanced Usage

### Tuning PID Gains

PID parameters can be tuned in real-time without rebuilding:

```bash
# Edit the PID gains
nano ~/bitcraze_ws/src/control/config/pid.yaml

# Reload parameters (while control node is running)
ros2 param load /controller_node ~/bitcraze_ws/src/control/config/pid.yaml

# View current parameters
ros2 param get /controller_node pid_x.kp
```

**Tuning Strategy:**
1. Start with **Kp only** (Ki=0, Kd=0) — increase until oscillation appears
2. Add **Kd** to reduce overshoot
3. Add small **Ki** if steady-state error exists

### Dynamic Parameter Updates

Set parameters while nodes are running:

```bash
# Set a single parameter
ros2 param set /controller_node pid_x.kp 1.2

# Set altitude in Guidance node
ros2 param set /pattern_node altitude 2.0

# Get current parameter value
ros2 param get /controller_node max_vel_xy
```

### Manual Waypoint Publishing

Send custom waypoints without using Guidance node:

```bash
# Takeoff to 1.0m altitude
ros2 topic pub /cf0/setpoint geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'world'}, pose: {position: {x: 0.0, y: 0.0, z: 1.0}}}"

# Move to (1.0, 1.0, 1.0)
ros2 topic pub /cf0/setpoint geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'world'}, pose: {position: {x: 1.0, y: 1.0, z: 1.0}}}"
```

### Inspecting Topic Data

```bash
# View message details for a topic
ros2 topic info /cf0/state -v

# Get message type
ros2 topic type /cf0/state

# Record to ROS2 bag and replay
ros2 bag play ~/rosbag2_<timestamp>

# Filter and record specific topics
ros2 bag record /cf0/state /cf0/setpoint -o my_flight_data
```

---

## Troubleshooting

### Drone Won't Arm
- Check `/cf0/enable` topic receives `data: true`
- Verify setpoint is being published to `/cf0/setpoint`
- Check logs: `ros2 node info /controller_node`

### Drone Oscillates or Unstable
- **Too high Kp:** Reduce PID gains for X, Y, Z
- **Missing Kd:** Add derivative term to stabilize
- **High control rate:** Reduce `control_rate` in config

### Poor Tracking (Large Position Error)
- **Increase Kp** for stronger response
- **Check communication:** Verify all nodes are running with `ros2 node list`
- **Increase setpoint frequency:** Make sure Guidance runs at sufficient rate

### Gazebo Simulation Crashes
- Ensure Gazebo Harmonic is installed: `gz sim --version`
- Check disk space: `df -h`
- Rebuild Gazebo packages: `sudo apt install --reinstall gazebo-latest`

### Data Recording Issues
```bash
# Verify bag file exists
ls -la ~/rosbag2_*/rosbag2_*.db3

# Check bag file integrity
ros2 bag info ~/rosbag2_<timestamp>

# Extract topics manually with rosbag2_py
python3 -c "import rosbag2_py; reader = rosbag2_py.SequentialReader(); reader.open('rosbag2_<timestamp>');"
```

### Missing Dependencies at Runtime
```bash
# List installed packages
ros2 pkg list

# Check if Gazebo models are found
find ~/.gazebo -name "*.sdf"

# Verify ROS2 environment
printenv | grep ROS

# Source workspace again
source ~/bitcraze_ws/install/setup.bash
```

---

## Monitoring & Debugging

### View Active Topics
```bash
# List all published topics
ros2 topic list

# Show topic types
ros2 topic list -t

# Monitor topic frequency
ros2 topic hz /cf0/state
```

### View Node Graph
```bash
# Print ROS2 graph
ros2 graph

# View node info
ros2 node info /pattern_node
```

### Record & Playback
```bash
# Record with filter for smaller file
ros2 bag record /cf0/state /cf0/setpoint -o compact_flight

# Play back at 0.5x speed
ros2 bag play ~/rosbag2_<timestamp> --rate 0.5

# Convert bag to CSV
ros2 bag play ~/rosbag2_<timestamp> --clock > flight_data.csv
```

---

## Performance Notes

- **Control Loop:** 50 Hz (20 ms cycle time) — sufficient for stable attitude control
- **Guidance Loop:** 20 Hz (50 ms cycle time) — adequate for waypoint navigation
- **State Estimation:** Event-driven (publishes when odometry updates ~100+ Hz from Gazebo)
- **Simulation Time:** Uses `use_sim_time=True` for synchronized clock

### Typical Timing

- **Takeoff:** ~2-3 seconds to reach altitude with default gains
- **Waypoint Transition:** ~3-5 seconds per meter at max velocity
- **Total Square Pattern:** ~20-30 seconds per loop (1m² square)

### Resource Usage (Gazebo Simulation)

- **CPU:** 1-2 cores (single drone)
- **RAM:** 2-3 GB (Gazebo + ROS2 nodes)
- **Disk:** ~50-100 MB per minute of recorded data

---

## Extending the System

### Adding Custom Guidance Strategies

1. Create new Python node in `Guidance/Guidance/`
2. Subscribe to `/cf0/state`
3. Publish to `/cf0/setpoint`
4. Add launch file in `Guidance/launch/`

Example:
```python
class CircleGuidance(Node):
    def __init__(self):
        super().__init__('circle_guidance')
        self.state_sub = self.create_subscription(
            DroneState, '/cf0/state', self.state_callback, 10)
        self.setpoint_pub = self.create_publisher(
            PoseStamped, '/cf0/setpoint', 10)
    
    def state_callback(self, msg):
        # Generate circular trajectory
        angle = atan2(msg.y, msg.x)
        x = cos(angle) * radius
        y = sin(angle) * radius
        z = self.altitude
        self.publish_setpoint(x, y, z)
```

### Adding Custom Control Strategies

1. Modify `control/src/controller_node.cpp`
2. Add new controller class or control law
3. Rebuild: `colcon build --packages-select control`

### Multi-Drone Extension

1. Duplicate Gazebo model for each drone (cf1, cf2, ...)
2. Create separate ROS2 topics for each: `/cf1/state`, `/cf2/setpoint`, etc.
3. Add formation control logic to Guidance
4. Scale controller parameters as needed

---

## Planned Features

- [ ] BMO (Bacterial Metaheuristic Optimization) guidance
- [ ] Extremum Seeking guidance (for adaptive control)
- [ ] Formation control for multi-drone swarm
- [ ] PID auto-tuning algorithm
- [ ] Real hardware deployment (Lighthouse positioning)
- [ ] Collision avoidance (RRT*)
- [ ] Trajectory tracking (spline-based)
- [ ] Energy optimization
- [ ] Multi-floor simulation worlds

---

## References

- [Bitcraze Crazyflie Simulation](https://github.com/bitcraze/crazyflie-simulation)
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Gazebo Harmonic Documentation](https://gazebosim.org/docs/harmonic/)
