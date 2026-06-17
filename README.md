# Crazyflie Single-Agent Simulation and Hardware

ROS 2 Guidance → Control → Navigation stack for a single Crazyflie. The
repository supports Gazebo simulation and a Lighthouse-positioned physical
Crazyflie through Crazyswarm2.

## Clone and Build

The hardware dependencies are pinned as Git submodules:

```bash
git clone --recurse-submodules \
  --branch Kalesha \
  https://github.com/siddhardha-2/Crazyflie_single_agent_simulation.git
cd Crazyflie_single_agent_simulation
colcon build
source install/setup.bash
```

For an existing clone:

```bash
git submodule update --init --recursive
colcon build
source install/setup.bash
```

## Simulation

Launch the simulator, navigation, control, and guidance in separate terminals:

```bash
ros2 launch crazy_sim spawn_crazyflie_gz.launch.py
ros2 launch navigation navigation.launch.py
ros2 launch control control.launch.py
ros2 launch Guidance guidance.launch.py
```

## Hardware

Hardware launch files require the physical radio URI at launch. No physical URI
is stored in Git, and the logical robot name defaults to `cf0`.

Connect Crazyswarm2 and the hardware bridge:

```bash
ros2 launch cf_plant connect_hardware.launch.py \
  robot_uri:=radio://0/80/2M/E7E7E7E730
```

Run the full hardware Guidance → Control → Navigation stack:

```bash
ros2 launch cf_plant hardware.launch.py \
  robot_uri:=radio://0/80/2M/E7E7E7E730
```

The older `connect_731.launch.py` and `hardware_731.launch.py` filenames remain
available as compatibility wrappers, but they also require `robot_uri`.

## Hardware Experiments

Start `connect_hardware.launch.py` first. These experiments use Crazyswarm2
high-level services directly, so do not run them with `hardware.launch.py`.

Hover at 0.5 m for 10 seconds, then land:

```bash
ros2 run Guidance hardware_hover_test --ros-args \
  -p robot_name:=cf0 \
  -p altitude:=0.5 \
  -p hold_duration:=10.0
```

Fly and log a circle using Lighthouse pose feedback:

```bash
ros2 run Guidance hardware_circle_test --ros-args \
  -p robot_name:=cf0 \
  -p altitude:=0.8 \
  -p radius:=0.8 \
  -p waypoints:=36 \
  -p time_per_point:=0.5 \
  -p output_directory:=experiment_logs
```

The circle test writes a CSV log and trajectory plot under `experiment_logs/`.
Experiment output and Crazyswarm2 runtime caches are ignored by the Git.

## Packages

- `Guidance`: waypoint, extremum-seeking, hover, and circle missions.
- `control`: PID position controller.
- `navigation`: odometry and IMU state estimator.
- `crazy_sim`: Gazebo model, world, and ROS-Gazebo bridge.
- `cf_plant`: Crazyswarm2 hardware connection and command bridge.
