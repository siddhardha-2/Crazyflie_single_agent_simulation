# Crazyflie Single-Agent Simulation Workspace Report

This report covers the first-party workspace files that define the behavior of the Crazyflie simulation/hardware stack, plus the major vendored trees and generated artifacts. Generated ROS 2 build outputs under `build/`, `install/`, `log/`, and `cache/` are excluded from functional analysis because they are machine-produced.

## What This Workspace Does

The repository implements a ROS 2 guidance-control-navigation pipeline for a single Crazyflie. Guidance publishes setpoints, Control converts position error into velocity commands, Navigation publishes fused drone state, and the plant layer bridges those commands to either Gazebo simulation or a physical Crazyflie.

## Root-Level Files

- [README.md](README.md) - main project overview, build instructions, simulation launch commands, hardware launch commands, and package summary.
- [analyze_flight.py](analyze_flight.py) - reads a ROS 2 bag and generates flight-analysis plots for state, setpoint, command, trajectory, and IMU data.
- [analyze_extremum.py](analyze_extremum.py) - reads a ROS 2 bag and generates extremum-seeking analysis plots, including convergence, learned weights, and trajectory diagnostics.
- [log3832276383.csv](log3832276383.csv) - recorded log data used by analysis workflows.
- [params746062153.csv](params746062153.csv) - recorded parameter data used by analysis workflows.
- [extremum_simulation_plots/](extremum_simulation_plots/) - generated plot outputs from extremum-seeking analysis.
- [flight_plots/](flight_plots/) - generated plot outputs from flight analysis.

## Source Packages Under `src/`

### [src/README.md](src/README.md)

- Repository-level documentation for the ROS workspace layout and package roles.

### `navigation`

- [src/navigation/package.xml](src/navigation/package.xml) - ROS package manifest for the state-estimation package and its dependencies.
- [src/navigation/CMakeLists.txt](src/navigation/CMakeLists.txt) - builds the C++ node, generates the `DroneState` message, and installs headers and launch files.
- [src/navigation/msg/DroneState.msg](src/navigation/msg/DroneState.msg) - custom drone state message containing position, velocity, orientation, Euler angles, and IMU data.
- [src/navigation/include/navigation/state_estimator.hpp](src/navigation/include/navigation/state_estimator.hpp) - declares the state estimator node, its callbacks, publishers, subscribers, and helper methods.
- [src/navigation/src/state_estimator.cpp](src/navigation/src/state_estimator.cpp) - subscribes to odometry and IMU, converts quaternion orientation to RPY, and publishes `/cf0/state`.
- [src/navigation/launch/navigation.launch.py](src/navigation/launch/navigation.launch.py) - launches the state estimator node with `use_sim_time` support.

### `control`

- [src/control/package.xml](src/control/package.xml) - ROS package manifest for the PID controller package.
- [src/control/CMakeLists.txt](src/control/CMakeLists.txt) - builds and installs the controller executable and its include/config/launch assets.
- [src/control/include/control/pid.hpp](src/control/include/control/pid.hpp) - defines a reusable PID helper with gain updates, reset, and bounded output.
- [src/control/include/control/controller_node.hpp](src/control/include/control/controller_node.hpp) - declares the controller node, PID instances, subscriptions, publishers, timer, and runtime parameters.
- [src/control/src/pid.cpp](src/control/src/pid.cpp) - implements PID computation, anti-windup clamping, gain updates, and reset logic.
- [src/control/src/controller_node.cpp](src/control/src/controller_node.cpp) - reads state/setpoint, computes velocity commands, normalizes yaw error, and publishes `/cf0/cmd_vel` and `/cf0/enable`.
- [src/control/config/pid.yaml](src/control/config/pid.yaml) - default control gains and output limits loaded by the launch file.
- [src/control/launch/control.launch.py](src/control/launch/control.launch.py) - launches the controller node with the PID config file and `use_sim_time` support.

### `Guidance`

- [src/Guidance/package.xml](src/Guidance/package.xml) - ROS package manifest for the guidance and experiment nodes.
- [src/Guidance/setup.py](src/Guidance/setup.py) - Python package metadata and console-script entry points for all guidance executables.
- [src/Guidance/setup.cfg](src/Guidance/setup.cfg) - Python packaging configuration.
- [src/Guidance/resource/Guidance](src/Guidance/resource/Guidance) - ROS 2 resource marker for the package.
- [src/Guidance/Guidance/__init__.py](src/Guidance/Guidance/__init__.py) - package marker for the Python module.
- [src/Guidance/Guidance/pattern_node.py](src/Guidance/Guidance/pattern_node.py) - square-pattern waypoint generator that publishes `/cf0/setpoint` from `/cf0/state` feedback.
- [src/Guidance/Guidance/extremum_seeker.py](src/Guidance/Guidance/extremum_seeker.py) - simulation extremum-seeking controller with sub-stepping, setpoint generation, and debug-topic publishing.
- [src/Guidance/Guidance/extremum_seeker_hardware.py](src/Guidance/Guidance/extremum_seeker_hardware.py) - hardware-oriented extremum seeker that keeps the same ROS graph as simulation but retargets the bounds and objective for a physical run.
- [src/Guidance/Guidance/extremum_seeker_tv_hardware.py](src/Guidance/Guidance/extremum_seeker_tv_hardware.py) - time-varying extremum seeker for hardware with Lighthouse calibration, safe bounds, takeoff handling, and emergency land logic.
- [src/Guidance/Guidance/hardware_hover_test.py](src/Guidance/Guidance/hardware_hover_test.py) - scripted hover experiment that arms, takes off, holds altitude, and lands using Crazyflie high-level services.
- [src/Guidance/Guidance/hardware_circle_test.py](src/Guidance/Guidance/hardware_circle_test.py) - scripted circular-flight experiment that logs Lighthouse pose data, writes CSV output, and saves a plot.
- [src/Guidance/config/guidance.yaml](src/Guidance/config/guidance.yaml) - default parameters for pattern, extremum, and hardware guidance nodes.
- [src/Guidance/launch/guidance.launch.py](src/Guidance/launch/guidance.launch.py) - launches the pattern node and the time-varying hardware extremum seeker.

### `cf_plant`

- [src/cf_plant/package.xml](src/cf_plant/package.xml) - ROS package manifest for the hardware bridge package.
- [src/cf_plant/setup.py](src/cf_plant/setup.py) - Python packaging metadata and console-script entry point for the bridge node.
- [src/cf_plant/setup.cfg](src/cf_plant/setup.cfg) - Python packaging configuration.
- [src/cf_plant/resource/cf_plant](src/cf_plant/resource/cf_plant) - ROS 2 resource marker for the package.
- [src/cf_plant/cf_plant/__init__.py](src/cf_plant/cf_plant/__init__.py) - package marker for the Python module.
- [src/cf_plant/cf_plant/hardware_bridge_node.py](src/cf_plant/cf_plant/hardware_bridge_node.py) - bridges Lighthouse pose to odometry, forwards velocity commands to high-level hover commands, and auto-arms/takes off the drone.
- [src/cf_plant/config/crazyflies.yaml](src/cf_plant/config/crazyflies.yaml) - Crazyflie robot definitions, radio URIs, robot types, and shared firmware parameters.
- [src/cf_plant/launch/connect_hardware.launch.py](src/cf_plant/launch/connect_hardware.launch.py) - starts the Crazyflie server backend and the hardware bridge for one robot.
- [src/cf_plant/launch/connect_731.launch.py](src/cf_plant/launch/connect_731.launch.py) - compatibility wrapper around the generic hardware connection launch.
- [src/cf_plant/launch/hardware.launch.py](src/cf_plant/launch/hardware.launch.py) - launches the full hardware pipeline with navigation, control, and a configurable guidance executable.
- [src/cf_plant/launch/hardware_731.launch.py](src/cf_plant/launch/hardware_731.launch.py) - compatibility wrapper around the full hardware launch.
- [src/cf_plant/test/test_pep257.py](src/cf_plant/test/test_pep257.py) - style check for docstring conventions.
- [src/cf_plant/test/test_flake8.py](src/cf_plant/test/test_flake8.py) - lint check for Python style.
- [src/cf_plant/test/test_copyright.py](src/cf_plant/test/test_copyright.py) - copyright/license check.

### `crazy_sim`

- [src/crazy_sim/package.xml](src/crazy_sim/package.xml) - ROS package manifest for the Gazebo simulation assets.
- [src/crazy_sim/CMakeLists.txt](src/crazy_sim/CMakeLists.txt) - installs the simulation data directories; no compilation logic.
- [src/crazy_sim/launch/spawn_crazyflie_gz.launch.py](src/crazy_sim/launch/spawn_crazyflie_gz.launch.py) - launches Gazebo, sets the model resource path, and starts the ROS-Gazebo bridge.
- [src/crazy_sim/config/ros_gz_bridge.yaml](src/crazy_sim/config/ros_gz_bridge.yaml) - topic bridge mapping for `/clock`, `/cf0/cmd_vel`, `/cf0/enable`, `/cf0/odom`, `/cf0/imu`, and the overhead camera image.
- [src/crazy_sim/worlds/single_crazy_world.sdf](src/crazy_sim/worlds/single_crazy_world.sdf) - Gazebo world containing the ground plane, overhead camera, and the single Crazyflie model.
- [src/crazy_sim/models/cf0/model.config](src/crazy_sim/models/cf0/model.config) - Gazebo model metadata for the Crazyflie asset.
- [src/crazy_sim/models/cf0/model.sdf](src/crazy_sim/models/cf0/model.sdf) - Crazyflie model definition with body, propeller links, IMU sensor, motor plugins, and the velocity-control plugin.

## Vendored Upstream Trees

These directories are large third-party codebases or dependency bundles. They are part of the workspace, but they are not authored here, so this report summarizes them by package rather than repeating thousands of upstream files one by one.

### [src/crazyswarm2/README.md](src/crazyswarm2/README.md)

Vendored upstream Crazyswarm2 workspace. It provides the Crazyflie Python API, simulation backend, example missions, interface definitions, documentation, system tests, and supporting launch assets.

Key subtrees and what they do:

- `src/crazyswarm2/crazyflie_py/` - Crazyflie Python client API, joystick helpers, keyboard control, trajectory utilities, and the main `Crazyflie` wrapper.
- `src/crazyswarm2/crazyflie_sim/` - simulation server, backend models, visualization helpers, and simulation data assets.
- `src/crazyswarm2/crazyflie_examples/` - example launch files and demo scripts such as hover, waypoint, mapping, and trajectory examples.
- `src/crazyswarm2/crazyflie_interfaces/` - ROS message and service definitions used by the Crazyflie APIs.
- `src/crazyswarm2/crazyflie_server_py/` - Python implementation of the Crazyflie server backend.
- `src/crazyswarm2/crazyflie/` - upstream hardware tooling and launch wrappers.
- `src/crazyswarm2/docs/` and `src/crazyswarm2/docs2/` - upstream documentation sources.
- `src/crazyswarm2/systemtests/` - upstream flight/system-test harness and plotting helpers.
- `src/crazyswarm2/ros_ws/` - legacy ROS workspace content preserved from upstream.

### [src/motion_capture_tracking/README.md](src/motion_capture_tracking/README.md)

Vendored motion-capture tracking workspace and its external SDK dependencies. It provides the tracking node, custom messages, configuration, and large upstream SDK trees for OptiTrack, Qualisys, VRPN, Vicon, and related libraries.

Key subtrees and what they do:

- `src/motion_capture_tracking/motion_capture_tracking/` - the ROS package for motion-capture tracking, including the tracking node, launch files, config, and local CMake setup.
- `src/motion_capture_tracking/motion_capture_tracking_interfaces/` - message definitions for named pose data from motion-capture systems.
- `src/motion_capture_tracking/motion_capture_tracking/deps/libmotioncapture/` - upstream motion-capture abstraction layer and example bindings.
- `src/motion_capture_tracking/motion_capture_tracking/deps/librigidbodytracker/` - upstream rigid-body tracking library and examples.
- `src/motion_capture_tracking/motion_capture_tracking/deps/libmotioncapture/deps/vrpn/` - large upstream VRPN dependency tree.
- `src/motion_capture_tracking/motion_capture_tracking/deps/libmotioncapture/deps/qualisys_cpp_sdk/` - upstream Qualisys SDK sources and examples.
- `src/motion_capture_tracking/motion_capture_tracking/deps/libmotioncapture/deps/NatNetSDKCrossplatform/` - upstream OptiTrack/NatNet SDK sources and samples.

## Generated and Runtime Artifacts

These directories are present in the workspace but are not part of the authored application logic:

- [build/](build/) - Colcon build outputs.
- [install/](install/) - Colcon install tree.
- [log/](log/) - Colcon logs and build history.
- [cache/](cache/) - cached build metadata.
- [extremum_simulation_plots/](extremum_simulation_plots/) and [flight_plots/](flight_plots/) - analysis outputs generated by the Python reporting scripts.

## Runtime Flow

1. `Guidance` publishes `/cf0/setpoint`.
2. `control` subscribes to the setpoint and `/cf0/state`, then publishes `/cf0/cmd_vel` and `/cf0/enable`.
3. `navigation` subscribes to `/cf0/odom` and `/cf0/imu`, then publishes fused `/cf0/state`.
4. `cf_plant` or `crazy_sim` provides the plant side: hardware bridging for the physical drone, or Gazebo simulation for the virtual drone.

## Notes

- The first-party files above are described individually.
- The vendored upstream trees are summarized by package because expanding every third-party source file would mostly duplicate upstream code with little value for this workspace report.
- If you want, I can produce a second pass that expands one vendored subtree file-by-file, starting with `src/crazyswarm2/` or `src/motion_capture_tracking/`.