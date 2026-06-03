"""
analyze_extremum.py
-------------------
Complete 2D and 3D analysis plots for PL_EXTREMUM.slx ROS2 port.

Usage:
    python3 analyze_extremum.py <bag_folder>

Example:
    python3 analyze_extremum.py ~/extremum_seeking_ws/rosbag2_2026_05_29-16_51_53

Generates plots in ./extremum_plots/
"""

import sys
import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except ImportError:
    print("ERROR: Source your workspace first:")
    print("  source ~/extremum_seeking_ws/install/setup.bash")
    sys.exit(1)

OUTPUT_DIR = 'extremum_plots'

# True function parameters (from Simulink)
W_STAR = np.array([-1.0, -0.5, 4.0, 2.0, -6.0])

# -----------------------------------------------------------------------
# Read bag
# -----------------------------------------------------------------------
TOPICS = {
    'state':   '/cf0/state',
    'sp':      '/cf0/setpoint',
    'cmd':     '/cf0/cmd_vel',
    'Z1':      '/cf0/Z1',
    'Z11':     '/cf0/Z11',
    'fhat':    '/cf0/f_hat',
    'xdot':    '/cf0/x_dot',
    'what':    '/cf0/w_hat_debug',
}

def read_bag(path):
    print(f"\nReading bag: {path}")
    storage  = rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3')
    conv     = rosbag2_py.ConverterOptions('', '')
    reader   = rosbag2_py.SequentialReader()
    reader.open(storage, conv)
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    data = {k: [] for k in TOPICS}
    while reader.has_next():
        topic, raw, ts = reader.read_next()
        for k, tn in TOPICS.items():
            if topic == tn:
                msg = deserialize_message(raw, get_message(type_map[topic]))
                data[k].append((ts * 1e-9, msg))
    for k in data:
        print(f"  {TOPICS[k]}: {len(data[k])} messages")
    return data

# -----------------------------------------------------------------------
# Extract arrays
# -----------------------------------------------------------------------
def extract(data):
    out = {}

    # State
    if data['state']:
        t0 = data['state'][0][0]
        arr = [(ts-t0, m) for ts, m in data['state']]
        out['t']    = np.array([a[0] for a in arr])
        out['x']    = np.array([a[1].x  for a in arr])
        out['y']    = np.array([a[1].y  for a in arr])
        out['z']    = np.array([a[1].z  for a in arr])
        out['vx']   = np.array([a[1].vx for a in arr])
        out['vy']   = np.array([a[1].vy for a in arr])
        out['vz']   = np.array([a[1].vz for a in arr])
        out['roll'] = np.array([a[1].roll  for a in arr])
        out['pitch']= np.array([a[1].pitch for a in arr])
        out['yaw']  = np.array([a[1].yaw   for a in arr])
        out['ax']   = np.array([a[1].linear_acceleration.x for a in arr])
        out['ay']   = np.array([a[1].linear_acceleration.y for a in arr])
        out['az']   = np.array([a[1].linear_acceleration.z for a in arr])
        out['wx']   = np.array([a[1].angular_velocity.x for a in arr])
        out['wy']   = np.array([a[1].angular_velocity.y for a in arr])
        out['wz']   = np.array([a[1].angular_velocity.z for a in arr])
    else:
        print("WARNING: No /cf0/state messages")

    # Setpoint (algorithm state)
    if data['sp']:
        t0 = data['state'][0][0] if data['state'] else data['sp'][0][0]
        out['sp_t'] = np.array([ts-t0 for ts, _ in data['sp']])
        out['sp_x'] = np.array([m.pose.position.x for _, m in data['sp']])
        out['sp_y'] = np.array([m.pose.position.y for _, m in data['sp']])
        out['sp_z'] = np.array([m.pose.position.z for _, m in data['sp']])

    # cmd_vel
    if data['cmd']:
        t0 = data['state'][0][0] if data['state'] else data['cmd'][0][0]
        out['cmd_t']  = np.array([ts-t0 for ts, _ in data['cmd']])
        out['cmd_lx'] = np.array([m.linear.x  for _, m in data['cmd']])
        out['cmd_ly'] = np.array([m.linear.y  for _, m in data['cmd']])
        out['cmd_lz'] = np.array([m.linear.z  for _, m in data['cmd']])
        out['cmd_az'] = np.array([m.angular.z for _, m in data['cmd']])

    # Algorithm diagnostics
    for key, topic in [('Z1','Z1'), ('Z11','Z11'), ('fhat','fhat')]:
        if data[key]:
            t0 = data['state'][0][0] if data['state'] else data[key][0][0]
            out[f'{key}_t'] = np.array([ts-t0 for ts, _ in data[key]])
            out[f'{key}_v'] = np.array([m.data for _, m in data[key]])

    # x_dot
    if data['xdot']:
        t0 = data['state'][0][0] if data['state'] else data['xdot'][0][0]
        out['xdot_t']  = np.array([ts-t0 for ts, _ in data['xdot']])
        out['xdot_x']  = np.array([m.data[0] for _, m in data['xdot']])
        out['xdot_y']  = np.array([m.data[1] for _, m in data['xdot']])

    # w_hat
    if data['what']:
        t0 = data['state'][0][0] if data['state'] else data['what'][0][0]
        out['what_t'] = np.array([ts-t0 for ts, _ in data['what']])
        out['what']   = np.array([m.data for _, m in data['what']])

    return out

# -----------------------------------------------------------------------
# True function f(x1, x2) = 1 - exp(w_star' * phi)
# -----------------------------------------------------------------------
def f_true(x1, x2):
    phi = np.array([x1**2, x2**2, x1, x2, 1.0])
    return 1.0 - np.exp(float(W_STAR @ phi))

def f_surface(x1_range, x2_range):
    X1, X2 = np.meshgrid(x1_range, x2_range)
    F = np.zeros_like(X1)
    for i in range(X1.shape[0]):
        for j in range(X1.shape[1]):
            val = f_true(X1[i,j], X2[i,j])
            F[i,j] = np.clip(val, -2, 1)
    return X1, X2, F

# -----------------------------------------------------------------------
# Plot 1 — Position vs Time (2D)
# -----------------------------------------------------------------------
def plot_position(d, out):
    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    fig.suptitle('Drone Position vs Time', fontsize=14, fontweight='bold')

    for ax, actual, sp, label, color in zip(
        axes,
        [d['x'], d['y'], d['z']],
        [d.get('sp_x'), d.get('sp_y'), d.get('sp_z')],
        ['X (m)', 'Y (m)', 'Z (m)'],
        ['red', 'green', 'blue']
    ):
        ax.plot(d['t'], actual, color=color, linewidth=1.5, label='Drone actual')
        if sp is not None:
            ax.plot(d['sp_t'], sp, color=color, linestyle='--',
                    linewidth=1.2, alpha=0.7, label='Algorithm setpoint')
        ax.set_ylabel(label)
        ax.legend(loc='upper right')
        ax.grid(True, alpha=0.3)

    axes[-1].set_xlabel('Time (s)')
    plt.tight_layout()
    plt.savefig(os.path.join(out, '01_position_vs_time.png'), dpi=150)
    plt.close()
    print("  Saved: 01_position_vs_time.png")

# -----------------------------------------------------------------------
# Plot 2 — Convergence Z1 and Z11 (2D)
# -----------------------------------------------------------------------
def plot_convergence(d, out):
    fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    fig.suptitle('Algorithm Convergence', fontsize=14, fontweight='bold')

    if 'Z1_t' in d:
        axes[0].plot(d['Z1_t'], d['Z1_v'], 'b-', linewidth=1.5)
        axes[0].axhline(0, color='black', linestyle='--', linewidth=0.8)
        axes[0].fill_between(d['Z1_t'], d['Z1_v'], alpha=0.2)
        axes[0].set_ylabel('Z1 = ||x - [2,2]|| (m)')
        axes[0].set_title('Position Convergence (Z1 → 0)')
        axes[0].grid(True, alpha=0.3)

    if 'Z11_t' in d:
        axes[1].plot(d['Z11_t'], d['Z11_v'], 'r-', linewidth=1.5)
        axes[1].axhline(0, color='black', linestyle='--', linewidth=0.8)
        axes[1].fill_between(d['Z11_t'], d['Z11_v'], alpha=0.2, color='red')
        axes[1].set_ylabel('Z11 = ||w_hat - w_star||')
        axes[1].set_title('Parameter Convergence (Z11 → 0)')
        axes[1].grid(True, alpha=0.3)

    axes[-1].set_xlabel('Time (s)')
    plt.tight_layout()
    plt.savefig(os.path.join(out, '02_convergence.png'), dpi=150)
    plt.close()
    print("  Saved: 02_convergence.png")

# -----------------------------------------------------------------------
# Plot 3 — f_hat vs Time (2D)
# -----------------------------------------------------------------------
def plot_fhat(d, out):
    fig, ax = plt.subplots(figsize=(12, 5))
    fig.suptitle('Estimated Function Value f_hat vs Time', fontsize=14, fontweight='bold')

    if 'fhat_t' in d:
        ax.plot(d['fhat_t'], d['fhat_v'], 'purple', linewidth=1.5, label='f_hat')
        ax.axhline(1.0, color='red', linestyle='--', linewidth=1.0,
                   label='f_max = 1.0 (theoretical)')
        ax.set_ylabel('f_hat')
        ax.set_xlabel('Time (s)')
        ax.legend()
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(os.path.join(out, '03_f_hat_vs_time.png'), dpi=150)
    plt.close()
    print("  Saved: 03_f_hat_vs_time.png")

# -----------------------------------------------------------------------
# Plot 4 — w_hat weights vs Time (2D)
# -----------------------------------------------------------------------
def plot_what(d, out):
    if 'what' not in d:
        return

    fig, ax = plt.subplots(figsize=(12, 6))
    fig.suptitle('Learned Weights w_hat vs Time', fontsize=14, fontweight='bold')

    labels  = ['w0', 'w1', 'w2', 'w3', 'w4']
    colors  = ['red', 'blue', 'green', 'orange', 'purple']
    w_true  = W_STAR

    for i, (label, color, wtrue) in enumerate(zip(labels, colors, w_true)):
        ax.plot(d['what_t'], d['what'][:, i], color=color,
                linewidth=1.5, label=f'{label} (true={wtrue})')
        ax.axhline(wtrue, color=color, linestyle='--',
                   linewidth=0.8, alpha=0.5)

    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Weight value')
    ax.legend(loc='upper right', ncol=2)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(os.path.join(out, '04_w_hat_vs_time.png'), dpi=150)
    plt.close()
    print("  Saved: 04_w_hat_vs_time.png")

# -----------------------------------------------------------------------
# Plot 5 — x_dot (algorithm velocity) vs Time (2D)
# -----------------------------------------------------------------------
def plot_xdot(d, out):
    if 'xdot_t' not in d:
        return

    fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    fig.suptitle('Algorithm Velocity (x_dot) vs Time', fontsize=14, fontweight='bold')

    axes[0].plot(d['xdot_t'], d['xdot_x'], 'red', linewidth=1.5)
    axes[0].axhline(0, color='black', linestyle='--', linewidth=0.8)
    axes[0].set_ylabel('x_dot X (m/s)')
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(d['xdot_t'], d['xdot_y'], 'green', linewidth=1.5)
    axes[1].axhline(0, color='black', linestyle='--', linewidth=0.8)
    axes[1].set_ylabel('x_dot Y (m/s)')
    axes[1].grid(True, alpha=0.3)

    axes[-1].set_xlabel('Time (s)')
    plt.tight_layout()
    plt.savefig(os.path.join(out, '05_x_dot_vs_time.png'), dpi=150)
    plt.close()
    print("  Saved: 05_x_dot_vs_time.png")

# -----------------------------------------------------------------------
# Plot 6 — PID cmd_vel vs Time (2D)
# -----------------------------------------------------------------------
def plot_cmdvel(d, out):
    if 'cmd_t' not in d:
        return

    fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
    fig.suptitle('PID Output (cmd_vel) vs Time', fontsize=14, fontweight='bold')

    for ax, val, label, color in zip(
        axes,
        [d['cmd_lx'], d['cmd_ly'], d['cmd_lz'], d['cmd_az']],
        ['linear.x (m/s)', 'linear.y (m/s)', 'linear.z (m/s)', 'angular.z (rad/s)'],
        ['red', 'green', 'blue', 'orange']
    ):
        ax.plot(d['cmd_t'], val, color=color, linewidth=1.5)
        ax.axhline(0, color='black', linestyle='--', linewidth=0.8)
        ax.set_ylabel(label)
        ax.grid(True, alpha=0.3)

    axes[-1].set_xlabel('Time (s)')
    plt.tight_layout()
    plt.savefig(os.path.join(out, '06_cmd_vel_vs_time.png'), dpi=150)
    plt.close()
    print("  Saved: 06_cmd_vel_vs_time.png")

# -----------------------------------------------------------------------
# Plot 7 — Roll Pitch Yaw vs Time (2D)
# -----------------------------------------------------------------------
def plot_rpy(d, out):
    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    fig.suptitle('Roll Pitch Yaw vs Time', fontsize=14, fontweight='bold')

    for ax, val, label, color in zip(
        axes,
        [np.degrees(d['roll']), np.degrees(d['pitch']), np.degrees(d['yaw'])],
        ['Roll (deg)', 'Pitch (deg)', 'Yaw (deg)'],
        ['red', 'green', 'blue']
    ):
        ax.plot(d['t'], val, color=color, linewidth=1.5)
        ax.axhline(0, color='black', linestyle='--', linewidth=0.8)
        ax.set_ylabel(label)
        ax.grid(True, alpha=0.3)

    axes[-1].set_xlabel('Time (s)')
    plt.tight_layout()
    plt.savefig(os.path.join(out, '07_roll_pitch_yaw.png'), dpi=150)
    plt.close()
    print("  Saved: 07_roll_pitch_yaw.png")

# -----------------------------------------------------------------------
# Plot 8 — IMU Data (2D)
# -----------------------------------------------------------------------
def plot_imu(d, out):
    fig, axes = plt.subplots(2, 3, figsize=(15, 8))
    fig.suptitle('IMU Data', fontsize=14, fontweight='bold')

    items = [
        (d['wx'], 'ω X (rad/s)', 'red',    axes[0][0]),
        (d['wy'], 'ω Y (rad/s)', 'green',  axes[0][1]),
        (d['wz'], 'ω Z (rad/s)', 'blue',   axes[0][2]),
        (d['ax'], 'Acc X (m/s²)', 'red',   axes[1][0]),
        (d['ay'], 'Acc Y (m/s²)', 'green', axes[1][1]),
        (d['az'], 'Acc Z (m/s²)', 'blue',  axes[1][2]),
    ]
    for val, label, color, ax in items:
        ax.plot(d['t'], val, color=color, linewidth=1.0)
        ax.set_title(label)
        ax.set_xlabel('Time (s)')
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(os.path.join(out, '08_imu_data.png'), dpi=150)
    plt.close()
    print("  Saved: 08_imu_data.png")

# -----------------------------------------------------------------------
# Plot 9 — 2D Trajectory X vs Y
# -----------------------------------------------------------------------
def plot_2d_trajectory(d, out):
    fig, ax = plt.subplots(figsize=(8, 8))
    fig.suptitle('2D Trajectory — Top View', fontsize=14, fontweight='bold')

    # Color trajectory by time
    points = np.array([d['x'], d['y']]).T.reshape(-1, 1, 2)
    segs   = np.concatenate([points[:-1], points[1:]], axis=1)
    from matplotlib.collections import LineCollection
    from matplotlib.colors import Normalize
    norm = Normalize(vmin=d['t'].min(), vmax=d['t'].max())
    lc   = LineCollection(segs, cmap='plasma', norm=norm, linewidth=2)
    lc.set_array(d['t'][:-1])
    ax.add_collection(lc)
    plt.colorbar(lc, ax=ax, label='Time (s)')

    # Setpoint path
    if 'sp_x' in d:
        ax.plot(d['sp_x'], d['sp_y'], 'r--', linewidth=1.0,
                alpha=0.6, label='Algorithm state')

    # Start and end
    ax.plot(d['x'][0],  d['y'][0],  'go', markersize=12, label='Start', zorder=5)
    ax.plot(d['x'][-1], d['y'][-1], 'r*', markersize=15, label='End',   zorder=5)
    ax.plot(2.0, 2.0, 'k^', markersize=12, label='True maximum (2,2)', zorder=5)

    ax.set_xlim(min(d['x'].min(), -0.2) - 0.1, max(d['x'].max(), 2.2) + 0.1)
    ax.set_ylim(min(d['y'].min(), -0.2) - 0.1, max(d['y'].max(), 2.2) + 0.1)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.legend(loc='lower right')
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')

    plt.tight_layout()
    plt.savefig(os.path.join(out, '09_2d_trajectory.png'), dpi=150)
    plt.close()
    print("  Saved: 09_2d_trajectory.png")

# -----------------------------------------------------------------------
# Plot 10 — 2D Trajectory on f surface (top view)
# -----------------------------------------------------------------------
def plot_trajectory_on_surface_2d(d, out):
    fig, ax = plt.subplots(figsize=(9, 8))
    fig.suptitle('Drone Trajectory on f(x,y) Surface — Top View',
                 fontsize=14, fontweight='bold')

    x_range = np.linspace(-0.5, 2.5, 100)
    y_range = np.linspace(-0.5, 2.5, 100)
    X1, X2, F = f_surface(x_range, y_range)

    contour = ax.contourf(X1, X2, F, levels=30, cmap='RdYlGn', alpha=0.7)
    plt.colorbar(contour, ax=ax, label='f(x,y)')
    ax.contour(X1, X2, F, levels=10, colors='white', alpha=0.3, linewidths=0.5)

    # Drone trajectory
    ax.plot(d['x'], d['y'], 'b-', linewidth=2, label='Drone path', zorder=5)
    ax.plot(d['x'][0],  d['y'][0],  'wo', markersize=10,
            markeredgecolor='black', label='Start', zorder=6)
    ax.plot(d['x'][-1], d['y'][-1], 'w*', markersize=14,
            markeredgecolor='black', label='End', zorder=6)
    ax.plot(2.0, 2.0, 'k^', markersize=12, label='True max (2,2)', zorder=6)

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.legend(loc='lower right')
    ax.set_aspect('equal')

    plt.tight_layout()
    plt.savefig(os.path.join(out, '10_trajectory_on_surface_2d.png'), dpi=150)
    plt.close()
    print("  Saved: 10_trajectory_on_surface_2d.png")

# -----------------------------------------------------------------------
# Plot 11 — 3D Flight Trajectory
# -----------------------------------------------------------------------
def plot_3d_trajectory(d, out):
    fig = plt.figure(figsize=(10, 8))
    ax  = fig.add_subplot(111, projection='3d')
    fig.suptitle('3D Flight Trajectory', fontsize=14, fontweight='bold')

    ax.plot(d['x'], d['y'], d['z'], 'b-', linewidth=1.5, label='Drone')
    if 'sp_x' in d:
        ax.plot(d['sp_x'], d['sp_y'], d['sp_z'], 'r--',
                linewidth=1.0, alpha=0.6, label='Setpoint')

    ax.scatter(d['x'][0],  d['y'][0],  d['z'][0],
               color='green', s=100, zorder=5, label='Start')
    ax.scatter(d['x'][-1], d['y'][-1], d['z'][-1],
               color='red',   s=100, zorder=5, label='End')
    ax.scatter(2.0, 2.0, d['z'].mean(), color='black',
               marker='^', s=150, label='True max projection')

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.legend()

    plt.tight_layout()
    plt.savefig(os.path.join(out, '11_3d_trajectory.png'), dpi=150)
    plt.close()
    print("  Saved: 11_3d_trajectory.png")

# -----------------------------------------------------------------------
# Plot 12 — 3D f(x,y) surface + trajectory
# -----------------------------------------------------------------------
def plot_3d_surface_trajectory(d, out):
    fig = plt.figure(figsize=(12, 9))
    ax  = fig.add_subplot(111, projection='3d')
    fig.suptitle('f(x,y) Surface with Drone Trajectory',
                 fontsize=14, fontweight='bold')

    x_range = np.linspace(-0.3, 2.5, 80)
    y_range = np.linspace(-0.3, 2.5, 80)
    X1, X2, F = f_surface(x_range, y_range)

    surf = ax.plot_surface(X1, X2, F, cmap='RdYlGn',
                           alpha=0.6, linewidth=0, antialiased=True)
    plt.colorbar(surf, ax=ax, label='f(x,y)', shrink=0.5)

    # Drone trajectory on surface
    f_drone = np.array([np.clip(f_true(xi, yi), -2, 1)
                        for xi, yi in zip(d['x'], d['y'])])
    ax.plot(d['x'], d['y'], f_drone + 0.02, 'b-',
            linewidth=2.5, label='Drone path', zorder=5)

    ax.scatter(d['x'][0],  d['y'][0],  f_drone[0],
               color='green', s=100, label='Start', zorder=6)
    ax.scatter(d['x'][-1], d['y'][-1], f_drone[-1],
               color='red', s=100, label='End', zorder=6)
    ax.scatter(2.0, 2.0, f_true(2.0, 2.0),
               color='black', marker='^', s=200,
               label='True max', zorder=6)

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('f(x,y)')
    ax.legend()

    plt.tight_layout()
    plt.savefig(os.path.join(out, '12_3d_surface_trajectory.png'), dpi=150)
    plt.close()
    print("  Saved: 12_3d_surface_trajectory.png")

# -----------------------------------------------------------------------
# Plot 13 — 3D w_hat convergence
# -----------------------------------------------------------------------
def plot_3d_what_convergence(d, out):
    if 'what' not in d:
        return

    fig = plt.figure(figsize=(10, 8))
    ax  = fig.add_subplot(111, projection='3d')
    fig.suptitle('w_hat Convergence Path (w2, w3, Z11)',
                 fontsize=14, fontweight='bold')

    w2  = d['what'][:, 2]
    w3  = d['what'][:, 3]
    z11 = d['Z11_v'] if 'Z11_v' in d else np.linalg.norm(
        d['what'] - W_STAR, axis=1)

    if len(z11) != len(w2):
        z11 = np.interp(d['what_t'], d['Z11_t'], d['Z11_v']) \
              if 'Z11_t' in d else np.zeros(len(w2))

    ax.plot(w2, w3, z11, 'b-', linewidth=1.5, label='w_hat path')
    ax.scatter(w2[0],  w3[0],  z11[0],
               color='green', s=100, label='Start')
    ax.scatter(w2[-1], w3[-1], z11[-1],
               color='red',   s=100, label='End')
    ax.scatter(W_STAR[2], W_STAR[3], 0,
               color='black', marker='^', s=200,
               label=f'w_star (w2={W_STAR[2]}, w3={W_STAR[3]})')

    ax.set_xlabel('w2')
    ax.set_ylabel('w3')
    ax.set_zlabel('Z11')
    ax.legend()

    plt.tight_layout()
    plt.savefig(os.path.join(out, '13_3d_what_convergence.png'), dpi=150)
    plt.close()
    print("  Saved: 13_3d_what_convergence.png")

# -----------------------------------------------------------------------
# Plot 14 — 3D Phase portrait (x, y, Z1)
# -----------------------------------------------------------------------
def plot_3d_phase(d, out):
    if 'Z1_v' not in d:
        return

    fig = plt.figure(figsize=(10, 8))
    ax  = fig.add_subplot(111, projection='3d')
    fig.suptitle('3D Phase Portrait — (x, y, Z1)',
                 fontsize=14, fontweight='bold')

    z1 = np.interp(d['t'], d['Z1_t'], d['Z1_v'])

    ax.plot(d['x'], d['y'], z1, 'purple', linewidth=1.5)
    ax.scatter(d['x'][0],  d['y'][0],  z1[0],
               color='green', s=100, label='Start')
    ax.scatter(d['x'][-1], d['y'][-1], z1[-1],
               color='red',   s=100, label='End (converged)')

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z1 = ||x - [2,2]|| (m)')
    ax.legend()

    plt.tight_layout()
    plt.savefig(os.path.join(out, '14_3d_phase_portrait.png'), dpi=150)
    plt.close()
    print("  Saved: 14_3d_phase_portrait.png")

# -----------------------------------------------------------------------
# Plot 15 — Summary Dashboard
# -----------------------------------------------------------------------
def plot_dashboard(d, out):
    fig = plt.figure(figsize=(18, 12))
    fig.suptitle('Extremum Seeking — Summary Dashboard',
                 fontsize=16, fontweight='bold')
    gs  = gridspec.GridSpec(3, 4, figure=fig)

    # Position
    ax1 = fig.add_subplot(gs[0, :2])
    ax1.plot(d['t'], d['x'], 'r-', label='x')
    ax1.plot(d['t'], d['y'], 'g-', label='y')
    ax1.plot(d['t'], d['z'], 'b-', label='z')
    ax1.axhline(2.0, color='k', linestyle='--', alpha=0.4, label='target=2')
    ax1.set_title('Position'); ax1.set_ylabel('m')
    ax1.legend(loc='upper left', ncol=4); ax1.grid(True, alpha=0.3)

    # Z1 convergence
    ax2 = fig.add_subplot(gs[0, 2])
    if 'Z1_v' in d:
        ax2.plot(d['Z1_t'], d['Z1_v'], 'b-', linewidth=1.5)
        ax2.fill_between(d['Z1_t'], d['Z1_v'], alpha=0.2)
    ax2.set_title('Z1 → 0'); ax2.set_ylabel('m'); ax2.grid(True, alpha=0.3)

    # Z11 convergence
    ax3 = fig.add_subplot(gs[0, 3])
    if 'Z11_v' in d:
        ax3.plot(d['Z11_t'], d['Z11_v'], 'r-', linewidth=1.5)
        ax3.fill_between(d['Z11_t'], d['Z11_v'], alpha=0.2, color='red')
    ax3.set_title('Z11 → 0'); ax3.set_ylabel(''); ax3.grid(True, alpha=0.3)

    # f_hat
    ax4 = fig.add_subplot(gs[1, :2])
    if 'fhat_v' in d:
        ax4.plot(d['fhat_t'], d['fhat_v'], 'purple', linewidth=1.5)
        ax4.axhline(1.0, color='r', linestyle='--', alpha=0.6, label='f_max=1')
    ax4.set_title('f_hat'); ax4.set_ylabel('f_hat'); ax4.legend()
    ax4.grid(True, alpha=0.3)

    # 2D trajectory
    ax5 = fig.add_subplot(gs[1, 2:])
    x_range = np.linspace(-0.3, 2.5, 60)
    y_range = np.linspace(-0.3, 2.5, 60)
    X1, X2, F = f_surface(x_range, y_range)
    ax5.contourf(X1, X2, F, levels=20, cmap='RdYlGn', alpha=0.6)
    ax5.plot(d['x'], d['y'], 'b-', linewidth=2)
    ax5.plot(d['x'][0],  d['y'][0],  'go', markersize=8)
    ax5.plot(d['x'][-1], d['y'][-1], 'r*', markersize=12)
    ax5.plot(2.0, 2.0, 'k^', markersize=10)
    ax5.set_title('Trajectory on f(x,y)')
    ax5.set_xlabel('X (m)'); ax5.set_ylabel('Y (m)')
    ax5.set_aspect('equal'); ax5.grid(True, alpha=0.2)

    # w_hat
    ax6 = fig.add_subplot(gs[2, :2])
    if 'what' in d:
        colors = ['red','blue','green','orange','purple']
        for i, (c, wtrue) in enumerate(zip(colors, W_STAR)):
            ax6.plot(d['what_t'], d['what'][:, i], color=c,
                     linewidth=1.2, label=f'w{i}')
            ax6.axhline(wtrue, color=c, linestyle='--',
                        linewidth=0.6, alpha=0.5)
    ax6.set_title('w_hat weights')
    ax6.set_xlabel('Time (s)'); ax6.legend(ncol=5); ax6.grid(True, alpha=0.3)

    # RPY
    ax7 = fig.add_subplot(gs[2, 2:])
    ax7.plot(d['t'], np.degrees(d['roll']),  'r-', label='roll')
    ax7.plot(d['t'], np.degrees(d['pitch']), 'g-', label='pitch')
    ax7.plot(d['t'], np.degrees(d['yaw']),   'b-', label='yaw')
    ax7.set_title('Roll Pitch Yaw')
    ax7.set_xlabel('Time (s)'); ax7.set_ylabel('deg')
    ax7.legend(ncol=3); ax7.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(os.path.join(out, '00_dashboard.png'), dpi=150)
    plt.close()
    print("  Saved: 00_dashboard.png")

# -----------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------
def main():
    if len(sys.argv) < 2:
        print("Usage: python3 analyze_extremum.py <bag_folder>")
        sys.exit(1)

    bag_path = sys.argv[1]
    if not os.path.exists(bag_path):
        print(f"ERROR: Path not found: {bag_path}")
        sys.exit(1)

    os.makedirs(OUTPUT_DIR, exist_ok=True)
    print(f"Output: ./{OUTPUT_DIR}/")

    data = read_bag(bag_path)
    print("\nExtracting...")
    d = extract(data)

    print("\nGenerating plots...")
    plot_dashboard(d, OUTPUT_DIR)
    plot_position(d, OUTPUT_DIR)
    plot_convergence(d, OUTPUT_DIR)
    plot_fhat(d, OUTPUT_DIR)
    plot_what(d, OUTPUT_DIR)
    plot_xdot(d, OUTPUT_DIR)
    plot_cmdvel(d, OUTPUT_DIR)
    plot_rpy(d, OUTPUT_DIR)
    plot_imu(d, OUTPUT_DIR)
    plot_2d_trajectory(d, OUTPUT_DIR)
    plot_trajectory_on_surface_2d(d, OUTPUT_DIR)
    plot_3d_trajectory(d, OUTPUT_DIR)
    plot_3d_surface_trajectory(d, OUTPUT_DIR)
    plot_3d_what_convergence(d, OUTPUT_DIR)
    plot_3d_phase(d, OUTPUT_DIR)

    print(f"\nDone — {OUTPUT_DIR}/")
    print("\nPlots:")
    print("  00_dashboard.png              — full summary")
    print("  01_position_vs_time.png       — x,y,z actual vs setpoint")
    print("  02_convergence.png            — Z1, Z11 convergence")
    print("  03_f_hat_vs_time.png          — estimated function value")
    print("  04_w_hat_vs_time.png          — learned weights vs w_star")
    print("  05_x_dot_vs_time.png          — algorithm velocity")
    print("  06_cmd_vel_vs_time.png        — PID output")
    print("  07_roll_pitch_yaw.png         — orientation")
    print("  08_imu_data.png               — IMU acceleration + gyro")
    print("  09_2d_trajectory.png          — top view, colored by time")
    print("  10_trajectory_on_surface_2d.png — path on f(x,y) contour")
    print("  11_3d_trajectory.png          — 3D flight path")
    print("  12_3d_surface_trajectory.png  — path on f(x,y) 3D surface")
    print("  13_3d_what_convergence.png    — w_hat convergence in 3D")
    print("  14_3d_phase_portrait.png      — phase portrait (x,y,Z1)")

if __name__ == '__main__':
    main()
