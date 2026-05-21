"""
analyze_flight.py
-----------------
Reads a ROS2 bag file and generates flight analysis plots for
the Crazyflie BMO simulation.

Usage:
    python3 analyze_flight.py <path_to_bag_folder>

Example:
    python3 analyze_flight.py ~/rosbag2_2026_05_21
"""

import sys
import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import griddata

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except ImportError:
    print("ERROR: ROS2 Python libraries not found.")
    print("Source your workspace: source ~/bitcraze_ws/install/setup.bash")
    sys.exit(1)

TOPICS = {
    'state':    '/cf0/state',
    'setpoint': '/cf0/setpoint',
    'cmd_vel':  '/cf0/cmd_vel',
}
OUTPUT_DIR = 'flight_plots'

def read_bag(bag_path):
    print(f"\nReading bag: {bag_path}")
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}
    data = {key: [] for key in TOPICS}
    while reader.has_next():
        topic, raw, timestamp = reader.read_next()
        for key, topic_name in TOPICS.items():
            if topic == topic_name:
                msg_type = get_message(type_map[topic])
                msg = deserialize_message(raw, msg_type)
                data[key].append((timestamp * 1e-9, msg))
    for key in data:
        print(f"  {TOPICS[key]}: {len(data[key])} messages")
    return data

def extract_state(data):
    t, x, y, z = [], [], [], []
    vx, vy, vz = [], [], []
    roll, pitch, yaw = [], [], []
    ax, ay, az = [], [], []
    wx, wy, wz = [], [], []
    for ts, msg in data['state']:
        t.append(ts)
        x.append(msg.x);  y.append(msg.y);  z.append(msg.z)
        vx.append(msg.vx); vy.append(msg.vy); vz.append(msg.vz)
        roll.append(msg.roll); pitch.append(msg.pitch); yaw.append(msg.yaw)
        ax.append(msg.linear_acceleration.x)
        ay.append(msg.linear_acceleration.y)
        az.append(msg.linear_acceleration.z)
        wx.append(msg.angular_velocity.x)
        wy.append(msg.angular_velocity.y)
        wz.append(msg.angular_velocity.z)
    t0 = t[0] if t else 0
    return {
        't': np.array(t)-t0,
        'x': np.array(x), 'y': np.array(y), 'z': np.array(z),
        'vx': np.array(vx), 'vy': np.array(vy), 'vz': np.array(vz),
        'roll': np.array(roll), 'pitch': np.array(pitch), 'yaw': np.array(yaw),
        'ax': np.array(ax), 'ay': np.array(ay), 'az': np.array(az),
        'wx': np.array(wx), 'wy': np.array(wy), 'wz': np.array(wz),
    }

def extract_setpoint(data):
    t, x, y, z = [], [], [], []
    for ts, msg in data['setpoint']:
        t.append(ts)
        x.append(msg.pose.position.x)
        y.append(msg.pose.position.y)
        z.append(msg.pose.position.z)
    t0 = t[0] if t else 0
    return {'t': np.array(t)-t0, 'x': np.array(x), 'y': np.array(y), 'z': np.array(z)}

def extract_cmd_vel(data):
    t, lx, ly, lz, az = [], [], [], [], []
    for ts, msg in data['cmd_vel']:
        t.append(ts)
        lx.append(msg.linear.x); ly.append(msg.linear.y)
        lz.append(msg.linear.z); az.append(msg.angular.z)
    t0 = t[0] if t else 0
    return {'t': np.array(t)-t0, 'lx': np.array(lx), 'ly': np.array(ly), 'lz': np.array(lz), 'az': np.array(az)}

def plot_position(state, setpoint, out_dir):
    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    fig.suptitle('Position vs Time', fontsize=14, fontweight='bold')
    for i, (ax, s_val, sp_val, label) in enumerate(zip(
        axes,
        [state['x'], state['y'], state['z']],
        [setpoint['x'], setpoint['y'], setpoint['z']],
        ['X (m)', 'Y (m)', 'Z (m)']
    )):
        ax.plot(state['t'], s_val, 'b-', linewidth=1.5, label='Actual')
        ax.plot(setpoint['t'], sp_val, 'r--', linewidth=1.2, label='Setpoint')
        ax.set_ylabel(label); ax.legend(loc='upper right'); ax.grid(True, alpha=0.3)
    axes[-1].set_xlabel('Time (s)')
    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, '1_position_vs_time.png'), dpi=150)
    plt.close()
    print("  Saved: 1_position_vs_time.png")

def plot_error(state, setpoint, out_dir):
    sp_x = np.interp(state['t'], setpoint['t'], setpoint['x'])
    sp_y = np.interp(state['t'], setpoint['t'], setpoint['y'])
    sp_z = np.interp(state['t'], setpoint['t'], setpoint['z'])
    ex = sp_x - state['x']; ey = sp_y - state['y']; ez = sp_z - state['z']
    total = np.sqrt(ex**2 + ey**2 + ez**2)
    fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
    fig.suptitle('Position Error vs Time', fontsize=14, fontweight='bold')
    for ax, err, label, color in zip(axes, [ex, ey, ez, total],
        ['Error X (m)', 'Error Y (m)', 'Error Z (m)', 'Total Error (m)'],
        ['red', 'green', 'blue', 'purple']):
        ax.plot(state['t'], err, color=color, linewidth=1.5)
        ax.axhline(0, color='black', linewidth=0.8, linestyle='--')
        ax.set_ylabel(label); ax.grid(True, alpha=0.3)
    axes[-1].set_xlabel('Time (s)')
    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, '2_position_error.png'), dpi=150)
    plt.close()
    print("  Saved: 2_position_error.png")

def plot_velocity(state, out_dir):
    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    fig.suptitle('Velocity vs Time', fontsize=14, fontweight='bold')
    for ax, vel, label, color in zip(axes,
        [state['vx'], state['vy'], state['vz']],
        ['Vx (m/s)', 'Vy (m/s)', 'Vz (m/s)'],
        ['red', 'green', 'blue']):
        ax.plot(state['t'], vel, color=color, linewidth=1.5)
        ax.axhline(0, color='black', linewidth=0.8, linestyle='--')
        ax.set_ylabel(label); ax.grid(True, alpha=0.3)
    axes[-1].set_xlabel('Time (s)')
    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, '3_velocity_vs_time.png'), dpi=150)
    plt.close()
    print("  Saved: 3_velocity_vs_time.png")

def plot_cmd_vel(cmd, out_dir):
    fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
    fig.suptitle('PID Output (cmd_vel)', fontsize=14, fontweight='bold')
    for ax, val, label, color in zip(axes,
        [cmd['lx'], cmd['ly'], cmd['lz'], cmd['az']],
        ['linear.x (m/s)', 'linear.y (m/s)', 'linear.z (m/s)', 'angular.z (rad/s)'],
        ['red', 'green', 'blue', 'orange']):
        ax.plot(cmd['t'], val, color=color, linewidth=1.5)
        ax.axhline(0, color='black', linewidth=0.8, linestyle='--')
        ax.set_ylabel(label); ax.grid(True, alpha=0.3)
    axes[-1].set_xlabel('Time (s)')
    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, '4_pid_output.png'), dpi=150)
    plt.close()
    print("  Saved: 4_pid_output.png")

def plot_2d_trajectory(state, setpoint, out_dir):
    fig, ax = plt.subplots(figsize=(8, 8))
    fig.suptitle('2D Trajectory (Top View)', fontsize=14, fontweight='bold')
    ax.plot(state['x'], state['y'], 'b-', linewidth=1.5, label='Actual')
    ax.plot(setpoint['x'], setpoint['y'], 'r--', linewidth=1.2, label='Setpoint')
    ax.plot(state['x'][0], state['y'][0], 'go', markersize=10, label='Start')
    ax.plot(state['x'][-1], state['y'][-1], 'rs', markersize=10, label='End')
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
    ax.legend(); ax.grid(True, alpha=0.3); ax.set_aspect('equal')
    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, '5_2d_trajectory.png'), dpi=150)
    plt.close()
    print("  Saved: 5_2d_trajectory.png")

def plot_rpy(state, out_dir):
    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    fig.suptitle('Roll Pitch Yaw vs Time', fontsize=14, fontweight='bold')
    for ax, val, label, color in zip(axes,
        [np.degrees(state['roll']), np.degrees(state['pitch']), np.degrees(state['yaw'])],
        ['Roll (deg)', 'Pitch (deg)', 'Yaw (deg)'],
        ['red', 'green', 'blue']):
        ax.plot(state['t'], val, color=color, linewidth=1.5)
        ax.axhline(0, color='black', linewidth=0.8, linestyle='--')
        ax.set_ylabel(label); ax.grid(True, alpha=0.3)
    axes[-1].set_xlabel('Time (s)')
    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, '6_roll_pitch_yaw.png'), dpi=150)
    plt.close()
    print("  Saved: 6_roll_pitch_yaw.png")

def plot_imu(state, out_dir):
    fig, axes = plt.subplots(2, 3, figsize=(15, 8))
    fig.suptitle('IMU Data', fontsize=14, fontweight='bold')
    imu_data = [
        (state['wx'], 'ω X (rad/s)', 'red',    axes[0][0]),
        (state['wy'], 'ω Y (rad/s)', 'green',  axes[0][1]),
        (state['wz'], 'ω Z (rad/s)', 'blue',   axes[0][2]),
        (state['ax'], 'Acc X (m/s²)', 'red',   axes[1][0]),
        (state['ay'], 'Acc Y (m/s²)', 'green', axes[1][1]),
        (state['az'], 'Acc Z (m/s²)', 'blue',  axes[1][2]),
    ]
    for val, label, color, ax in imu_data:
        ax.plot(state['t'], val, color=color, linewidth=1.0)
        ax.set_title(label); ax.set_xlabel('Time (s)'); ax.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, '7_imu_data.png'), dpi=150)
    plt.close()
    print("  Saved: 7_imu_data.png")

def plot_heatmap(state, setpoint, out_dir):
    sp_x = np.interp(state['t'], setpoint['t'], setpoint['x'])
    sp_y = np.interp(state['t'], setpoint['t'], setpoint['y'])
    total_error = np.sqrt((sp_x - state['x'])**2 + (sp_y - state['y'])**2)
    xi = np.linspace(state['x'].min(), state['x'].max(), 100)
    yi = np.linspace(state['y'].min(), state['y'].max(), 100)
    xi, yi = np.meshgrid(xi, yi)
    try:
        zi = griddata((state['x'], state['y']), total_error, (xi, yi), method='linear')
        fig, ax = plt.subplots(figsize=(8, 7))
        fig.suptitle('Position Error Heatmap (XY Plane)', fontsize=14, fontweight='bold')
        heatmap = ax.contourf(xi, yi, zi, levels=20, cmap='RdYlGn_r')
        plt.colorbar(heatmap, ax=ax, label='Position Error (m)')
        ax.plot(state['x'], state['y'], 'b-', linewidth=0.8, alpha=0.5, label='Actual path')
        ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
        ax.legend(); ax.set_aspect('equal')
        plt.tight_layout()
        plt.savefig(os.path.join(out_dir, '8_error_heatmap.png'), dpi=150)
        plt.close()
        print("  Saved: 8_error_heatmap.png")
    except Exception as e:
        print(f"  Skipped heatmap: {e}")

def plot_3d_trajectory(state, setpoint, out_dir):
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    fig.suptitle('3D Trajectory', fontsize=14, fontweight='bold')
    ax.plot(state['x'], state['y'], state['z'], 'b-', linewidth=1.5, label='Actual')
    ax.plot(setpoint['x'], setpoint['y'], setpoint['z'], 'r--', linewidth=1.2, label='Setpoint')
    ax.scatter(state['x'][0], state['y'][0], state['z'][0], color='green', s=100, label='Start')
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
    ax.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, '9_3d_trajectory.png'), dpi=150)
    plt.close()
    print("  Saved: 9_3d_trajectory.png")

def plot_dashboard(state, setpoint, cmd, out_dir):
    fig = plt.figure(figsize=(16, 10))
    fig.suptitle('Flight Summary Dashboard', fontsize=16, fontweight='bold')
    gs = gridspec.GridSpec(3, 3, figure=fig)
    ax1 = fig.add_subplot(gs[0, :2])
    ax1.plot(state['t'], state['x'], 'r-', label='x')
    ax1.plot(state['t'], state['y'], 'g-', label='y')
    ax1.plot(state['t'], state['z'], 'b-', label='z')
    ax1.plot(setpoint['t'], setpoint['z'], 'b--', alpha=0.5, label='z_sp')
    ax1.set_title('Position'); ax1.set_ylabel('m')
    ax1.legend(loc='upper right'); ax1.grid(True, alpha=0.3)
    ax2 = fig.add_subplot(gs[0, 2])
    ax2.plot(state['x'], state['y'], 'b-', linewidth=1.2)
    ax2.set_title('Top View'); ax2.set_xlabel('x'); ax2.set_ylabel('y')
    ax2.set_aspect('equal'); ax2.grid(True, alpha=0.3)
    ax3 = fig.add_subplot(gs[1, :2])
    ax3.plot(state['t'], state['vx'], 'r-', label='vx')
    ax3.plot(state['t'], state['vy'], 'g-', label='vy')
    ax3.plot(state['t'], state['vz'], 'b-', label='vz')
    ax3.set_title('Velocity'); ax3.set_ylabel('m/s')
    ax3.legend(loc='upper right'); ax3.grid(True, alpha=0.3)
    ax4 = fig.add_subplot(gs[1, 2])
    ax4.plot(state['t'], np.degrees(state['roll']),  'r-', label='roll')
    ax4.plot(state['t'], np.degrees(state['pitch']), 'g-', label='pitch')
    ax4.plot(state['t'], np.degrees(state['yaw']),   'b-', label='yaw')
    ax4.set_title('Roll Pitch Yaw'); ax4.set_ylabel('deg')
    ax4.legend(loc='upper right'); ax4.grid(True, alpha=0.3)
    ax5 = fig.add_subplot(gs[2, :2])
    ax5.plot(cmd['t'], cmd['lx'], 'r-', label='cmd_x')
    ax5.plot(cmd['t'], cmd['ly'], 'g-', label='cmd_y')
    ax5.plot(cmd['t'], cmd['lz'], 'b-', label='cmd_z')
    ax5.set_title('PID Output'); ax5.set_ylabel('m/s'); ax5.set_xlabel('Time (s)')
    ax5.legend(loc='upper right'); ax5.grid(True, alpha=0.3)
    ax6 = fig.add_subplot(gs[2, 2])
    ax6.plot(state['t'], state['az'], 'b-', linewidth=0.8)
    ax6.axhline(9.81, color='r', linestyle='--', label='9.81')
    ax6.set_title('IMU Acc Z'); ax6.set_ylabel('m/s²'); ax6.set_xlabel('Time (s)')
    ax6.legend(); ax6.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, '0_dashboard.png'), dpi=150)
    plt.close()
    print("  Saved: 0_dashboard.png")

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 analyze_flight.py <path_to_bag_folder>")
        sys.exit(1)
    bag_path = sys.argv[1]
    if not os.path.exists(bag_path):
        print(f"ERROR: Bag path not found: {bag_path}")
        sys.exit(1)
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    print(f"Output directory: {OUTPUT_DIR}/")
    data     = read_bag(bag_path)
    if not data['state']:
        print("ERROR: No /cf0/state messages found.")
        sys.exit(1)
    print("\nExtracting data...")
    state    = extract_state(data)
    setpoint = extract_setpoint(data)
    cmd      = extract_cmd_vel(data)
    print("\nGenerating plots...")
    plot_dashboard(state, setpoint, cmd, OUTPUT_DIR)
    plot_position(state, setpoint, OUTPUT_DIR)
    plot_error(state, setpoint, OUTPUT_DIR)
    plot_velocity(state, OUTPUT_DIR)
    plot_cmd_vel(cmd, OUTPUT_DIR)
    plot_2d_trajectory(state, setpoint, OUTPUT_DIR)
    plot_rpy(state, OUTPUT_DIR)
    plot_imu(state, OUTPUT_DIR)
    plot_heatmap(state, setpoint, OUTPUT_DIR)
    plot_3d_trajectory(state, setpoint, OUTPUT_DIR)
    print(f"\nDone. All plots saved to ./{OUTPUT_DIR}/")

if __name__ == '__main__':
    main()
