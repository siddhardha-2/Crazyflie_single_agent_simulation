# Header
std_msgs/Header header

# Position (meters)
float64 x
float64 y
float64 z

# Velocity (m/s)
float64 vx
float64 vy
float64 vz

# Orientation (quaternion)
geometry_msgs/Quaternion orientation

# Orientation (euler angles in radians)
float64 roll
float64 pitch
float64 yaw

# IMU data
geometry_msgs/Vector3 angular_velocity      # rad/s
geometry_msgs/Vector3 linear_acceleration   # m/s^2