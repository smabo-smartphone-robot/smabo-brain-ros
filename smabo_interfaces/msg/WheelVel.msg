# Raw differential-drive wheel velocities from smabo-esp32.
# smabo-brain-ros integrates these into nav_msgs/Odometry (see odom_node).
std_msgs/Header header
float64 left    # left wheel linear speed (m/s)
float64 right   # right wheel linear speed (m/s)
float64 dt      # integration interval (s)
