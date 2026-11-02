# BottleIdentity ID from /P_current[0], independent of the internal operation ID.
std_msgs/Header header
int32 bottle_id
geometry_msgs/Point position
# Row-major 3x3 position covariance, in header.frame_id, in m^2.
float64[9] covariance
