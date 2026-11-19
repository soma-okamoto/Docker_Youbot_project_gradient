# Atomic prior and the two source hypotheses used for this Place operation.
std_msgs/Header header
uint32 operation_id
# -1 when bottle identity is disabled.
int32 bottle_id
geometry_msgs/Point position
float64[9] covariance
# Unmodified inputs, retained for diagnostics.
geometry_msgs/Point raw_current_position
geometry_msgs/Point raw_tf_position
# Bias-corrected source hypotheses, before prior CI.
geometry_msgs/Point current_position
float64[9] current_covariance
geometry_msgs/Point tf_position
float64[9] tf_covariance
