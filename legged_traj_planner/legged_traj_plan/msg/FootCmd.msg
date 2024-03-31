std_msgs/Header header
# 0: foot_effort, 1: joint_torque
bool feedforward_type
geometry_msgs/Point[] foot_position
geometry_msgs/Vector3[] foot_velocity
geometry_msgs/Vector3[] foot_effort
geometry_msgs/Vector3[] joint_kp
geometry_msgs/Vector3[] joint_kd
geometry_msgs/Vector3[] joint_torque
