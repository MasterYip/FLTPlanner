# A1 Robot State (4 legs instead of 6)
geometry_msgs/Pose base_Pose_Now
geometry_msgs/Pose base_Pose_Next

# Foot positions (4 legs: FR, FL, RR, RL)
legged_traj_plan/A1FeetPosition feetPositionNow
legged_traj_plan/A1FeetPosition feetPositionNext

# Support state (4 legs)
bool[4] support_State_Now
bool[4] support_State_Next

# Fault state (4 legs)
int32[4] faultLeg_State_Now
int32[4] faultLeg_State_Next

# Move direction
geometry_msgs/Vector3 move_Direction