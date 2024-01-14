#当前机器人状态
fast_legged_planner/hexapod_Base_Pose base_Pose_Now
int8[6] support_State_Now
int8[6] faultLeg_State_Now
fast_legged_planner/FeetPosition feetPositionNow

#下一步机器人状态 (FIXME: This is not used)
fast_legged_planner/hexapod_Base_Pose base_Pose_Next
int8[6] support_State_Next
int8[6] faultLeg_State_Next
fast_legged_planner/FeetPosition feetPositionNext

#移动方向
geometry_msgs/Point move_Direction

std_msgs/String remarks

fast_legged_planner/FeetNormalVector feetNormalVector

float64[6] mu

float64[6] maxNormalF