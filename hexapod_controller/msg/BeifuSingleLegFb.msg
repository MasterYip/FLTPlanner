std_msgs/Header header
# 这一条腿是支撑态还是摆动态, 1是摆动态, 0是支撑态
uint8 legged_swing_state
# 这一条腿的三个关节的编码器的位置反馈
float64[3] joint_pos
# 这一条腿的三个关节的编码器的速度反馈(实际上是倍福自己差分计算的)
float64[3] joint_vel
# 这条腿上的足端力传感器在Z轴上的力反馈
float64 contact_force

