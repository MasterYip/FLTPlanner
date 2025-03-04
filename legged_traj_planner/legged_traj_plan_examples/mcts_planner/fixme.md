- if(averageFootholdsHeight+USER::norminalTrunkHeight > maxZ+10000000) //////////////// 这里潜在问题
高度移动这里有问题，这里把1000000去掉，原理上是对的。 不过提前采用一个凸包通道提前给定机器人的位姿更好。

- 临时解决IK fault问题; the method does not support rotation.
if (result_.first == false)
{
    std::cout << "IK failed--------!" << std::endl;
}
else
{
    theta = result_.second(0) * (-1.0f);
}

- 机器人在odom负轴时，会出问题。 可能是patch tracker里计算距离没处理好

- 机器人沿着y轴前进时，机身不倾斜？

# max_depth 参数未被使用


# --- 关节扭矩
dynamic_contrain.cpp里         
// pinocchio::Data data = USER::HexMini_PinoModel_Data; /// 这里
改为了 pinocchio::Data data(model);
未经过验证