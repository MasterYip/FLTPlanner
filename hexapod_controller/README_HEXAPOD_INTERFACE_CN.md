# Hexapod201 接口文档

本文档描述了 Hexapod201 接口的实现和使用，用于步态控制和机体运动。

## 概述

Hexapod 接口提供了一个统一的接口，用于控制真实和模拟的六足机器人。它支持以下功能：

- **速度指令**：通过积分速度指令生成目标位姿
- **直接位姿指令**：直接移动到指定位姿
- **步态参数控制**：配置详细的运动参数
- **可视化**：在 RViz 中实时可视化
- **双模式操作**：支持真实（PLC）和模拟（仿真）接口

## 架构

### 基础接口 (`Hexapod201BaseInterface`)

定义所有六足机器人控制器的通用接口的抽象基类：

- **ROS 集成**：用于 ROS 通信的发布器、订阅器和定时器
- **速度积分**：将速度指令转换为目标位姿
- **可视化**：六足机器人机体的 RViz 标记可视化
- **状态管理**：跟踪当前位姿和目标位姿

### 模拟接口 (`DummyHexapod201Interface`)

用于测试和开发的仿真接口：

- **阻塞运动**：模拟具有可配置速度的真实运动
- **线程执行**：非阻塞运动执行
- **插值**：平滑的位置和方向插值
- **可配置参数**：可调整的运动和旋转速度

### 真实接口 (`Hexapod201Interface`)

使用 PLC 通信的真实机器人接口：

- **PLC 连接**：建立与 Beckhoff PLC 的连接
- **参数管理**：读取和写入 PLC 参数
- **状态监控**：监控 PLC 状态和反馈
- **步态控制**：配置详细的步态参数

## 安装与设置

### 前置条件

- ROS Noetic（或兼容版本）
- Python 3
- 所需的 Python 包：
  - `pyads`（用于 PLC 通信）
  - `numpy`
  - `tf`（用于坐标变换）

### 安装

1. 将仓库克隆到你的 ROS 工作空间
2. 安装依赖项：
   ```bash
   pip install pyads numpy
   ```
3. 构建工作空间：
   ```bash
   catkin_make
   source devel/setup.bash
   ```

## 使用方法

### 运行演示

#### 使用 Launch 文件（推荐）

```bash
# 使用模拟接口（仿真）
roslaunch hexapod_controller hexapod_demo.launch use_dummy:=true

# 使用真实六足机器人（需要 PLC 连接）
roslaunch hexapod_controller hexapod_demo.launch use_dummy:=false plc_ip:=5.157.100.214.1.1

# 运行特定的演示类型
roslaunch hexapod_controller hexapod_demo.launch demo_type:=velocity
```

#### 直接运行 Python 脚本

```bash
# 仅运行 Hexapod 接口
rosrun hexapod_controller hexapod201_interface.py --dummy

# 使用特定类型运行演示
rosrun hexapod_controller hexapod_gait_demo.py --dummy --demo velocity
```

### 演示类型

1. **速度演示**：展示速度指令积分
   - 前进/后退运动
   - 左转/右转
   - 横向运动

2. **位姿演示**：展示直接位姿指令
   - 移动到特定位置
   - 旋转到特定方向
   - 复杂的运动序列

3. **圆形演示**：展示连续运动
   - 圆形路径跟随
   - 动态方向控制

4. **步态参数演示**：展示步态配置（仅限真实六足机器人）
   - 不同的步态模式
   - 参数调节

5. **交互式演示**：交互式指令界面
   - 用户控制的演示
   - 实时状态监控

### 命令行参数

#### hexapod201_interface.py
- `--dummy`：使用模拟接口进行仿真
- `--plc_ip`：PLC 的 IP 地址（默认：5.157.100.214.1.1）
- `--node_name`：ROS 节点名称（默认：hexapod201_interface）

#### hexapod_gait_demo.py
- `--dummy`：使用模拟接口进行仿真
- `--plc_ip`：PLC 的 IP 地址
- `--demo`：演示类型（velocity, pose, circle, gait, interactive）

## ROS 话题

### 订阅的话题

- `/cmd_vel` (geometry_msgs/Twist)：速度指令
- `/hexapod/pose_cmd` (geometry_msgs/PoseStamped)：直接位姿指令

### 发布的话题

- `/hexapod/current_pose` (geometry_msgs/PoseStamped)：当前六足机器人位姿
- `/hexapod_visualization` (visualization_msgs/MarkerArray)：可视化标记

## API 参考

### Hexapod201BaseInterface

#### 方法

```python
def move_to_pose(self, target_pose: Pose) -> bool:
    """移动六足机器人到目标位姿 - 抽象方法"""

def setCmd(self, **kwargs) -> bool:
    """设置详细的指令参数 - 抽象方法"""

def stop_movement(self) -> bool:
    """停止当前运动 - 抽象方法"""

def reset_integration(self):
    """重置速度积分"""

def get_current_pose(self) -> Pose:
    """获取当前位姿"""

def get_target_pose(self) -> Pose:
    """获取目标位姿"""
```

### DummyHexapod201Interface

#### 构造函数

```python
def __init__(self, node_name: str = "dummy_hexapod201_interface"):
    """初始化模拟接口"""
```

#### 配置

```python
def setCmd(self, **kwargs) -> bool:
    """设置模拟接口参数
    
    参数:
        movement_speed: 运动速度（单位：m/s）
        rotation_speed: 旋转速度（单位：rad/s）
    """
```

### Hexapod201Interface

#### 构造函数

```python
def __init__(self, node_name: str = "hexapod201_interface", plc_ip: str = "5.157.100.214.1.1"):
    """初始化真实六足机器人接口"""
```

#### PLC 参数

```python
def setCmd(self, **kwargs) -> bool:
    """设置详细的运动参数
    
    参数:
        TA: 加速时间（单位：秒）
        TM: 运动时间（单位：秒）
        TD: 减速重叠时间（单位：秒）
        TZ: Z 轴提前时间（单位：秒）
        GaitMode: 步态模式（1=同步，2=三足，3=波浪）
        GaitDF: 占空比（0.5-0.75）
        SwapHigh: 摆动高度（单位：毫米）
        ForceMode: 力控制模式
    """
```

## 可视化

接口在 RViz 中提供实时可视化：

- **六足机器人机体**：以当前位姿显示为一个立方体
- **运动路径**：显示运动轨迹
- **标记**：用于调试的各种可视化标记

### RViz 配置

演示包含一个预配置的 RViz 设置，包含：
- 网格显示
- 标记数组可视化
- 位姿显示
- TF 框架可视化

## PLC 通信

### 连接参数

- **IP 地址**：5.157.100.214.1.1（可配置）
- **端口**：851（默认 pyads 端口）
- **符号**：访问 PLC 变量以进行控制和反馈

### PLC 变量

- `MAIN.PTCmd.TM`：时间参数结构
- `MAIN.PTCmd.Gait`：步态参数结构
- `MAIN.PTCmd.Pose`：位姿参数结构
- `MAIN.CtrlCmd`：控制指令
- `MAIN.state`：状态控制
- `MAIN.Q_State`：状态反馈
- `MAIN.PTActPos`：实际位置反馈

## 错误处理

接口包含全面的错误处理：

- **PLC 连接**：自动重试和回退
- **运动执行**：超时和错误恢复
- **参数验证**：输入验证和范围检查
- **ROS 通信**：话题可用性检查

## 故障排除

### 常见问题

1. **PLC 连接失败**
   - 检查网络连接
   - 验证 PLC IP 地址
   - 确保 PLC 正在运行且可访问

2. **运动未执行**
   - 检查六足机器人是否已启用
   - 验证参数值是否在范围内
   - 监控 ROS 话题以获取指令

3. **可视化未工作**
   - 确保 RViz 正在运行
   - 检查标记话题是否正在发布
   - 验证框架变换

### 调试

启用调试日志：
```bash
export ROS_LOG_LEVEL=DEBUG
```

监控话题：
```bash
rostopic echo /hexapod/current_pose
rostopic echo /cmd_vel
```

## 开发

### 添加新功能

1. **新运动类型**：扩展基础接口
2. **附加参数**：添加到 `setCmd` 方法
3. **自定义可视化**：扩展可视化系统

### 测试

- 使用模拟接口进行开发和测试
- 在受控环境中测试真实六足机器人
- 在部署前验证所有运动参数

## 许可证

此实现是六足机器人控制器包的一部分，并遵循相同的许可证条款。

## 贡献

在为此接口贡献代码时：

1. 遵循现有的代码风格
2. 添加全面的文档
3. 为新功能包含单元测试
4. 使用模拟和真实接口进行测试
5. 根据需要更新本文档