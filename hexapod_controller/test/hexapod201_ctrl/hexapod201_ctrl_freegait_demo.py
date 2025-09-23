import pyads
import numpy as np
import logging
from typing import Dict, Any, Optional
from enum import IntEnum
import time

# 运动模式枚举类
class CtrlCmd(IntEnum):
    IDLE = 0
    SET_PTPOSE = 1
    MODAL_MOV = 2
    FORCE_MOV = 3
    BACK_MOV = 4
    FOLLOW_MOV = 6
    WHEEL2LEG = 7
    LEG2WHEEL = 8
    WAIT_TRIG = 9
    EXAMPLE_MOV = 10
    POSE_MOV = 11
    STEP_MOV = 12
    TRACK_MOV = 13
    ONLINE_MOV = 14
    LEGS_MOV = 15
    FORCE_STEP_MOV = 16
    FORCE_ONLINE_MOV = 17
    STOP_MOV = 18
    PARK_MOV = 19
    DITCH_MOV = 20
    OBSTC_MOV = 21
    SLOPE1_MOV = 22
    SLOPE2_MOV = 23
    REMOTE_MOV = 30

# 状态切换枚举类
class State(IntEnum):
    INIT = 0
    ENABLE = 1
    FEEDMOV = 2
    DISENABLE = 3
    SETPOSITION = 4
    RESET = 5
    ERROR = 6
    IDLE = 7

# 运动参数-时间结构定义
stTime_def = (
    ("TA", pyads.PLCTYPE_REAL, 1),
    ("TM", pyads.PLCTYPE_REAL, 1),
    ("TD", pyads.PLCTYPE_REAL, 1),
    ("TZ", pyads.PLCTYPE_REAL, 1),
)

# 运动参数-步态结构定义
stGait_def = (
    ("GaitMode", pyads.PLCTYPE_UDINT, 1),
    ("GaitDF", pyads.PLCTYPE_REAL, 1),
    ("SwapHigh", pyads.PLCTYPE_REAL, 1),
    ("LegNum", pyads.PLCTYPE_UDINT, 1),
    ("ForceMode", pyads.PLCTYPE_DINT, 1),
    ("Res", pyads.PLCTYPE_DINT, 1),
)

# 运动参数-步长/姿态/足端结构定义
stPose_def = (
    ("X", pyads.PLCTYPE_REAL, 1),
    ("Y", pyads.PLCTYPE_REAL, 1),
    ("Z", pyads.PLCTYPE_REAL, 1),
    ("Roll", pyads.PLCTYPE_REAL, 1),
    ("Pitch", pyads.PLCTYPE_REAL, 1),
    ("Yaw", pyads.PLCTYPE_REAL, 1),
    ("FG", pyads.PLCTYPE_DINT, 1),
    ("Res", pyads.PLCTYPE_DINT, 1),

    ("X1", pyads.PLCTYPE_REAL, 1),
    ("Y1", pyads.PLCTYPE_REAL, 1),
    ("Z1", pyads.PLCTYPE_REAL, 1),
    ("SF1", pyads.PLCTYPE_DINT, 1),

    ("X2", pyads.PLCTYPE_REAL, 1),
    ("Y2", pyads.PLCTYPE_REAL, 1),
    ("Z2", pyads.PLCTYPE_REAL, 1),
    ("SF2", pyads.PLCTYPE_DINT, 1),

    ("X3", pyads.PLCTYPE_REAL, 1),
    ("Y3", pyads.PLCTYPE_REAL, 1),
    ("Z3", pyads.PLCTYPE_REAL, 1),
    ("SF3", pyads.PLCTYPE_DINT, 1),

    ("X4", pyads.PLCTYPE_REAL, 1),
    ("Y4", pyads.PLCTYPE_REAL, 1),
    ("Z4", pyads.PLCTYPE_REAL, 1),
    ("SF4", pyads.PLCTYPE_DINT, 1),

    ("X5", pyads.PLCTYPE_REAL, 1),
    ("Y5", pyads.PLCTYPE_REAL, 1),
    ("Z5", pyads.PLCTYPE_REAL, 1),
    ("SF5", pyads.PLCTYPE_DINT, 1),

    ("X6", pyads.PLCTYPE_REAL, 1),
    ("Y6", pyads.PLCTYPE_REAL, 1),
    ("Z6", pyads.PLCTYPE_REAL, 1),
    ("SF6", pyads.PLCTYPE_DINT, 1),
)

stXYZ_def = (
    ("X", pyads.PLCTYPE_REAL, 1),
    ("Y", pyads.PLCTYPE_REAL, 1),
    ("Z", pyads.PLCTYPE_REAL, 1),
    ("SF", pyads.PLCTYPE_DINT, 1),
)

class RLController:


    def __init__(self, config: Dict[str, Any]):
        """
        初始化控制器
        :param config: 配置字典，包含：
        """
        self.config = config
        self._setup_logging()
        # 建立PLC连接
        self._connect_plc()
        # 计数
        self.count = 0


    def _setup_logging(self):
        """初始化日志系统"""
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(levelname)s - %(message)s'
        )

    def _connect_plc(self):
        """建立PLC连接"""
    #       plc_config = self.config['plc_config']
        try:
	#		添加路由

    #         CLIENT_NETID = "192.168.1.99.1.1"
    #         CLIENT_IP = "192.168.1.99"
    #         TARGET_IP = "192.168.1.115"
    #         TARGET_USERNAME = "Administrator"
    #         TARGET_PASSWORD = "1"
    #         ROUTE_NAME = "route-to-my-plc"
    #         pyads.add_route_to_plc(
    # CLIENT_NETID, CLIENT_IP, TARGET_IP, TARGET_USERNAME, TARGET_PASSWORD,
    # route_name=ROUTE_NAME
# )

            # 建立连接
            self.plc = pyads.Connection('127.0.0.1.1.1', pyads.PORT_TC3PLC1)
            self.plc.open()
            self.cpp = pyads.Connection('127.0.0.1.1.1', 351)
            self.cpp.open()

			# symbol_xxx 为和PLC读写操作变量
            # 运动控制参数读写操作符
            self.symbol_Cmd_Time = self.plc.get_symbol('MAIN.PTCmd.TM', structure_def=stTime_def )
            self.symbol_Cmd_Gait = self.plc.get_symbol('MAIN.PTCmd.Gait', structure_def=stGait_def )
            self.symbol_Cmd_Pose = self.plc.get_symbol('MAIN.PTCmd.Pose', structure_def=stPose_def )
            # 运动控制模式读写操作符
            self.symbol_CtrlCmd = self.plc.get_symbol('MAIN.CtrlCmd', plc_datatype = "UDINT")
            self.symbol_CtrlCmd.symbol_type = pyads.PLCTYPE_UDINT	
            self.symbol_CtrlCmd.plc_type = pyads.PLCTYPE_UDINT
            # 控制状态切换读写操作符
            self.symbol_State = self.plc.get_symbol('MAIN.state', plc_datatype = "UDINT")
            self.symbol_State.symbol_type = pyads.PLCTYPE_UDINT	
            self.symbol_State.plc_type = pyads.PLCTYPE_UDINT
            # 反馈变量读写操作符
            self.symbol_QState = self.plc.get_symbol('MAIN.Q_State')
            self.symbol_PTActPos = self.plc.get_symbol('MAIN.PTActPos', structure_def=stPose_def )
            # 设置反馈变量自动更新 
            self.symbol_QState.auto_update = True
            self.symbol_PTActPos.auto_update = True
            # 运动控制参数变量
            self.cmdTime = None
            self.cmdGait = None
            self.cmdPose = None

            # 和CPP通讯 - 主要用于自由控制步态和步长
            self.symbol_ReqPTCmd = self.cpp.get_symbol('CPP.Inputs.ReqPTCmd', structure_def=stPose_def) # 机体运动量和摆动腿机器人坐标系下位置 
            self.symbol_ReqFlag  = self.cpp.get_symbol('CPP.Inputs.ReqFlag')  # ReqFlag读为1时 下位机等待数据, 写为2时启动运动  
            self.symbol_ReqFlag.auto_update = True
             # 自由步态给定变量
            self.ReqPTCmd = None
         

            logging.info("PLC connection established")
        except Exception as e:
            logging.error(f"PLC connection failed: {str(e)}")
            raise

    def _read_plc_state(self) -> np.ndarray:
        """从PLC读取原始状态数据"""
        # self.ptcmd = self.symbol_PTCmd.read()
        # self.symbol_PTCmd.write(self.ptcmd)
        # print("PTCmd", self.ptcmd)
        # print("QState:", self.symbol_QState.value)
        # print("CtrlCmd:", self.symbol_CtrlCmd.read())
        # print("State:", self.symbol_State.read())
        # print("PTActPos", self.symbol_PTActPos.read())
        # print("PTActPos:", self.symbol_PTActPos.value)


    def run_control_loop(self) -> str:
        """主控制循环"""
        # self._read_plc_state()    # 先读一下数据   
        op_step = 0
        max_iterations = 1000
        try:
            while self.count < max_iterations:
                if op_step == 0:  # 使能
                    # self.symbol_State.write(State.ENABLE)
                    op_step = 1
                    logging.info("Enabled PLC")

                elif op_step == 100:  # 等待使能
                    if self.symbol_QState.value == State.FEEDMOV:
                        op_step = 1
                        logging.info("PLC in FEEDMOV state")

                elif op_step == 1:  # 设置运动参数

                    if self.cmdTime is None:
                        self.cmdTime = self.symbol_Cmd_Time.read()
                        logging.info(self.cmdTime)
                    if self.cmdTime is None:
                        logging.error("Failed to read PTCmd.TM")
                        break

                    if self.cmdGait is None:
                        self.cmdGait = self.symbol_Cmd_Gait.read()
                        logging.info(self.cmdGait)
                    if self.cmdGait is None:
                        logging.error("Failed to read PTCmd.Gait")
                        break


                    self.cmdTime["TA"] = 0.5          # 减速时间 TA (s)                                                                                      
                    self.cmdTime["TM"] = 1.5          # 摆动时间 TM (s)                                                                                      
                    self.cmdTime["TD"] = 0.0            # 支撑重叠时间 TA (s)                                                                                  
                    self.cmdTime["TZ"] = 0.0            # Z向提前时间 TZ (s)   
                    self.symbol_Cmd_Time.write(self.cmdTime)   					

                    self.cmdGait["GaitMode"] = 5          # 自由步态                                                                                    
                    self.cmdGait["GaitDF"  ] = 0.5        # 二步态1/2 三步态2/3 六步态5/6                                                                 
                    self.cmdGait["SwapHigh"] = 80.0         # 摆动高度 (mm)                                                                                
                    self.cmdGait["LegNum"  ] = 0			# 力控模式 0-无 bit0-X方向 bit1-Y方向 bit2-Z方向                                                
                    self.cmdGait["ForceMode"]= 0                                                                                                         
                    self.cmdGait["Res"] = 0     
                    self.symbol_Cmd_Gait.write(self.cmdGait)   

                    op_step = 2
                    logging.info("Motion parameters set")

                elif op_step == 2:  # 启动运动
                    # cmd = CtrlCmd(self.symbol_CtrlCmd.read())
                    # self.symbol_CtrlCmd.write(CtrlCmd.MODAL_MOV)	# 规则模式运动
                    self.symbol_CtrlCmd.write(CtrlCmd.REMOTE_MOV)	# 远端自由步态                    
                    op_step = 3
                    logging.info("Started modal movement")

                elif op_step == 3:  # 启动运动     

                    if self.ReqPTCmd is None:
                        self.ReqPTCmd = self.symbol_Cmd_Time.read()
                    if self.ReqPTCmd is None:
                        logging.error("Failed to read PTCmd.TM")
                        break

                    if self.symbol_ReqFlag.value == 1:

                        self.ReqPTCmd["X"] = 100.0         # TA+TM时间()机体前+后-向运动距离 (mm)                                                                
                        self.ReqPTCmd["Y"] = 0.0           # 机体左+右-横向运动距离 (mm)                                                                          
                        self.ReqPTCmd["Z"] = 0.0           # 机体上+下-运动距离 (mm)                                                                              
                        self.ReqPTCmd["Roll"] = 0.0        # 绕X轴运动弧度 (rad)                                                                                  
                        self.ReqPTCmd["Pitch"] = 0.0       # 绕Y轴运动弧度 (rad)                                                                                  
                        self.ReqPTCmd["Yaw"] = 0.0         # 绕Z轴运动弧度 (rad)                                                                                  
                        self.ReqPTCmd["FG"] = 0            # PLC自动设置  PT_MOV_STP-0(波浪运动-指令间减速停止-摆动时间为TA+TM),PT_MOV_BLD-1(持续运动-指令间不减速-摆动时间为TM)  
                        self.ReqPTCmd["Res"] = 0           # PLC自动设置  PT_MOV_REF-0为弹性指令(自动计算步长),PT_MOV_RGD-1为硬性指令(不调整步长)                                

                        self.ReqPTCmd["X1" ] = 660 + 50           # 机体坐标系下摆动的目标足端位置 (mm)                                                                 
                        self.ReqPTCmd["Y1" ] = 996.2              #                                                                       
                        self.ReqPTCmd["Z1" ] = -405               # 
                        self.ReqPTCmd["SF1"] = self.count % 2     # 0-支撑(xyz不用给数据), 1-摆动(xyz给摆动目标位置) 

                        self.ReqPTCmd["X2" ] = 0.0 + 50           # 机体坐标系下摆动的目标足端位置 (mm)                                                                 
                        self.ReqPTCmd["Y2" ] = 1251.2             #                                                                       
                        self.ReqPTCmd["Z2" ] = -405               # 
                        self.ReqPTCmd["SF2"] = (self.count+1) % 2 # 0-支撑(xyz不用给数据), 1-摆动(xyz给摆动目标位置) 

                        self.ReqPTCmd["X3" ] = -660 + 50          # 机体坐标系下摆动的目标足端位置 (mm)                                                                 
                        self.ReqPTCmd["Y3" ] = 996.2              #                                                                       
                        self.ReqPTCmd["Z3" ] = -405               # 
                        self.ReqPTCmd["SF3"] = self.count % 2     # 0-支撑(xyz不用给数据), 1-摆动(xyz给摆动目标位置) 

                        self.ReqPTCmd["X4" ] = 660 + 50           # 机体坐标系下摆动的目标足端位置 (mm)                                                                 
                        self.ReqPTCmd["Y4" ] = -996.2             #                                                                       
                        self.ReqPTCmd["Z4" ] = -405               # 
                        self.ReqPTCmd["SF4"] = (self.count+1) % 2 # 0-支撑(xyz不用给数据), 1-摆动(xyz给摆动目标位置) 

                        self.ReqPTCmd["X5" ] = 0.0 + 50           # 机体坐标系下摆动的目标足端位置 (mm)                                                                 
                        self.ReqPTCmd["Y5" ] = -1251.2            #                                                                       
                        self.ReqPTCmd["Z5" ] = -405               # 
                        self.ReqPTCmd["SF5"] = self.count % 2     # 0-支撑(xyz不用给数据), 1-摆动(xyz给摆动目标位置) 

                        self.ReqPTCmd["X6" ] = -660 + 50          # 机体坐标系下摆动的目标足端位置 (mm)                                                                 
                        self.ReqPTCmd["Y6" ] = -996.2             #                                                                       
                        self.ReqPTCmd["Z6" ] = -405               # 
                        self.ReqPTCmd["SF6"] = (self.count+1) % 2 # 0-支撑(xyz不用给数据), 1-摆动(xyz给摆动目标位置) 

                        self.symbol_ReqPTCmd.write(self.ReqPTCmd) # 设置自由步态参数  
                        self.symbol_ReqFlag.write(2)              # 设置完成,启动此步运动  

                        logging.info("New movement Req. ")

                    else: 
                        time.sleep(0.1)


                elif op_step == 8:  # 延时
                    time.sleep(6)
                    op_step = 9
                    logging.info("Delay completed")

                elif op_step == 9:  # 停止运动
                    self.symbol_CtrlCmd.write(CtrlCmd.STOP_MOV)	# 减速停止运动  
                    op_step = 10
                    logging.info("Stopped movement")

                elif op_step == 10:  # 等待运动结束并去使能
                    time.sleep(2)
                    self.symbol_State.write(State.DISENABLE)
                    op_step = 11
                    logging.info("Disabled PLC")
                    break

                else:
                    logging.warning(f"Unknown state: {op_step}")
                    break

                self._read_plc_state()
                self.count += 1
                logging.info(f"Control loop iteration: {self.count}")
                time.sleep(0.1)

        except KeyboardInterrupt:
            logging.info("Control loop stopped by user")
        finally:
            self.cleanup()
        return "Control loop terminated"

    def cleanup(self) -> None:
        """资源清理"""
        if hasattr(self, 'plc') and self.plc.is_open:
            self.plc.close()
            logging.info("PLC connection closed")

if __name__ == "__main__":
    config = {
        "plc_vars": {
            "joint佛_pos": "GVL.joint_positions",
            "joint_vel": "GVL.joint_velocities",
            "joint_targets": "GVL.joint_targets"
        },
        "control_params": {
            # "TA": 0.5,  # 减速时间 (s)
            # "TM": 1.5,  # 摆动时间 (s)
            # "delay": 6,  # 运动延时 (s)
            # "stop_delay": 2,  # 停止延时 (s)
            # "cycle_time": 0.1,  # 控制周期 (s)
            # "max_iterations": 1000  # 最大迭代次数
        }
    }

    controller = RLController(config)
    result = controller.run_control_loop()
    print(f"Control loop result: {result}")