//******************* Copyriht (C), 2016-2020, HIT *********************
// file name:    RCSDataDef.h
// Author:       
// Version:      V1.0.0
// Data:         2015/12/15
// Description:  数据结构定义
// History:      
// <author>       <time>        <version>      <desc>
//  xcf         2015/12/15        1.0.0         新建文档
//**********************************************************************

#ifndef __RCSDATADEF_H
#define __RCSDATADEF_H

#define VXSIM 		1		//编译为仿真程序
#define HEXAGON		0		//六边形构型

#define PI			3.14159265359
#define TM_ERROR	0.0002f	
#define FIFO_SIZE	2

//状态宏定义
#define CS_CNT   	6		//足的个数
//#define CS_SP		0		//处于支撑相
//#define CS_SW		1		//处于摆动相
//#define CS_PreSP  2		//准备支撑相
//#define CS_PreSW  3		//准备摆动相
#define CS_UNION    0		//联合运动
#define CS_SINGLE   1		//独立运动
#define BEING_SP	0
#define BEING_SW	1
#define INTO_SP		2
#define INTO_SW		3

#define PT_GAIT_WAVE    0	//波动步态
#define PT_GAIT_SPHASE  1	//同相位步态--原2-3-6步态
#define PT_GAIT_EPHASE  2	//等相位步态--占空比大于0.5
#define PT_GAIT_FTL   3		//前足跟踪步态
#define PT_GAIT_PREC  4		//精准落足步态
#define PT_GAIT_FREE  5		//自由设置占空比和相位步态
#define PT_GAIT_OBSTC 8		//障碍运动步态
#define PT_GAIT_DITCH 9		//过沟运动步态
#define PT_GAIT_POSE  10	//位姿运动步态
#define PT_PHASE_SUM  60	//相位基底(相位映射到0-59之间)

#define PT_FIFO_SIZE 32     //平台指令缓冲区大小
#define PT_MOV_STP  0		//指令运动完停止
#define PT_MOV_BLD  1		//指令运动混合
#define PT_MOV_REF  0		//弹性运动指令(自动调整步态指令大小)reference
#define PT_MOV_RGD  1		//硬性运动指令 (不做更改)rigig
#define PT_MOV_TM   0		//运动使用TM计算
#define PT_MOV_FR   1		//运动使用FR计算
#define PT_STATE_STOP  0	//平台运动状态为停车
#define PT_STATE_ACC   1	//平台运动状态为加速
#define PT_STATE_SPD   2	//平台运动状态为匀速
#define PT_STATE_DEC   3	//平台运动状态为减速
#define PT_STATE_HLD   4	//平台运动状态为保持
#define PT_STATE_S2N   5	//平台运动状态为STP转下一指令
#define PT_STATE_B2N   6	//平台运动状态为BLD转下一指令
#define PT_MOVCTR_STOP  0	//平台控制状态为停车
#define PT_MOVCTR_START 1	//平台控制状态为运动
#if HEXAGON
	#define PT_CYCLE_TIME  0.002 //细分时间为0.002秒
#else
	#define PT_CYCLE_TIME  0.001 //细分时间为0.001秒
	extern float LenX;			//车体坐标系距腿部指标系X方向绝对距离
	extern float LenY;			//车体坐标系距腿部指标系Y方向绝对距离
	extern float LCSY;			//足端距腿部指标系Y方向绝对距离
#endif 
	extern float LCSZ;			//足端距腿部指标系Z方向绝对距离
	extern float MAX_X;
	extern float MIN_X;
	extern float MAX_Y;
	extern float MIN_Y;
	extern float MAX_Z;
	extern float MIN_Z;
	extern float MAX_RAD;
	extern float MIN_RAD;
	extern float MAX_RPY_RAD;
extern double	G_BASIC_TIME;//动态时间基准

typedef union ud32
{
	BYTE  ch[4];
	WORD  wData[2];
	int	  sData;
	float fData;
}ud32;

//足端数据结构定义
typedef struct stXYZ
{
	float X;		//X,Y,Z位置
	float Y;
	float Z;
	int   SF;		//CS_SP(0)-支撑，CS_SW(1)-摆动
}stXYZ;
//足端数据结构定义
typedef struct stRPY
{
	float Roll;		//滚转角(弧度)Rot(X,Roll)
	float Pitch;	//俯仰角(弧度)Rot(Y,Pitch)
	float Yaw;		//偏航角(弧度)Rot(Z,Yaw)
	int   FG;		//PT_MOV_STP-0，PT_MOV_BLD-1
}stRPY;
//位姿数据结构定义
typedef struct stPose
{
	float X;		//X,Y,Z位置/命令
	float Y;
	float Z;
	float Roll;		//滚转角(弧度)Rot(X,Roll)
	float Pitch;	//俯仰角(弧度)Rot(Y,Pitch)
	float Yaw;		//偏航角(弧度)Rot(Z,Yaw)
	int   FG;		//PT_MOV_STP-0，PT_MOV_BLD-1(cmd中相当于连续运动)
	int   Res;		//保留(cmd中PT_MOV_REF为弹性指令,PT_MOV_RGD为硬性指令)
	stPose& stPose::operator+=(const stPose& stCmd);
}stPose;

//运动平台状态数据结构定义
typedef struct stPT
{
	stPose CG;			//center of gravity中心位姿
	stXYZ CS[CS_CNT];	//平台坐标系下六足位置
}stPT;

//运动属性数据结构定义
typedef struct stTime
{
//	float TS;			//S曲线加速时间s,默认TA的一半
	float TA;			//加速时间s
	float TM;			//匀速运动时间s
	float TD;			//运动开始前和结束后延时(delay)
	float TZ;			//摆动相Z提前XY运动时间s  连续运动时要求 2*TD+2*TZ<=0.5*TM
//	float FR;			//进给速度mm/s
//	int   FG;		//FLAG使用TM/FR标志位: PT_MOV_TM-TM, PT_MOV_FR-FR
}stTime;
//运动属性数据结构定义（会自动整定到整数）
typedef struct stGait
{
	UINT  GaitMode;			//步态模式
	float GaitDF;			//步态占地系数
	float SwapHigh;			//摆动高度(mm)
	UINT  LegNum;			//当前不用,地形识别时候用,单腿运动腿号0-5	10-15指定腿摆动
	int   ForceMode;		//力控模式 0-无 bit1-X方向 bit2-Y方向 bit3-Z方向
	int   Res;
	//UINT  JointNum;			//单腿运动关节1-3
}stGait;

typedef struct stCmd
{
	stTime Time;
	stGait Gait;
	stPT   Pos;
}stCmd;



typedef enum enum_CmdType
{
	IDLE=0,
	SET_PTPOSE=1,	//运动中发送该命令会停止
	MODAL_MOV=2,
	FORCE_MOV=3,
	BACK_MOV =4,

	FOLLOW_MOV=6,
	WHEEL2LEG=7,
	LEG2WHEEL=8,
	WAIT_TRIG=9,
	EXAMPLE_MOV=10,	//
	POSE_MOV=11,
	STEP_MOV=12,
	TRACK_MOV=13,
	ONLINE_MOV=14,
	LEGS_MOV=15,
	FORCE_STEP_MOV=16,
	FORCE_ONLINE_MOV=17,
	STOP_MOV=18,
	PARK_MOV=19,
	Ditch_MOV=20,
	OBSTC_MOV=21,
	SLOPE1_MOV=22,
	SLOPE2_MOV=23,
	REMOTE_MOV=30	//远程控制,需要指定自由步态

}enum_CmdType;


#endif 