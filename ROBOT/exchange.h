#ifndef _EXCHANGE_H
#define _EXCHANGE_H

#include "includes.h"
#include "sys.h"
#include "delay.h"

#include "exchange.h"

//上层机构编号
#define ROBOT_ID		(5)						//把它当作五号轮组

//特殊指令的ID号 这些指令和轮组板子的指令是完全不同的
#define ROBOT_LANCH		(0x10+ROBOT_ID)					
#define ROBOT_LAYDOWN 	(0x20+ROBOT_ID)				

/*
#define WHEELSET_MOVEDATA_ID			(0X20+WheelsetID)
*/



//心跳包的ID号	这些指令和轮组板子的指令相同 上层结构板相当于是五号轮组
#define ROBOT_HB		(0x30 +ROBOT_ID)		//上层结构心跳包
#define ROBOT_INIT		(0x40 +ROBOT_ID)		//上层结构初始化
#define RESET_CMD		(0x50 +ROBOT_ID)		//复位	
#define CHASSIS_HB		(0x60 +ROBOT_ID)		//底盘心跳包
#define ROBOT_ONLINE 	(0x70 +ROBOT_ID)		//上层结构握手
#define CHASSIS_ONLINE	(0x700+ROBOT_ID)		//底盘握手

/*
#define WHEELSET_HB_ID					(0X30+WheelsetID)
#define WHEELSET_INIT_CMD_ID			(0X40+WheelsetID)
#define RESETCMD_ID						(0X50+WheelsetID)
#define CHASSIS_HB_ID					(0x60+WheelsetID)
#define WHEELSET_ONLINE_ID				(0X70+WheelsetID)
#define CHASSIS_ONLINE_ID				(0X700+WheelsetID)
*/


//过滤器

#define CANFILTERIDH					(0X0000|(ROBOT_ID<<5)) //CAN接收中会用到这个

typedef struct
{
	/*以下变量用u8 用来记录次数等*/
	u8 Chassis_Heartbeat;				//底盘心跳
	u8 Elmo_Heartbeat;					//ELMO心跳
	u8 M2006_Heartbeat;					//M2006心跳
	u8 Wheelset_Heartbeat[8];			//轮组发送心跳CAN报文(第一个字节为ELMO心跳,第二个字节为M2006心跳)
	
	/*以下变量用bool 用来记录状态*/
	bool Robot_Status;					//上层结构状态
	bool Chassis_Status;				//底盘主控心跳包状态
	bool Elmo_Status;					//ELMO状态
	bool M2006_Status;					//M2006状态

	
	bool FirstHSstatus;					//第一次握手状态
	bool SecondHSstatus;				//第二次握手状态
	bool HSstatus;						//握手状态
	
	bool First_Task_Status;				//第一个任务完成状态
	bool Second_Task_Status;			//第二个任务完成状态
	bool Third_Task_Status;				//第三个任务完成状态
	bool Fourth_Task_Status;			//第四个任务完成状态

}RobotStatus_T;


extern RobotStatus_T RobotStatus;


void Robot_Init(void);
void RobotStatus_Init(void);
void SelfCheckDeal(void);


#endif

