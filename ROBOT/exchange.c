#include "exchange.h"
#include "can.h"
#include "RC_elmo.h"
#include "timer.h"
#include "led.h"

RobotStatus_T RobotStatus;

void Robot_Init()
{
	/*上层结构状态初始化*/
	RobotStatus_Init();
	
	/*CAN初始化*/
	CAN1_Init();
	CAN2_Init();
	Elmo_Init(CAN1, 2, 1);

	/*这里加一个延时是为了保证elmo初始化成功*/
	Delay_ms(500);
	
	u8 hstemp[8] = {0};
	u8 secondhs = 0;
	
	/*握手*/
	while(1)
	{
		if(RobotStatus.FirstHSstatus==true)
		{
			hstemp[0] = 0xff;
			CAN2_Send_Msg(hstemp,8,ROBOT_ONLINE);
		}
		if(RobotStatus.SecondHSstatus==true)
		{
			hstemp[1] = 0xff;
			CAN2_Send_Msg(hstemp,8,ROBOT_ONLINE);
			if(secondhs==0)
			{
				EndHSCheck_TIM6_Init();
				secondhs = 1;
			}
		}
		if(RobotStatus.HSstatus==true)
		{
			TIM_ITConfig(TIM6, TIM_IT_Update, DISABLE);
			break;
		}
		Delay_ms(1);
	}
	
	/*底盘,elmo,M2006心跳检测定时器初始化*/
	HBCheck_TIM4_Init();
	
	/*轮组心跳发送初始化定时器初始化*/
	HBSelfCheck_TIM5_Init();
	
}

void RobotStatus_Init()
{
	/*底盘状态 elmo状态 2006状态的初始化*/
	RobotStatus.Chassis_Status = false;
	RobotStatus.Elmo_Status = false;
	RobotStatus.M2006_Status = false;
	
	/*任务状态初始化*/
	RobotStatus.First_Task_Status = true;
	RobotStatus.Second_Task_Status = true;
	RobotStatus.Third_Task_Status = true;
	RobotStatus.Fourth_Task_Status = true;
	
	
	/*握手状态初始化*/
	RobotStatus.FirstHSstatus = false;
	RobotStatus.SecondHSstatus = false;
	RobotStatus.HSstatus = false;
	
	/*轮组心跳包定时发送300ms，由TIM5中断控制*/
	RobotStatus.Robot_Status = true;
	
	/*底盘主控,EC30,M2006心跳丢失次数，以及报文*/
	RobotStatus.Chassis_Heartbeat = 0;
	RobotStatus.Elmo_Heartbeat = 0;
	RobotStatus.M2006_Heartbeat = 0;
	

	for(u8 i=0;i<8;i++)RobotStatus.Wheelset_Heartbeat[i] = 0X00;
	
	
}

void SelfCheckDeal()
{
	Elmo_Read_POS(0);									//30数据查询  用于产生30心跳自检
	
	if(RobotStatus.Robot_Status == true)
	{
		CAN2_Send_Msg(RobotStatus.Wheelset_Heartbeat,8,ROBOT_HB);	//该结构体里面第一位是Elmo状态 第二位是2006状态
		RobotStatus.Robot_Status = false;
	}
	
	/*这里根据上层结构编写任务跑飞处理代码*/
	
	
	/*底盘主控心跳*/
	if(RobotStatus.Chassis_Status == true)
	{		
		LED0 = 1;
		/*ELMO心跳*/
		if(RobotStatus.Elmo_Status == false)
		{
			RobotStatus.Wheelset_Heartbeat[0] = 0X00;
			/*根据上册结构板编写相关LED代码*/
			LED1 = 0;
		}
		else 
		{	
			RobotStatus.Wheelset_Heartbeat[0] = 0XFF;
			/*根据上册结构板编写相关LED代码*/
			LED1 = 1;
		}		
		if(RobotStatus.M2006_Status == false)
		{
			RobotStatus.Wheelset_Heartbeat[1] = 0X00;
			/*根据上册结构板编写相关LED代码*/
			LED2 = 0;
		}
		else
		{
			RobotStatus.Wheelset_Heartbeat[1] = 0XFF;
			/*根据上册结构板编写相关LED代码*/
			LED2 = 1;
		}
		if((RobotStatus.M2006_Status == true)&&(RobotStatus.Elmo_Status == true))
			/*根据上册结构板编写相关LED代码*/;
			
						
	}
	else 
	{
		LED0 = 0;
		LED1 = 1;
		LED2 = 1;
	}
	/*根据上册结构板编写相关LED代码*/;



}











