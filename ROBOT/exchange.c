#include "exchange.h"
#include "can.h"
#include "RC_elmo.h"
#include "timer.h"
#include "led.h"

RobotStatus_T RobotStatus;

void Robot_Init()
{
	/*�ϲ�ṹ״̬��ʼ��*/
	RobotStatus_Init();
	
	/*CAN��ʼ��*/
	CAN1_Init();
	CAN2_Init();
	Elmo_Init(CAN1, 2, 1);

	/*�����һ����ʱ��Ϊ�˱�֤elmo��ʼ���ɹ�*/
	Delay_ms(500);
	
	u8 hstemp[8] = {0};
	u8 secondhs = 0;
	
	/*����*/
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
	
	/*����,elmo,M2006������ⶨʱ����ʼ��*/
	HBCheck_TIM4_Init();
	
	/*�����������ͳ�ʼ����ʱ����ʼ��*/
	HBSelfCheck_TIM5_Init();
	
}

void RobotStatus_Init()
{
	/*����״̬ elmo״̬ 2006״̬�ĳ�ʼ��*/
	RobotStatus.Chassis_Status = false;
	RobotStatus.Elmo_Status = false;
	RobotStatus.M2006_Status = false;
	
	/*����״̬��ʼ��*/
	RobotStatus.First_Task_Status = true;
	RobotStatus.Second_Task_Status = true;
	RobotStatus.Third_Task_Status = true;
	RobotStatus.Fourth_Task_Status = true;
	
	
	/*����״̬��ʼ��*/
	RobotStatus.FirstHSstatus = false;
	RobotStatus.SecondHSstatus = false;
	RobotStatus.HSstatus = false;
	
	/*������������ʱ����300ms����TIM5�жϿ���*/
	RobotStatus.Robot_Status = true;
	
	/*��������,EC30,M2006������ʧ�������Լ�����*/
	RobotStatus.Chassis_Heartbeat = 0;
	RobotStatus.Elmo_Heartbeat = 0;
	RobotStatus.M2006_Heartbeat = 0;
	

	for(u8 i=0;i<8;i++)RobotStatus.Wheelset_Heartbeat[i] = 0X00;
	
	
}

void SelfCheckDeal()
{
	Elmo_Read_POS(0);									//30���ݲ�ѯ  ���ڲ���30�����Լ�
	
	if(RobotStatus.Robot_Status == true)
	{
		CAN2_Send_Msg(RobotStatus.Wheelset_Heartbeat,8,ROBOT_HB);	//�ýṹ�������һλ��Elmo״̬ �ڶ�λ��2006״̬
		RobotStatus.Robot_Status = false;
	}
	
	/*��������ϲ�ṹ��д�����ܷɴ������*/
	
	
	/*������������*/
	if(RobotStatus.Chassis_Status == true)
	{		
		LED0 = 1;
		/*ELMO����*/
		if(RobotStatus.Elmo_Status == false)
		{
			RobotStatus.Wheelset_Heartbeat[0] = 0X00;
			/*�����ϲ�ṹ���д���LED����*/
			LED1 = 0;
		}
		else 
		{	
			RobotStatus.Wheelset_Heartbeat[0] = 0XFF;
			/*�����ϲ�ṹ���д���LED����*/
			LED1 = 1;
		}		
		if(RobotStatus.M2006_Status == false)
		{
			RobotStatus.Wheelset_Heartbeat[1] = 0X00;
			/*�����ϲ�ṹ���д���LED����*/
			LED2 = 0;
		}
		else
		{
			RobotStatus.Wheelset_Heartbeat[1] = 0XFF;
			/*�����ϲ�ṹ���д���LED����*/
			LED2 = 1;
		}
		if((RobotStatus.M2006_Status == true)&&(RobotStatus.Elmo_Status == true))
			/*�����ϲ�ṹ���д���LED����*/;
			
						
	}
	else 
	{
		LED0 = 0;
		LED1 = 1;
		LED2 = 1;
	}
	/*�����ϲ�ṹ���д���LED����*/;



}











