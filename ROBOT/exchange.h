#ifndef _EXCHANGE_H
#define _EXCHANGE_H

#include "includes.h"
#include "sys.h"
#include "delay.h"

#include "exchange.h"

//�ϲ�������
#define ROBOT_ID		(5)						//���������������

//����ָ���ID�� ��Щָ���������ӵ�ָ������ȫ��ͬ��
#define ROBOT_LANCH		(0x10+ROBOT_ID)					
#define ROBOT_LAYDOWN 	(0x20+ROBOT_ID)				

/*
#define WHEELSET_MOVEDATA_ID			(0X20+WheelsetID)
*/



//��������ID��	��Щָ���������ӵ�ָ����ͬ �ϲ�ṹ���൱�����������
#define ROBOT_HB		(0x30 +ROBOT_ID)		//�ϲ�ṹ������
#define ROBOT_INIT		(0x40 +ROBOT_ID)		//�ϲ�ṹ��ʼ��
#define RESET_CMD		(0x50 +ROBOT_ID)		//��λ	
#define CHASSIS_HB		(0x60 +ROBOT_ID)		//����������
#define ROBOT_ONLINE 	(0x70 +ROBOT_ID)		//�ϲ�ṹ����
#define CHASSIS_ONLINE	(0x700+ROBOT_ID)		//��������

/*
#define WHEELSET_HB_ID					(0X30+WheelsetID)
#define WHEELSET_INIT_CMD_ID			(0X40+WheelsetID)
#define RESETCMD_ID						(0X50+WheelsetID)
#define CHASSIS_HB_ID					(0x60+WheelsetID)
#define WHEELSET_ONLINE_ID				(0X70+WheelsetID)
#define CHASSIS_ONLINE_ID				(0X700+WheelsetID)
*/


//������

#define CANFILTERIDH					(0X0000|(ROBOT_ID<<5)) //CAN�����л��õ����

typedef struct
{
	/*���±�����u8 ������¼������*/
	u8 Chassis_Heartbeat;				//��������
	u8 Elmo_Heartbeat;					//ELMO����
	u8 M2006_Heartbeat;					//M2006����
	u8 Wheelset_Heartbeat[8];			//���鷢������CAN����(��һ���ֽ�ΪELMO����,�ڶ����ֽ�ΪM2006����)
	
	/*���±�����bool ������¼״̬*/
	bool Robot_Status;					//�ϲ�ṹ״̬
	bool Chassis_Status;				//��������������״̬
	bool Elmo_Status;					//ELMO״̬
	bool M2006_Status;					//M2006״̬

	
	bool FirstHSstatus;					//��һ������״̬
	bool SecondHSstatus;				//�ڶ�������״̬
	bool HSstatus;						//����״̬
	
	bool First_Task_Status;				//��һ���������״̬
	bool Second_Task_Status;			//�ڶ����������״̬
	bool Third_Task_Status;				//�������������״̬
	bool Fourth_Task_Status;			//���ĸ��������״̬

}RobotStatus_T;


extern RobotStatus_T RobotStatus;


void Robot_Init(void);
void RobotStatus_Init(void);
void SelfCheckDeal(void);


#endif

