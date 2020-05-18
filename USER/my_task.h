#ifndef MY_TASK_H
#define MY_TASK_H

#include "includes.h"
#include "sys.h"
#include "delay.h"

#include "RC_2006.h"
#include "RC_uart_dma.h"
#include "led.h"
#include "exchange.h"
#include "ball.h"

#ifdef TASK_GLOBAL
	#define TASK_EXT
#else
	#define TASK_EXT extern 
#endif

//�������ȼ�
#define CREATE_TASK_PRIO	3
#define CHECK_TASK_PRIO		4
#define LOOP_TASK_PRIO		5

//�����ջ��С	
#define CREATE_STK_SIZE 	128
#define CHECK_STK_SIZE 		2048
#define LOOP_STK_SIZE 		2048

	
//������ƿ�
TASK_EXT OS_TCB CreateTaskTCB;
TASK_EXT OS_TCB Check_TaskTCB;
TASK_EXT OS_TCB Loop_TaskTCB;


//�����ջ
TASK_EXT CPU_STK CREATE_TASK_STK[CREATE_STK_SIZE];
TASK_EXT CPU_STK CHECK_TASK_STK[CHECK_STK_SIZE];
TASK_EXT CPU_STK LOOP_TASK_STK[LOOP_STK_SIZE];

//����һ������  ���������
TASK_EXT OS_ERR Merr;

//�ź���
TASK_EXT OS_SEM	Move_SEM;


void Create_task(void *p_arg);
void Check_task(void *p_arg);
void Loop_task(void *p_arg);


#endif
