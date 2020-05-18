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

//任务优先级
#define CREATE_TASK_PRIO	3
#define CHECK_TASK_PRIO		4
#define LOOP_TASK_PRIO		5

//任务堆栈大小	
#define CREATE_STK_SIZE 	128
#define CHECK_STK_SIZE 		2048
#define LOOP_STK_SIZE 		2048

	
//任务控制块
TASK_EXT OS_TCB CreateTaskTCB;
TASK_EXT OS_TCB Check_TaskTCB;
TASK_EXT OS_TCB Loop_TaskTCB;


//任务堆栈
TASK_EXT CPU_STK CREATE_TASK_STK[CREATE_STK_SIZE];
TASK_EXT CPU_STK CHECK_TASK_STK[CHECK_STK_SIZE];
TASK_EXT CPU_STK LOOP_TASK_STK[LOOP_STK_SIZE];

//定义一个变量  保存错误码
TASK_EXT OS_ERR Merr;

//信号量
TASK_EXT OS_SEM	Move_SEM;


void Create_task(void *p_arg);
void Check_task(void *p_arg);
void Loop_task(void *p_arg);


#endif
