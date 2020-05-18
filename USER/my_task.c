#define TASK_GLOBAL

#include "my_task.h"


void Create_task(void *p_arg)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	p_arg = p_arg;

	CPU_Init();
#if OS_CFG_STAT_TASK_EN > 0u
   OSStatTaskCPUUsageInit(&err);  	//统计任务                
#endif
	
#ifdef CPU_CFG_INT_DIS_MEAS_EN		//如果使能了测量中断关闭时间
    CPU_IntDisMeasMaxCurReset();	
#endif
	
#if	OS_CFG_SCHED_ROUND_ROBIN_EN  //当使用时间片轮转的时候
	 //使能时间片轮转调度功能,时间片长度为1个系统时钟节拍，既1*5=5ms
	OSSchedRoundRobinCfg(DEF_ENABLED,1,&err);  
#endif	
	
	OS_CRITICAL_ENTER();	//进入临界区
	//创建CHECK任务
	OSTaskCreate((OS_TCB 	* )&Check_TaskTCB,		
				 (CPU_CHAR	* )"Check task", 		
                 (OS_TASK_PTR )Check_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )CHECK_TASK_PRIO,     
                 (CPU_STK   * )&CHECK_TASK_STK[0],	
                 (CPU_STK_SIZE)CHECK_STK_SIZE/10,	
                 (CPU_STK_SIZE)CHECK_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,					
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
                 (OS_ERR 	* )&err);
	
	//创建MOVE任务
	OSTaskCreate((OS_TCB 	* )&Loop_TaskTCB,		
				 (CPU_CHAR	* )"Loop task", 		
                 (OS_TASK_PTR )Loop_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )LOOP_TASK_PRIO,     
                 (CPU_STK   * )&LOOP_TASK_STK[0],	
                 (CPU_STK_SIZE)LOOP_STK_SIZE/10,	
                 (CPU_STK_SIZE)LOOP_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,					
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
                 (OS_ERR 	* )&err);				
				 
	//创建Move任务开始信号量
	OSSemCreate((OS_SEM		*)&Move_SEM,
				(CPU_CHAR* )"MOVE_SEM", 
				(OS_SEM_CTR )1, 
				(OS_ERR*)&err);	
				 
	OS_CRITICAL_EXIT();	//退出临界区
	OSTaskDel((OS_TCB*)0,&err);	//删除Create_task任务自身
}

void Check_task(void *p_arg)
{
	OSSemPend(&Move_SEM,0,OS_OPT_PEND_BLOCKING,0,&Merr); 
	
	/*下面的这个初始化函数包括结构体的初始化
	以及握手循环如果单独调试将循环部分注释掉*/
	Robot_Init();

	OSSemPost (&Move_SEM,OS_OPT_POST_1,&Merr);
	while(1)
	{
		SelfCheckDeal();
		delay_ms(10);
	}


}


void Loop_task(void *p_arg)
{
	OSSemPend(&Move_SEM,0,OS_OPT_PEND_BLOCKING,0,&Merr); 
	
		while(1)
		{
			/*根据上层结构来写*/
			delay_ms(5);
		}


}






