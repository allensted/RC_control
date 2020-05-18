#define TASK_GLOBAL

#include "my_task.h"


void Create_task(void *p_arg)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	p_arg = p_arg;

	CPU_Init();
#if OS_CFG_STAT_TASK_EN > 0u
   OSStatTaskCPUUsageInit(&err);  	//ͳ������                
#endif
	
#ifdef CPU_CFG_INT_DIS_MEAS_EN		//���ʹ���˲����жϹر�ʱ��
    CPU_IntDisMeasMaxCurReset();	
#endif
	
#if	OS_CFG_SCHED_ROUND_ROBIN_EN  //��ʹ��ʱ��Ƭ��ת��ʱ��
	 //ʹ��ʱ��Ƭ��ת���ȹ���,ʱ��Ƭ����Ϊ1��ϵͳʱ�ӽ��ģ���1*5=5ms
	OSSchedRoundRobinCfg(DEF_ENABLED,1,&err);  
#endif	
	
	OS_CRITICAL_ENTER();	//�����ٽ���
	//����CHECK����
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
	
	//����MOVE����
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
				 
	//����Move����ʼ�ź���
	OSSemCreate((OS_SEM		*)&Move_SEM,
				(CPU_CHAR* )"MOVE_SEM", 
				(OS_SEM_CTR )1, 
				(OS_ERR*)&err);	
				 
	OS_CRITICAL_EXIT();	//�˳��ٽ���
	OSTaskDel((OS_TCB*)0,&err);	//ɾ��Create_task��������
}

void Check_task(void *p_arg)
{
	OSSemPend(&Move_SEM,0,OS_OPT_PEND_BLOCKING,0,&Merr); 
	
	/*����������ʼ�����������ṹ��ĳ�ʼ��
	�Լ�����ѭ������������Խ�ѭ������ע�͵�*/
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
			/*�����ϲ�ṹ��д*/
			delay_ms(5);
		}


}






