#include "my_task.h"

int main(void)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	
	delay_init(168);  	//ʱ�ӳ�ʼ��
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//�жϷ�������
	Uart_Init(USART1,DISABLE,USART_IT_RXNE,2,0);
	LED_Init();         //LED��ʼ��

	
	OSInit(&err);		//��ʼ��UCOSIII
	OS_CRITICAL_ENTER();//�����ٽ���
	//������ʼ����
	OSTaskCreate((OS_TCB 	* )&CreateTaskTCB,		//������ƿ�
				 (CPU_CHAR	* )"Create task", 		//��������
                 (OS_TASK_PTR )Create_task, 		//������
                 (void		* )0,					//���ݸ��������Ĳ���
                 (OS_PRIO	  )CREATE_TASK_PRIO,    //�������ȼ�
                 (CPU_STK   * )&CREATE_TASK_STK[0],	//�����ջ����ַ
                 (CPU_STK_SIZE)CREATE_STK_SIZE/10,	//�����ջ�����λ
                 (CPU_STK_SIZE)CREATE_STK_SIZE,		//�����ջ��С
                 (OS_MSG_QTY  )0,					//�����ڲ���Ϣ�����ܹ����յ������Ϣ��Ŀ,Ϊ0ʱ��ֹ������Ϣ
                 (OS_TICK	  )0,					//��ʹ��ʱ��Ƭ��תʱ��ʱ��Ƭ���ȣ�Ϊ0ʱΪĬ�ϳ��ȣ�
                 (void   	* )0,					//�û�����Ĵ洢��
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, //����ѡ��
                 (OS_ERR 	* )&err);				//��Ÿú�������ʱ�ķ���ֵ
	OS_CRITICAL_EXIT();	//�˳��ٽ���	 
	OSStart(&err);  //����UCOSIII
	while(1);
}

