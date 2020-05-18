#include "timer.h"
#include "led.h"
/*********************该文件用于定时器的中断以及中断服务函数**************










*************************************************************************/



/*定时器2 每1ms产生一次中断*/
void TIM2_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	
	TIM_TimeBaseInitStructure.TIM_Period = 84-1;
	TIM_TimeBaseInitStructure.TIM_Prescaler = 1000-1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE); 
	TIM_Cmd(TIM2,ENABLE); 
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0X01;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0X01;

	NVIC_Init(&NVIC_InitStructure);
}

/*定时器4 用于大主控 Elmo 以及2006的心跳检查*/
void HBCheck_TIM4_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
	
	TIM_TimeBaseInitStructure.TIM_Period = 8400-1;
	TIM_TimeBaseInitStructure.TIM_Prescaler = 5000-1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStructure);
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE); 
	TIM_Cmd(TIM4,ENABLE); 
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0X01;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0X01;

	NVIC_Init(&NVIC_InitStructure);
}

/*定时器5 自我心跳检测*/
void HBSelfCheck_TIM5_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);
	
	TIM_TimeBaseInitStructure.TIM_Period = 8400-1;
	TIM_TimeBaseInitStructure.TIM_Prescaler = 3000-1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	
	TIM_TimeBaseInit(TIM5,&TIM_TimeBaseInitStructure);
	TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE); 
	TIM_Cmd(TIM5,ENABLE); 
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStructure);
	
	TIM_ClearFlag(TIM5, TIM_FLAG_Update);

	TIM_Cmd(TIM5, ENABLE);
	TIM5->CNT = 0;
}

/*定时器6 握手结束检测*/
void EndHSCheck_TIM6_Init(void)
{
	TIM_TimeBaseInitTypeDef   TIM_TimeBaseStructure;
	NVIC_InitTypeDef          NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
	
	TIM_TimeBaseStructure.TIM_Period = 8400-1;
	TIM_TimeBaseStructure.TIM_Prescaler = 5000-1;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);
	TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM6_DAC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	TIM_ClearFlag(TIM6, TIM_FLAG_Update);

	TIM_Cmd(TIM6, ENABLE);	
	TIM6->CNT = 0;
}


/*以下为定时器中断的服务函数*/

void TIM2_IRQHandler(void)
{
	OSIntEnter();
	if(TIM_GetITStatus(TIM2,TIM_IT_Update)==SET)
		/*上层结构出了可以往里面填写*/;
	TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
	OSIntExit();
}


void TIM4_IRQHandler(void)
{
	OSIntEnter();
	if(TIM_GetITStatus(TIM4,TIM_IT_Update)==SET)
	{
		/*底盘主控心跳检测*/
		if(RobotStatus.Chassis_Status == true)RobotStatus.Chassis_Heartbeat++;
		if(RobotStatus.Chassis_Heartbeat >= 2)RobotStatus.Chassis_Status = false;
		
		/*M2006心跳检测*/
		if(RobotStatus.M2006_Status == true)RobotStatus.M2006_Heartbeat++;
		if(RobotStatus.M2006_Heartbeat >= 3)RobotStatus.M2006_Status = false;
		
		/*EC30心跳检测*/
		if(RobotStatus.Elmo_Status == true)RobotStatus.Elmo_Heartbeat++;
		if(RobotStatus.Elmo_Heartbeat >= 3)RobotStatus.Elmo_Status = false;
		
		/*根据上层结构进行调整*/
		
	}
	TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
	OSIntExit();
}

void TIM5_IRQHandler(void)
{
	OSIntEnter();
	if(TIM_GetITStatus(TIM5,TIM_IT_Update)==SET)RobotStatus.Robot_Status = true;
	TIM_ClearITPendingBit(TIM5,TIM_IT_Update);	
	OSIntExit();
}

void TIM6_DAC_IRQHandler(void)
{
	OSIntEnter();
	if(TIM_GetITStatus(TIM6,TIM_IT_Update)==SET)RobotStatus.HSstatus = true;
	TIM_ClearITPendingBit(TIM6,TIM_IT_Update);
	OSIntExit();
}

void wait(uint32_t n)
{
	do
	{
		n--;
	}while(n);
}
