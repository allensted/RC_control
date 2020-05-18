#include "can.h"
#include "led.h"
#include "delay.h"
#include "RC_uart_dma.h"
#include "RC_2006.h"
#include "exchange.h"
#include "RC_elmo.h"

u8 CAN1_Init()
{

  	GPIO_InitTypeDef GPIO_InitStructure; 
	CAN_InitTypeDef        CAN_InitStructure;
  	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
#if CAN1_RX0_INT_ENABLE 
   	NVIC_InitTypeDef  NVIC_InitStructure;
#endif
    //使能相关时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能PORTA时钟	                   											 

  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//使能CAN1时钟	
	
    //初始化GPIO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8| GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化PB8,PB9
	
	//引脚复用映射配置
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource8,GPIO_AF_CAN1); //GPIOB8复用为CAN1
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource9,GPIO_AF_CAN1); //GPIOB9复用为CAN1
	  
  	//CAN单元设置
   	CAN_InitStructure.CAN_TTCM=DISABLE;	//非时间触发通信模式   
  	CAN_InitStructure.CAN_ABOM=DISABLE;	//软件自动离线管理	  
  	CAN_InitStructure.CAN_AWUM=DISABLE;//睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
  	CAN_InitStructure.CAN_NART=DISABLE;	//禁止报文自动传送 
  	CAN_InitStructure.CAN_RFLM=DISABLE;	//报文不锁定,新的覆盖旧的  
  	CAN_InitStructure.CAN_TXFP=ENABLE;	//优先级由报文标识符决定 
  	CAN_InitStructure.CAN_Mode= CAN_Mode_Normal;	 //模式设置 
  	CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;	//重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位 CAN_SJW_1tq~CAN_SJW_4tq
  	CAN_InitStructure.CAN_BS1=CAN_BS1_9tq; //Tbs1范围CAN_BS1_1tq ~CAN_BS1_16tq
  	CAN_InitStructure.CAN_BS2=CAN_BS2_4tq;//Tbs2范围CAN_BS2_1tq ~	CAN_BS2_8tq
  	CAN_InitStructure.CAN_Prescaler=3;  //分频系数(Fdiv)为brp+1	
  	CAN_Init(CAN1, &CAN_InitStructure);   // 初始化CAN1 
    
		//配置过滤器
 	CAN_FilterInitStructure.CAN_FilterNumber=14;	  //过滤器0
  	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 
  	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32位 
  	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;////32位ID
  	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
  	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32位MASK
  	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
   	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//过滤器0关联到FIFO0
  	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //激活过滤器0
  	CAN_FilterInit(&CAN_FilterInitStructure);//滤波器初始化
		
#if CAN1_RX0_INT_ENABLE
	
	  CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);//FIFO0消息挂号中断允许.		    
  
  	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // 主优先级为1
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // 次优先级为0
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
#endif
	return 0;
}   

void CAN2_Init(void)
{
	GPIO_InitTypeDef		GPIO_InitStructure;
	CAN_InitTypeDef			CAN_InitStructure;
	CAN_FilterInitTypeDef	CAN_FilterInitStructure;
	NVIC_InitTypeDef		NVIC_InitStructure;
	
	/*CAN2初始化*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);

	GPIO_InitStructure.GPIO_Pin		= GPIO_Pin_12 | GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AF;							//复用功能
	GPIO_InitStructure.GPIO_OType	= GPIO_OType_PP;						//推挽输出
	GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_100MHz;					//100MHz
	GPIO_InitStructure.GPIO_PuPd	= GPIO_PuPd_UP;							//上拉
	GPIO_Init(GPIOB, &GPIO_InitStructure);									//初始化PB12,PB13
	//引脚复用映射配置
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource12,GPIO_AF_CAN2);					//GPIOB12复用为CAN2
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource13,GPIO_AF_CAN2);					//GPIOB13复用为CAN2

	CAN_DeInit(CAN2);
	CAN_StructInit(&CAN_InitStructure);	
	//CAN单元设置
	CAN_InitStructure.CAN_TTCM		= DISABLE;								//非时间触发通信模式   
	CAN_InitStructure.CAN_ABOM		= DISABLE;								//软件自动离线管理	  
	CAN_InitStructure.CAN_AWUM		= DISABLE;								//睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
	CAN_InitStructure.CAN_NART		= DISABLE;								//禁止报文自动传送 
	CAN_InitStructure.CAN_RFLM		= DISABLE;								//报文不锁定,新的覆盖旧的  
	CAN_InitStructure.CAN_TXFP		= ENABLE;								//优先级由报文标识符决定 
	CAN_InitStructure.CAN_Mode		= CAN_Mode_Normal;						//正常模式 
	
	CAN_InitStructure.CAN_SJW		= CAN_SJW_1tq;							//重新同步跳跃宽度(Tsjw)CAN_SJW_1tq+1个时间单位
	CAN_InitStructure.CAN_BS1		= CAN_BS1_9tq;							//Tbs1范围CAN_BS1_1tq ~CAN_BS1_16tq
	CAN_InitStructure.CAN_BS2		= CAN_BS2_4tq;							//Tbs2范围CAN_BS2_1tq ~	CAN_BS2_8tq
	CAN_InitStructure.CAN_Prescaler	= 3;									//分频系数(Fdiv)3+1	
	CAN_Init(CAN2, &CAN_InitStructure);										//初始化CAN1 
	//配置过滤器
	CAN_FilterInitStructure.CAN_FilterNumber			=14;						//(CAN2只能用过滤器14~28？小于14的进不了中断)						//过滤器14
	CAN_FilterInitStructure.CAN_FilterMode				=CAN_FilterMode_IdMask; 
	CAN_FilterInitStructure.CAN_FilterScale				=CAN_FilterScale_32bit;		//32位
	CAN_FilterInitStructure.CAN_FilterIdHigh			=CANFILTERIDH;				
	CAN_FilterInitStructure.CAN_FilterIdLow				=0x0000;					//32位MASK
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh		=0x00E0;					//32位MASK
	CAN_FilterInitStructure.CAN_FilterMaskIdLow			=0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment	=CAN_Filter_FIFO0;			//过滤器14关联到FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation		=ENABLE;					//激活过滤器0
	CAN_FilterInit(&CAN_FilterInitStructure);

	CAN_ITConfig(CAN2,CAN_IT_FMP0,ENABLE);
	NVIC_InitStructure.NVIC_IRQChannel 						= CAN2_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	= 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority			= 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd					= ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
}


#if CAN1_RX0_INT_ENABLE	//使能RX0中断
//中断服务函数
CanRxMsg CAN1RxMessage;
void CAN1_RX1_IRQHandler(void)
{
	OSIntEnter();
	CAN_Receive(CAN1, 0, &CAN1RxMessage);

	Elmo_Init_Flag |= (1<<((CAN1RxMessage.StdId&0x0F)-1));//是1右移多少位

	if((CAN1RxMessage.StdId>COBID_TPDO2) && (CAN1RxMessage.StdId<(COBID_TPDO2 + 17)))
	{
		RobotStatus.Elmo_Heartbeat = 0;
		RobotStatus.Elmo_Status = true;
	}
	
	if((CAN1RxMessage.StdId>0x200)&&(CAN1RxMessage.StdId<0x208))
	{
		M2006_Get_Feedback(CAN1RxMessage.StdId,CAN1RxMessage.Data);
		pos_rec(CAN1RxMessage.StdId - 0x200);
		RobotStatus.M2006_Heartbeat = 0;
		RobotStatus.M2006_Status = true;
	}
	
	else
	{
		RobotStatus.Elmo_Heartbeat = 0;
		RobotStatus.Elmo_Status = true;
	}

	OSIntExit();
}
#endif

//can发送一组数据(固定格式:ID为0X12,标准帧,数据帧)	
//len:数据长度(最大为8)				     
//msg:数据指针,最大为8个字节.
//返回值:0,成功;
//		 其他,失败;
u8 debug=0;
u8 CAN1_Send_Msg(u8* msg,u8 len,u8 send_id)
{	
	u8 mbox;
	u16 i=0;
	CanTxMsg TxMessage;
	TxMessage.StdId=send_id;	 // 
	debug = send_id;
	TxMessage.ExtId=0x12;	 // 设置扩展标示符（29位）
	TxMessage.IDE=0;		  // 使用扩展标识符
	TxMessage.RTR=0;		  // 消息类型为数据帧，一帧8位
	TxMessage.DLC=len;							 // 发送两帧信息
	for(i=0;i<len;i++)
	TxMessage.Data[i]=msg[i];				 // 第一帧信息          
	mbox= CAN_Transmit(CAN1, &TxMessage);   
	i=0;
	while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//等待发送结束
	if(i>=0XFFF)return 1;
	return 0;		

}
//can口接收数据查询
//buf:数据缓存区;	 
//返回值:0,无数据被收到;
//		 其他,接收的数据长度;
u8 CAN1_Receive_Msg(u8 *buf)
{		   		   
 	u32 i;
	CanRxMsg RxMessage;
    if( CAN_MessagePending(CAN1,CAN_FIFO0)==0)return 0;		//没有接收到数据,直接退出 
    CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);//读取数据	
    for(i=0;i<RxMessage.DLC;i++)
    buf[i]=RxMessage.Data[i];  
	return RxMessage.DLC;	
}



//CAN2用于与主控进行通信
CanRxMsg CAN2RxMessage;
void CAN2_RX0_IRQHandler()
{
	OSIntEnter();
	CAN_Receive(CAN2, CAN_FIFO0, &CAN2RxMessage);
	switch(CAN2RxMessage.StdId)
	{
		
		/*软件复位*/
		case RESET_CMD:
			NVIC_SystemReset();
			break;
		
		/*主控心跳包*/
		case CHASSIS_HB:
			RobotStatus.Chassis_Heartbeat = 0;
			RobotStatus.Chassis_Status = true;
			break;
		
		/*上层机构抓球以及发射*/
		case ROBOT_LANCH:
			/*要根据上层结构来写*/
			break;
		
		/*上层机构放下机械臂*/
		case ROBOT_LAYDOWN:
			/*要根据上层结构来写*/
			break;
		
		
		/*上层机构初始化*/
		case ROBOT_INIT:
			/*要根据上层结构来写*/
			Robot_Init();
			break;
		
		/*底盘主控握手*/
		case CHASSIS_ONLINE:
			if(CAN2RxMessage.Data[0]==0xff)
				RobotStatus.FirstHSstatus  = true;
			if(CAN2RxMessage.Data[1]==0xff)
				RobotStatus.SecondHSstatus = true;  
			//这个地方还没加定时器
		
			break;
	
	
		default:break;
	}
	OSIntExit();
}

