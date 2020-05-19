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
    //ʹ�����ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//ʹ��PORTAʱ��	                   											 

  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//ʹ��CAN1ʱ��	
	
    //��ʼ��GPIO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8| GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
    GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��PB8,PB9
	
	//���Ÿ���ӳ������
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource8,GPIO_AF_CAN1); //GPIOB8����ΪCAN1
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource9,GPIO_AF_CAN1); //GPIOB9����ΪCAN1
	  
  	//CAN��Ԫ����
   	CAN_InitStructure.CAN_TTCM=DISABLE;	//��ʱ�䴥��ͨ��ģʽ   
  	CAN_InitStructure.CAN_ABOM=DISABLE;	//�����Զ����߹���	  
  	CAN_InitStructure.CAN_AWUM=DISABLE;//˯��ģʽͨ����������(���CAN->MCR��SLEEPλ)
  	CAN_InitStructure.CAN_NART=DISABLE;	//��ֹ�����Զ����� 
  	CAN_InitStructure.CAN_RFLM=DISABLE;	//���Ĳ�����,�µĸ��Ǿɵ�  
  	CAN_InitStructure.CAN_TXFP=ENABLE;	//���ȼ��ɱ��ı�ʶ������ 
  	CAN_InitStructure.CAN_Mode= CAN_Mode_Normal;	 //ģʽ���� 
  	CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;	//����ͬ����Ծ����(Tsjw)Ϊtsjw+1��ʱ�䵥λ CAN_SJW_1tq~CAN_SJW_4tq
  	CAN_InitStructure.CAN_BS1=CAN_BS1_9tq; //Tbs1��ΧCAN_BS1_1tq ~CAN_BS1_16tq
  	CAN_InitStructure.CAN_BS2=CAN_BS2_4tq;//Tbs2��ΧCAN_BS2_1tq ~	CAN_BS2_8tq
  	CAN_InitStructure.CAN_Prescaler=3;  //��Ƶϵ��(Fdiv)Ϊbrp+1	
  	CAN_Init(CAN1, &CAN_InitStructure);   // ��ʼ��CAN1 
    
		//���ù�����
 	CAN_FilterInitStructure.CAN_FilterNumber=14;	  //������0
  	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 
  	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32λ 
  	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;////32λID
  	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
  	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32λMASK
  	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
   	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//������0������FIFO0
  	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //���������0
  	CAN_FilterInit(&CAN_FilterInitStructure);//�˲�����ʼ��
		
#if CAN1_RX0_INT_ENABLE
	
	  CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);//FIFO0��Ϣ�Һ��ж�����.		    
  
  	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // �����ȼ�Ϊ1
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // �����ȼ�Ϊ0
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
	
	/*CAN2��ʼ��*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);

	GPIO_InitStructure.GPIO_Pin		= GPIO_Pin_12 | GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AF;							//���ù���
	GPIO_InitStructure.GPIO_OType	= GPIO_OType_PP;						//�������
	GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_100MHz;					//100MHz
	GPIO_InitStructure.GPIO_PuPd	= GPIO_PuPd_UP;							//����
	GPIO_Init(GPIOB, &GPIO_InitStructure);									//��ʼ��PB12,PB13
	//���Ÿ���ӳ������
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource12,GPIO_AF_CAN2);					//GPIOB12����ΪCAN2
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource13,GPIO_AF_CAN2);					//GPIOB13����ΪCAN2

	CAN_DeInit(CAN2);
	CAN_StructInit(&CAN_InitStructure);	
	//CAN��Ԫ����
	CAN_InitStructure.CAN_TTCM		= DISABLE;								//��ʱ�䴥��ͨ��ģʽ   
	CAN_InitStructure.CAN_ABOM		= DISABLE;								//�����Զ����߹���	  
	CAN_InitStructure.CAN_AWUM		= DISABLE;								//˯��ģʽͨ����������(���CAN->MCR��SLEEPλ)
	CAN_InitStructure.CAN_NART		= DISABLE;								//��ֹ�����Զ����� 
	CAN_InitStructure.CAN_RFLM		= DISABLE;								//���Ĳ�����,�µĸ��Ǿɵ�  
	CAN_InitStructure.CAN_TXFP		= ENABLE;								//���ȼ��ɱ��ı�ʶ������ 
	CAN_InitStructure.CAN_Mode		= CAN_Mode_Normal;						//����ģʽ 
	
	CAN_InitStructure.CAN_SJW		= CAN_SJW_1tq;							//����ͬ����Ծ����(Tsjw)CAN_SJW_1tq+1��ʱ�䵥λ
	CAN_InitStructure.CAN_BS1		= CAN_BS1_9tq;							//Tbs1��ΧCAN_BS1_1tq ~CAN_BS1_16tq
	CAN_InitStructure.CAN_BS2		= CAN_BS2_4tq;							//Tbs2��ΧCAN_BS2_1tq ~	CAN_BS2_8tq
	CAN_InitStructure.CAN_Prescaler	= 3;									//��Ƶϵ��(Fdiv)3+1	
	CAN_Init(CAN2, &CAN_InitStructure);										//��ʼ��CAN1 
	//���ù�����
	CAN_FilterInitStructure.CAN_FilterNumber			=14;						//(CAN2ֻ���ù�����14~28��С��14�Ľ������ж�)						//������14
	CAN_FilterInitStructure.CAN_FilterMode				=CAN_FilterMode_IdMask; 
	CAN_FilterInitStructure.CAN_FilterScale				=CAN_FilterScale_32bit;		//32λ
	CAN_FilterInitStructure.CAN_FilterIdHigh			=CANFILTERIDH;				
	CAN_FilterInitStructure.CAN_FilterIdLow				=0x0000;					//32λMASK
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh		=0x00E0;					//32λMASK
	CAN_FilterInitStructure.CAN_FilterMaskIdLow			=0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment	=CAN_Filter_FIFO0;			//������14������FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation		=ENABLE;					//���������0
	CAN_FilterInit(&CAN_FilterInitStructure);

	CAN_ITConfig(CAN2,CAN_IT_FMP0,ENABLE);
	NVIC_InitStructure.NVIC_IRQChannel 						= CAN2_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	= 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority			= 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd					= ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
}


#if CAN1_RX0_INT_ENABLE	//ʹ��RX0�ж�
//�жϷ�����
CanRxMsg CAN1RxMessage;
void CAN1_RX1_IRQHandler(void)
{
	OSIntEnter();
	CAN_Receive(CAN1, 0, &CAN1RxMessage);

	Elmo_Init_Flag |= (1<<((CAN1RxMessage.StdId&0x0F)-1));//��1���ƶ���λ

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

//can����һ������(�̶���ʽ:IDΪ0X12,��׼֡,����֡)	
//len:���ݳ���(���Ϊ8)				     
//msg:����ָ��,���Ϊ8���ֽ�.
//����ֵ:0,�ɹ�;
//		 ����,ʧ��;
u8 debug=0;
u8 CAN1_Send_Msg(u8* msg,u8 len,u8 send_id)
{	
	u8 mbox;
	u16 i=0;
	CanTxMsg TxMessage;
	TxMessage.StdId=send_id;	 // 
	debug = send_id;
	TxMessage.ExtId=0x12;	 // ������չ��ʾ����29λ��
	TxMessage.IDE=0;		  // ʹ����չ��ʶ��
	TxMessage.RTR=0;		  // ��Ϣ����Ϊ����֡��һ֡8λ
	TxMessage.DLC=len;							 // ������֡��Ϣ
	for(i=0;i<len;i++)
	TxMessage.Data[i]=msg[i];				 // ��һ֡��Ϣ          
	mbox= CAN_Transmit(CAN1, &TxMessage);   
	i=0;
	while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//�ȴ����ͽ���
	if(i>=0XFFF)return 1;
	return 0;		

}
//can�ڽ������ݲ�ѯ
//buf:���ݻ�����;	 
//����ֵ:0,�����ݱ��յ�;
//		 ����,���յ����ݳ���;
u8 CAN1_Receive_Msg(u8 *buf)
{		   		   
 	u32 i;
	CanRxMsg RxMessage;
    if( CAN_MessagePending(CAN1,CAN_FIFO0)==0)return 0;		//û�н��յ�����,ֱ���˳� 
    CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);//��ȡ����	
    for(i=0;i<RxMessage.DLC;i++)
    buf[i]=RxMessage.Data[i];  
	return RxMessage.DLC;	
}



//CAN2���������ؽ���ͨ��
CanRxMsg CAN2RxMessage;
void CAN2_RX0_IRQHandler()
{
	OSIntEnter();
	CAN_Receive(CAN2, CAN_FIFO0, &CAN2RxMessage);
	switch(CAN2RxMessage.StdId)
	{
		
		/*������λ*/
		case RESET_CMD:
			NVIC_SystemReset();
			break;
		
		/*����������*/
		case CHASSIS_HB:
			RobotStatus.Chassis_Heartbeat = 0;
			RobotStatus.Chassis_Status = true;
			break;
		
		/*�ϲ����ץ���Լ�����*/
		case ROBOT_LANCH:
			/*Ҫ�����ϲ�ṹ��д*/
			break;
		
		/*�ϲ�������»�е��*/
		case ROBOT_LAYDOWN:
			/*Ҫ�����ϲ�ṹ��д*/
			break;
		
		
		/*�ϲ������ʼ��*/
		case ROBOT_INIT:
			/*Ҫ�����ϲ�ṹ��д*/
			Robot_Init();
			break;
		
		/*������������*/
		case CHASSIS_ONLINE:
			if(CAN2RxMessage.Data[0]==0xff)
				RobotStatus.FirstHSstatus  = true;
			if(CAN2RxMessage.Data[1]==0xff)
				RobotStatus.SecondHSstatus = true;  
			//����ط���û�Ӷ�ʱ��
		
			break;
	
	
		default:break;
	}
	OSIntExit();
}
