/*
**********************************************************************************************************
    *@  file:elmo.c���ڲ���ϵͳ�У�
		*@  data: 11/7/2017
    *@  version: v2.0
    *@  brief: ���ӵ������õĿ��������Ϳ���ɲ������
    *@         ����������Ԥ������
    *@         ������note1�еĺ���ע��
    *@         ������note2�е�ע������
    *...............................................................................
    *@ Notes1: (1)��ʼ��������       
    *@            name      : Elmo_Init(CAN_TypeDef* CANx, uint8_t PPr, uint8_t SPr)
    *@            function  : ���ó�ʼ����������ʹ��CANͨ��ģ�飬���ΪCAN1��CAN2������ѡ��
    *@			                  CAN�ڣ�ȷ������ͨ�Ų���ȷ��ͨ���ٶ�Ϊ1Mbps��ͬʱѡ��TIM7���ж�
    *@	     		              ���ȼ���Ϊ�ײ㱨�ķ����ṩʱ��	
    *@            input     : CANx      CAN1 or CAN2
    *@						            PPr       TIM7����ռ���ȼ�
    *@                        SPr       TIM7�Ĵ����ȼ�
    *@            output    : 0	          ��ʼ���ɹ�
    *@                        0x80000000  ����û������CAN����
    *@ 							          ����         ��Ӧ��ELMO��IDλû����1
    *@	
    *@         (2)����ģʽ������        		
    *@            name      : Elmo_PTM(u8 elmoID, float torque);
    *@            function  : �������ؿ��ƺ�����ά�ֵ�����غ㶨,֧��ģʽ�л�  
    *@            input     : elmoID    ȡelmo�ڵ�ID
    *@			    			        torque     Ŀ��ת��(A)
    *@            output    : 0         �������óɹ�
    *@                        1         ��������ʧ��
    *@ 	
    *@         (3)�ٶ�ģʽ������  
    *@            name      : Elmo_PVM(u8 elmoID, s32 speed);
    *@            function  : �����ٶȿ��ƺ���,֧��ģʽ�л� ��֧���������ã�elmo 
    *@                        ���������ٶȻ���ٶ�ʹ�� ��ת�ٴﵽ�趨ֵ
    *@            input     : elmoID    ȡelmo�ڵ�ID      
    *@                        speed     Ŀ���ٶ�(cnt/s)
    *@            output    : 0         �������óɹ�
    *@                        1         ��������ʧ��
    *@
    *@         (4)λ��ģʽ������   
    *@            name      : Elmo_PPM(u8 elmoID, u32 speed, s32 position, u8 PPMmode);
    *@            function  : �����ٶȿ��ƺ�����֧��ģʽ�л���֧���������ã�����POS_ABSģʽ
    *@ 						            ����ָ���յ����λ��������ϵ��ĵ����λ�ã�Ҳ���ǵ�� �У�Ŀ    
    *@ 						            ��λ��ʼ�ľ���λ�á�����POS_RELģʽ�У����յ����λ��������elmo
    *@ 				                ����λ�üĴ����еľ���λ�ü���Ŀ��λ�ã�����λ�üĴ�����CAN_init 
    *@                        ��Ĭ��Ϊ0
    *@						  ����1��
    *@						  Elmo_PPM(1, 50000, 500000, POS_ABS);
    *@						  while(position != 500000);
    *@					   	  Elmo_PPM(1, 50000, 250000, POS_REL);		
    *@                             ���λ����750000��		
    *@						            ����2��
    *@						            Elmo_PPM(1, 50000, 500000, POS_ABS);
    *@						            while(position != 500000);
    *@						            Elmo_PPM(1, 50000, 250000, POS_ABS);			
    *@						            ���λ����250000��		
    *@						            ����3��
    *@						            Elmo_PPM(1, 50000, 500000, POS_ABS);
    *@						            while(position <= 250000);
    *@						            Elmo_PPM(1, 50000, 250000, POS_REL);			
    *@					              ���λ����750000��		
    *@            input     : elmoID      ȡelmo�ڵ�ID,�����ڵ��̿����е���1--4        
    *@ 						            speed       �ٶ�(cnt/s)
    *@ 					              position    Ŀ��λ��(cnt)
    *@ 						            PPMmode     ����ģʽ
    *@ 						            POS_ABS    // PPM���з�ʽ:����λ��
    *@ 		   			            POS_REL    // PPM���з�ʽ:���λ��
    *@  			    output    : 0         �������óɹ�
    *@                        1         ��������ʧ��
    *@
    *@         (5)����ͷź�����   
    *@            name      : Elmo_Close(u8 elmoID);
    *@            function  : ��������رգ���������Լ�����ʻ,֧���������ã�֧�����ϵ��ã���
    *@						            ���ڸ���elmo����ǰ����ã�������Ӱ�����������Ĺ���ʵ�ֺ�elmo��
    *@						            ����														
    *@            input     : elmoID      ȡelmo�ڵ�ID     
    *@  			    output    : 0         �������óɹ�
    *@                        1         ��������ʧ��
    *@         
    *@         (6)�������������
    *@            name      : Elmo_Stop(u8 elmoID);
    *@            function  : ���������ά�ֵ����ǰλ�ò��䣬֧���������ã�֧�����ϵ��ã���
    *@ 						            ���ڸ���elmo����ǰ����ã�������Ӱ�����������Ĺ���ʵ�ֺ�elmo��
    *@ 						            ����	
    *@            input     : elmoID      ȡelmo�ڵ�ID     
    *@  			    output    : 0         �������óɹ�
    *@                        1         ��������ʧ��
    *@	
    *@         (7)���ٶ����ú�����	
    *@  		  name      : Elmo_SetAcc(u8 elmoID, u32 acc, u32 dec)
    *@            function  : �����ٶ�ģʽ��λ��ģʽ�ĵ���Ӽ��٣����û���ں����е��ã���ʼ��
    *@	                      ����ٶ�Ĭ��Ϊ1000000000�����ٶ�Ĭ��Ϊ1000000000
    *@			  input     : elmoID      ȡelmo�Ľڵ�ID
    *@                        acc         ���ٶ�,���ٶ�����ܳ���1000000000,ͬʱӦ���ǵ������
    *@                        dec         ���ٶ�,���ٶ�����ܳ���1000000000,ͬʱӦ���ǵ������
    *@  		  output    : 0         �������óɹ�
    *@         (8)λ�ö�ȡ������	
    *@  		  name      : Elmo_Read_POS(u8 elmoID)
    *@            function  : ��ȡ��������������ݣ��������ж��ж�ȡ���洢��ȫ�ֱ���Encoder_Data��
    *@			  input     : elmoID      ȡelmo�Ľڵ�ID
    *@  		  output    : NONE
    *@         (9)λ�����ú�����	
    *@  			    name      : Elmo_Set_POS(u8 elmoID,s32 POS)
    *@            function  : ���õ��������������
    *@			      input     : elmoID      ȡelmo�Ľڵ�ID
    *@                        POS         Ҫ��elmo���õĵ�ַ
    *@  			    output    : NONE
    *@         (10)������ȡ������	
    *@  			     name      : Elmo_Read_ACT_CUR(u8 elmoID)
    *@             function  : ��ȡ����й����������ݣ��������ж��ж�ȡ���洢��ȫ�ֱ���IQ��
    *@			       input     : elmoID      ȡelmo�Ľڵ�ID
    *@  			     output    : NONE
	*@        ��11��ELMO���³�ʼ������   ����û���õ���
	*@             name      : Elmo_Reinit(void)
	*@             function  : ELMO���³�ʼ��
    *@			       input     : NULL
    *@  			     output    : NULL		
	*@         (12)������������
	*@             name      : Elmo_SetContinuousCurrent(u8 elmoID, float rateCurrent)
	*@             function  : ������������
	*@             input     ��u8 elmoID    ELMO��ID��    
	*@                         float rateCurrent
	*@             output    ��0  �ɹ�
	*@        ��13����ֵ��������
	*@             name      : Elmo_SetPeakCurrent(u8 elmoID, float maxCurrent)
	*@             function  :��ֵ��������
	*@             input     ��u8 elmoID    ELMO��ID��    
	*@                         float maxCurrent ������ֵ
	*@             output    ��0  �ɹ�		
	*@        ��14����������
	*@             name      : Elmo_SetCurrent(u8 elmoID, float rateCurrent, float maxCurrent)
	*@             function  :��ֵ��������
	*@             input     ��u8 elmoID    ELMO��ID�� 
��  *@                         float rateCurrent
	*@                         float maxCurrent ������ֵ
	*@             output    ��0  �ɹ�
	*@        ��15��NMT״̬���ú���
	*@             name      : NMTCmd(u8 NodeID, u8 MNTCmd)
	*@             function  :����CANOPEN��NMT״̬���ã�ElmoCanOpen �ײ㺯����
	*@             input     ��u8 elmoID    ELMO��ID�� 
��  *@                         u8 MNTCmd    Canopen��NMTָ��,��NMT_xxx
	*@             output    ��NULL		
	*@        ��16�����͸�ELMOָ���
	*@             name      ��RSDO(u8 NodeID, u16 Index, u8 SubIndex, u32 Data)
	*@             function  ��ʹ������SDO����ָ���
    *@             input     ��u8 NodeID       ELMO��ID��
	*@                         u16 Index       ����
    *@                         u8 SubIndex     ������
    *@                         u32 Data        ����
	*@             output    ��NULL
	*@         (17)��ȡ��ELMO��ָ���
	*@             name      ��RSDORead(u8 NodeID, u16 Index, u8 SubIndex, u32 Data)
	*@             function  ��ʹ������SDO����ָ���
    *@             input     ��u8 NodeID       ELMO��ID��
	*@                         u16 Index       ����
    *@                         u8 SubIndex     ������
    *@                         u32 Data        ����
	*@             output    ��NULL		            
	*@         ��18�����Ͷ�����ָ���ELMO
	*@              name     ��RPDO2_Cmd_data(u8 NodeID, u8 *Cmd, u8 Index, u8 Type, u32 Data)
	*@              function  �����Ͷ�����ָ���ELMO
    *@              input     ��u8 NodeID       ELMO��ID��
	*@                          u8 Cmd          ����,���ַ�����ʽ����
	*@                         	u16 Index       ����
    *@                          u8 Type         ��������
    *@                          u32 Data        ����
	*@              output    ��NULL				
	*@          ��19��ʹ�ö����Ʒ����ַ���ָ���ELMO
	*@              name      ��RPDO2_Cmd_string(u8 NodeID, u8 *Cmd)
	*@          ��20�����ط�������֡��ELMO
	*@               name      ��void SendHeart2Elmo(void)
	*@          ��21��������ʼ������
	*@               name      ��void Variate_init(void)
	*@          ��22����������ת����8�ֽ�ʮ��������
	*@               name      ��u32 f2h(float x)
	*@          ��23��CAN�жϴ�����
	*@               name      ��CAN1_RX0_IRQHandler()
	*@               funtion   ����ȡELMO����ֵ
	*@          ��24��ͬʱ����ID1~ID4���
	*@               name      ��Chassis+ָ��
	*@               function  ��ͬʱ����ID1~ID4���
	*@          ��24��ͬʱ����ID7~ID48���
	*@               name      ��Climb+ָ��
	*@               function  ��ͬʱ����ID7~ID8���		
	*@
	*@ 
	*@		
    *...............................................................................
    *@ Notes2: (1)��CAN������ 
    *@			     CAN1 RX -> PD0     CAN1 TX -> PD1
    *@               CAN2 RX -> PB5     CAN2 TX -> PB6
    *@         (2)���޸ĺ궨��CAN_BUF_NUM��ֵ�޸Ļ�����Ŀ
    *@         (3)���޸�TIM7_Configuration()��TIM7�����ж����ȼ�
    *@         (4)��ע��CANʱ��Ϊ42MHz,TIM7ʱ��Ϊ42MHz
    *@ 	       (5)��ע��д���ȼ����麯��
    *@         (6)��ע������ж��еĵ��ID
    *@         (7)����Elmo_Read_POS��Elmo_Read_ACT_CUR��������֮ǰҪ��ʱ1ms�����Է�ֹ��ϵ�ǰ������˶�״̬
    *@         (8)����Four_Elmo_PVM��������֮ǰҪ����Elmo_Pre_PVM��Ԥ����
    *@         (9)��globalCAN_ID_MASTER_CONTROLΪȫ�ֱ�����������Global.h��
	*@         (10)��#define portTICK_MSΪȫ�ֱ�����������portmacro.h��freertos��ͷ�ļ�֮�У���
	*@         (11)��T_CanFrame��������can.h��
	*@
	*@
		
    *@...............................................................................
    *@ Notes3: (1)��û��д�ڵ�ID����ID�ķ��亯��
    *@	       (2)��û�мӷ������Ļ��� 
    *@		   (3)��û�м�ƽ���������ú���	    
    *@		   (4)��Elmo_Stop()��Elmo_PTM()����Ӧʱ��̫������Ҫ�Ż�			
    *@         (5)��û�мӽڵ㱣��������������������				 
    *@    	   (6)��Elmo_Delay100us()���ԸĽ������Ч��    
    *@         (7)��GroupID��ʱ��������	
**********************************************************************************************************
*/

#ifndef _ELMO_H
#define _ELMO_H
#include "stm32f4xx.h"
#include "string.h"
#include "RC_2006.h"
#include "includes.h"

/*******************************************************************************
* �궨�� 
*******************************************************************************/
/* 0:���ٶȿ���ģʽ�������ٶ�ģʽ,��ʱ���ٶ�ʧЧ */ 
/* 1:��λ�ÿ���ģʽ�������ٶ�ģʽ,��ʱ���ٶ���Ч */ 
#define  JV_IN_PCM          0


/* ѭ�����в��� */
#define ELMO_NUM            8 //(11)                        // Elmo����,�����ϸ��չ��ظ�����ELMO���ã����ö����ã�����
#define CAN_BUF_NUM         150                          	//����ָ������  �ĳ�200�� ԭ����1000  �ּ�С����150
#define CAN_ID_DELAY        0x129                           //��ʱָ��

/* ELMO��ز��� */
#define RATE_CURRENT        15                       	    // �����(A)
#define PEAK_CURRENT        (15*2)                	      	// ��ֵ����(A)
#define MAX_VOLOCITY        10600                           // ������ת��(rpm)

/* ����������ز��� */
#define RATE_CUR            RATE_CURRENT                    // �����(A)          CL[1]
#define MAX_CURRENT         PEAK_CURRENT                    // ��ֵ����(A)          PL[1]
#define MAX_PM_SPEED        (MAX_VOLOCITY*2000/60)          // ���ƽ���ٶ�(cnt/s)  VH[2]
#define MIN_PM_SPEED        (u32)(-MAX_PM_SPEED)            // ��Сƽ���ٶ�(cnt/s)  VL[2]
#define MAX_FB_SPEED        ((MAX_VOLOCITY+1000)*2000/60)   // ������ٶ�(cnt/s)  HL[2]
#define MIN_FB_SPEED        (u32)(-MAX_FB_SPEED)            // ��С�����ٶ�(cnt/s)  LL[2]

/* ƽ���˶���ز��� */ 
#define PM_ACC              3500000                         // ƽ�����ٶ�(cnt/s^2)  AC
#define PM_DEC              3500000                         // ƽ�����ٶ�(cnt/s^2)  DC
#define QUICKSTOP_DEC       1000000000                      // ��ͣ���ٶ�(cnt/s^2)  SD
#define POSITION_LIMIT_MAX  1000000000                      // ���λ�ü���         VH[3] HL[3]
#define POSITION_LIMIT_MIN  (u32)-1000000000                // ��Сλ�ü���         VL[3] LL[3]

/* CANopen COB identifiers */
#define COBID_NMT_SERVICE   0x000
#define COBID_SYNC          0x080
#define COBID_EMERGENCY     0x080
#define COBID_TIME_STAMP    0x100
#define COBID_TPDO1         0x180
#define COBID_RPDO1         0x200
#define COBID_TPDO2         0x280  
#define COBID_RPDO2         0x300
#define COBID_TPDO3         0x380
#define COBID_RPDO3         0x400
#define COBID_TPDO4         0x480
#define COBID_RPDO4         0x500
#define COBID_TSDO          0x580
#define COBID_RSDO          0x600
#define COBID_HEARTBEAT     0x700

/* NMT Command Specifier */
#define NMT_ENTER_OPERATIONAL      0x01
#define NMT_ENTER_STOPPED          0x02
#define NMT_ENTER_PRE_OPERATIONAL  0x80
#define NMT_RESET_NODE             0x81
#define NMT_RESET_COMMUNICATION    0x82

/* Binary Interpreter Command */
#define UM_IDLE             0x00
#define UM_TCM              0x01                      // Torque control mode,���ؿ���ģʽ
#define UM_PCM              0x05                      // Position control mode��λ�ÿ���ģʽ
#define UM_UNC              0x06                      // ����ȷ��ģʽ��������ȫ���������

#if JV_IN_PCM
   #define UM_SCM           0x05                      // Position control mode��
#else
   #define UM_SCM           0x02                      // Speed control mode
#endif

#define TYPE_INTEGER      0
#define TYPE_FLOAT        1

#define MO_OFF            0
#define MO_ON             1

#define POS_REL           0
#define POS_ABS           1

/*****************************************************************************/
//����ѡ��ĵ��ID��Ϊ��������������
/*****************************************************************************/

/*���ѡ��*/
#define ID_0    //��ʾ���еĵ�� 
#define ID_1
#define ID_2
#define ID_3
#define ID_4
#define ID_5
#define ID_6
#define ID_7
#define ID_8
//#define ID_9
//#define ID_10
//#define ID_11
//#define ID_12
//#define ID_13
//#define ID_14
//#define ID_15



#define CATCH_BALL	0x01
#define SET_BALL	0x02
/*******************************************************************************
* �ṹ��
*******************************************************************************/
/* CANѭ������Ԫ�� */ 
typedef struct __CANDATA 
{
   u16 COBID;               // CANopen COB identifier + NodeID
   u8  DLC;                 // Data Length Code
   u8  DATA[8];             // Data
} CANDATA;

/* CANѭ�����нṹ�� */
typedef struct __CANQUEUE
{
   u16 Front;        
   u16 Rear;
   CANDATA CANBUF[CAN_BUF_NUM];
} CANQUEUE;

/* Elmo�ṹ��,��¼�ڵ�ID,״̬�Ϳ��Ʋ��� */
typedef struct __Elmo
{
   u8 NodeID;         // elmo����
   u8 CurOPMode;      // ��ǰ����ģʽ 
}Elmo;
/*����ת��������*/
typedef union 
{
    uint32_t u32_form;
    int32_t  s32_form;
    uint8_t  u8_form[4];
    int8_t   s8_form[4];
    float    float_form;
}DataConvert;


extern u8 CAN_State;													//CAN2״̬
extern u32 Elmo_Init_Flag;

extern CanTxMsg CAN1TxMessage;
extern CanTxMsg CAN2TxMessage;

/*******************************************************************************
* ��������
*******************************************************************************/
/* ELMO��ʼ������,������� */
extern u32 Elmo_Init( CAN_TypeDef* CANx, uint8_t PPr, uint8_t SPr);
extern void Elmo_Reinit(u8 elmoID);
u32 Elmo_Init_Fake(CAN_TypeDef* CANx, uint8_t PPr, uint8_t SPr);

/* ELMO���ƺ������������ */
extern u8 Elmo_PTM(u8 elmoID, float torque);
extern u8 Elmo_PVM(u8 elmoID, s32 speed);  
extern u8 Elmo_PPM(u8 elmoID, u32 speed, s32 position, u8 PPMmode);		//		POS_ABS  POS_REL 
extern u8 Elmo_Close(u8 elmoID);
extern u8 Elmo_Stop(u8 elmoID);
extern u8 Elmo_SetAcc(u8 elmoID, u32 acc, u32 dec);
extern void Elmo_Read_POS(u8 elmoID);
extern void Elmo_Set_POS(u8 elmoID,s32 POS);
extern void Elmo_Read_ACT_CUR(u8 elmoID);
extern void Elmo_Pre_PVM(u8 elmoID);
extern void Four_Elmo_Stop(u8 elmoID1,u8 elmoID2,u8 elmoID3,u8 elmoID4);
extern void Four_Elmo_PVM(u8 elmoID1, s32 speed1,u8 elmoID2, s32 speed2,u8 elmoID3, s32 speed3,u8 elmoID4, s32 speed4);


/* CANOpen��ʵ�ֺ���,��������� */
static void NMTCmd(Elmo *elmo, u8 MNTCmd);
static void RSDO(Elmo *elmo, u16 Index, u8 SubIndex, u32 Data);
static void RPDO2_Cmd_data(Elmo *elmo, u8 *Cmd, u8 Index, u8 Type, u32 Data);
static void RPDO2_Cmd_string(Elmo *elmo, u8 *Cmd);

/* Ӳ����ʼ������,��������� */
void CAN_init(CAN_TypeDef* CANx);   
static void TIM7_init(uint8_t PPr, uint8_t SPr);
static int Self_test(void);
static void Variate_init(void);

/* ���ݷ��͡�ת������ʱ����,��������� */
static void Elmo_SendCmd(void);
static void Elmo_CANSend(CANDATA *pCANDATA);
static void Elmo_Delay100us_IDx( Elmo *elmo , u8 N100us);
static void Elmo_software_delay_ms(unsigned int t);
static u32 f2h(float x);


/*�������*/

//u8 CAN1_Send_Msg(u8* msg,u8 len,int id);
u8 CAN2_Send_Msg(u8* msg,u8 len,int id);
void SendCmd2Chassis(u8 Identifier);

#endif

