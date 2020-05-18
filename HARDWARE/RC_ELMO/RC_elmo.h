/*
**********************************************************************************************************
    *@  file:elmo.c（在操作系统中）
		*@  data: 11/7/2017
    *@  version: v2.0
    *@  brief: 增加底盘试用的快速启动和快速刹车函数
    *@         增加了启动预处理函数
    *@         更新了note1中的函数注释
    *@         更新了note2中的注意事项
    *...............................................................................
    *@ Notes1: (1)初始化函数：       
    *@            name      : Elmo_Init(CAN_TypeDef* CANx, uint8_t PPr, uint8_t SPr)
    *@            function  : 调用初始化函数即可使用CAN通信模块，入参为CAN1或CAN2，用来选择
    *@			                  CAN口，确保物理通信层正确，通信速度为1Mbps，同时选择TIM7的中断
    *@	     		              优先级，为底层报文发送提供时序	
    *@            input     : CANx      CAN1 or CAN2
    *@						            PPr       TIM7的抢占优先级
    *@                        SPr       TIM7的从优先级
    *@            output    : 0	          初始化成功
    *@                        0x80000000  主控没有连上CAN总线
    *@ 							          其他         对应的ELMO的ID位没有置1
    *@	
    *@         (2)力矩模式函数：        		
    *@            name      : Elmo_PTM(u8 elmoID, float torque);
    *@            function  : 单轴力矩控制函数，维持电机力矩恒定,支持模式切换  
    *@            input     : elmoID    取elmo节点ID
    *@			    			        torque     目标转矩(A)
    *@            output    : 0         函数调用成功
    *@                        1         函数调用失败
    *@ 	
    *@         (3)速度模式函数：  
    *@            name      : Elmo_PVM(u8 elmoID, s32 speed);
    *@            function  : 单轴速度控制函数,支持模式切换 ，支持连续调用，elmo 
    *@                        将以最大加速度或减速度使电 机转速达到设定值
    *@            input     : elmoID    取elmo节点ID      
    *@                        speed     目标速度(cnt/s)
    *@            output    : 0         函数调用成功
    *@                        1         函数调用失败
    *@
    *@         (4)位置模式函数：   
    *@            name      : Elmo_PPM(u8 elmoID, u32 speed, s32 position, u8 PPMmode);
    *@            function  : 单轴速度控制函数，支持模式切换，支持连续调用，对于POS_ABS模式
    *@ 						            终是指最终电机的位置相对于上电后的电机的位置，也就是电机 中，目    
    *@ 						            标位置始的绝对位置。对于POS_REL模式中，最终电机的位置是现在elmo
    *@ 				                绝对位置寄存器中的绝对位置加上目标位置，绝对位置寄存器在CAN_init 
    *@                        后默认为0
    *@						  例子1：
    *@						  Elmo_PPM(1, 50000, 500000, POS_ABS);
    *@						  while(position != 500000);
    *@					   	  Elmo_PPM(1, 50000, 250000, POS_REL);		
    *@                             电机位置在750000处		
    *@						            例子2：
    *@						            Elmo_PPM(1, 50000, 500000, POS_ABS);
    *@						            while(position != 500000);
    *@						            Elmo_PPM(1, 50000, 250000, POS_ABS);			
    *@						            电机位置在250000处		
    *@						            例子3：
    *@						            Elmo_PPM(1, 50000, 500000, POS_ABS);
    *@						            while(position <= 250000);
    *@						            Elmo_PPM(1, 50000, 250000, POS_REL);			
    *@					              电机位置在750000处		
    *@            input     : elmoID      取elmo节点ID,请勿在底盘控制中调用1--4        
    *@ 						            speed       速度(cnt/s)
    *@ 					              position    目标位置(cnt)
    *@ 						            PPMmode     运行模式
    *@ 						            POS_ABS    // PPM运行方式:绝对位置
    *@ 		   			            POS_REL    // PPM运行方式:相对位置
    *@  			    output    : 0         函数调用成功
    *@                        1         函数调用失败
    *@
    *@         (5)电机释放函数：   
    *@            name      : Elmo_Close(u8 elmoID);
    *@            function  : 驱动输出关闭，电机靠惯性继续行驶,支持连续调用，支持联合调用（能
    *@						            够在各种elmo函数前后调用），不会影响其他函数的功能实现和elmo稳
    *@						            定性														
    *@            input     : elmoID      取elmo节点ID     
    *@  			    output    : 0         函数调用成功
    *@                        1         函数调用失败
    *@         
    *@         (6)电机抱死函数：
    *@            name      : Elmo_Stop(u8 elmoID);
    *@            function  : 电机抱死，维持电机当前位置不变，支持连续调用，支持联合调用（能
    *@ 						            够在各种elmo函数前后调用），不会影响其他函数的功能实现和elmo稳
    *@ 						            定性	
    *@            input     : elmoID      取elmo节点ID     
    *@  			    output    : 0         函数调用成功
    *@                        1         函数调用失败
    *@	
    *@         (7)加速度设置函数：	
    *@  		  name      : Elmo_SetAcc(u8 elmoID, u32 acc, u32 dec)
    *@            function  : 设置速度模式和位置模式的电机加减速，如果没有在函数中调用，初始化
    *@	                      后加速度默认为1000000000，减速度默认为1000000000
    *@			  input     : elmoID      取elmo的节点ID
    *@                        acc         加速度,加速度最大不能超过1000000000,同时应考虑电机性能
    *@                        dec         减速度,减速度最大不能超过1000000000,同时应考虑电机性能
    *@  		  output    : 0         函数调用成功
    *@         (8)位置读取函数：	
    *@  		  name      : Elmo_Read_POS(u8 elmoID)
    *@            function  : 读取电机编码器的数据，数据在中断中读取，存储在全局变量Encoder_Data中
    *@			  input     : elmoID      取elmo的节点ID
    *@  		  output    : NONE
    *@         (9)位置设置函数：	
    *@  			    name      : Elmo_Set_POS(u8 elmoID,s32 POS)
    *@            function  : 设置电机编码器的数据
    *@			      input     : elmoID      取elmo的节点ID
    *@                        POS         要给elmo设置的地址
    *@  			    output    : NONE
    *@         (10)电流读取函数：	
    *@  			     name      : Elmo_Read_ACT_CUR(u8 elmoID)
    *@             function  : 读取电机有功电流的数据，数据在中断中读取，存储在全局变量IQ中
    *@			       input     : elmoID      取elmo的节点ID
    *@  			     output    : NONE
	*@        （11）ELMO重新初始化函数   （并没有用到）
	*@             name      : Elmo_Reinit(void)
	*@             function  : ELMO重新初始化
    *@			       input     : NULL
    *@  			     output    : NULL		
	*@         (12)持续电流函数
	*@             name      : Elmo_SetContinuousCurrent(u8 elmoID, float rateCurrent)
	*@             function  : 持续电流函数
	*@             input     ：u8 elmoID    ELMO的ID号    
	*@                         float rateCurrent
	*@             output    ：0  成功
	*@        （13）峰值电流函数
	*@             name      : Elmo_SetPeakCurrent(u8 elmoID, float maxCurrent)
	*@             function  :峰值电流函数
	*@             input     ：u8 elmoID    ELMO的ID号    
	*@                         float maxCurrent 电流峰值
	*@             output    ：0  成功		
	*@        （14）电流函数
	*@             name      : Elmo_SetCurrent(u8 elmoID, float rateCurrent, float maxCurrent)
	*@             function  :峰值电流函数
	*@             input     ：u8 elmoID    ELMO的ID号 
、  *@                         float rateCurrent
	*@                         float maxCurrent 电流峰值
	*@             output    ：0  成功
	*@        （15）NMT状态设置函数
	*@             name      : NMTCmd(u8 NodeID, u8 MNTCmd)
	*@             function  :对于CANOPEN的NMT状态设置（ElmoCanOpen 底层函数）
	*@             input     ：u8 elmoID    ELMO的ID号 
、  *@                         u8 MNTCmd    Canopen的NMT指令,如NMT_xxx
	*@             output    ：NULL		
	*@        （16）发送给ELMO指令函数
	*@             name      ：RSDO(u8 NodeID, u16 Index, u8 SubIndex, u32 Data)
	*@             function  ：使用下载SDO进行指令发送
    *@             input     ：u8 NodeID       ELMO的ID号
	*@                         u16 Index       索引
    *@                         u8 SubIndex     子索引
    *@                         u32 Data        数据
	*@             output    ：NULL
	*@         (17)读取给ELMO的指令函数
	*@             name      ：RSDORead(u8 NodeID, u16 Index, u8 SubIndex, u32 Data)
	*@             function  ：使用下载SDO进行指令发送
    *@             input     ：u8 NodeID       ELMO的ID号
	*@                         u16 Index       索引
    *@                         u8 SubIndex     子索引
    *@                         u32 Data        数据
	*@             output    ：NULL		            
	*@         （18）发送二进制指令给ELMO
	*@              name     ：RPDO2_Cmd_data(u8 NodeID, u8 *Cmd, u8 Index, u8 Type, u32 Data)
	*@              function  ：发送二进制指令给ELMO
    *@              input     ：u8 NodeID       ELMO的ID号
	*@                          u8 Cmd          命令,以字符串形式输入
	*@                         	u16 Index       索引
    *@                          u8 Type         数据类型
    *@                          u32 Data        数据
	*@              output    ：NULL				
	*@          （19）使用二进制发送字符串指令给ELMO
	*@              name      ：RPDO2_Cmd_string(u8 NodeID, u8 *Cmd)
	*@          （20）主控发送心跳帧给ELMO
	*@               name      ：void SendHeart2Elmo(void)
	*@          （21）参数初始化函数
	*@               name      ：void Variate_init(void)
	*@          （22）将浮点数转化成8字节十六进制数
	*@               name      ：u32 f2h(float x)
	*@          （23）CAN中断处理函数
	*@               name      ：CAN1_RX0_IRQHandler()
	*@               funtion   ：读取ELMO返回值
	*@          （24）同时操作ID1~ID4电机
	*@               name      ：Chassis+指令
	*@               function  ：同时操作ID1~ID4电机
	*@          （24）同时操作ID7~ID48电机
	*@               name      ：Climb+指令
	*@               function  ：同时操作ID7~ID8电机		
	*@
	*@ 
	*@		
    *...............................................................................
    *@ Notes2: (1)、CAN口引脚 
    *@			     CAN1 RX -> PD0     CAN1 TX -> PD1
    *@               CAN2 RX -> PB5     CAN2 TX -> PB6
    *@         (2)、修改宏定义CAN_BUF_NUM的值修改缓冲数目
    *@         (3)、修改TIM7_Configuration()中TIM7更新中断优先级
    *@         (4)、注意CAN时钟为42MHz,TIM7时钟为42MHz
    *@ 	       (5)、注意写优先级分组函数
    *@         (6)、注意更改中断中的电机ID
    *@         (7)、在Elmo_Read_POS和Elmo_Read_ACT_CUR函数调用之前要延时1ms，可以防止打断当前电机的运动状态
    *@         (8)、在Four_Elmo_PVM函数调用之前要调用Elmo_Pre_PVM做预处理
    *@         (9)、globalCAN_ID_MASTER_CONTROL为全局变量，定义在Global.h中
	*@         (10)、#define portTICK_MS为全局变量，定义在portmacro.h（freertos的头文件之中）中
	*@         (11)、T_CanFrame被定义在can.h中
	*@
	*@
		
    *@...............................................................................
    *@ Notes3: (1)、没有写节点ID和组ID的分配函数
    *@	       (2)、没有加反馈报文机制 
    *@		   (3)、没有加平滑因子设置函数	    
    *@		   (4)、Elmo_Stop()对Elmo_PTM()的响应时间太长，需要优化			
    *@         (5)、没有加节点保护函数和总线心跳报文				 
    *@    	   (6)、Elmo_Delay100us()可以改进，提高效率    
    *@         (7)、GroupID暂时不能用了	
**********************************************************************************************************
*/

#ifndef _ELMO_H
#define _ELMO_H
#include "stm32f4xx.h"
#include "string.h"
#include "RC_2006.h"
#include "includes.h"

/*******************************************************************************
* 宏定义 
*******************************************************************************/
/* 0:在速度控制模式下启动速度模式,此时加速度失效 */ 
/* 1:在位置控制模式下启动速度模式,此时加速度生效 */ 
#define  JV_IN_PCM          0


/* 循环队列参数 */
#define ELMO_NUM            8 //(11)                        // Elmo个数,必须严格按照挂载个数的ELMO配置，不得多配置！！！
#define CAN_BUF_NUM         150                          	//缓冲指令条数  改成200了 原来是1000  又减小到了150
#define CAN_ID_DELAY        0x129                           //延时指令

/* ELMO相关参数 */
#define RATE_CURRENT        15                       	    // 额定电流(A)
#define PEAK_CURRENT        (15*2)                	      	// 峰值电流(A)
#define MAX_VOLOCITY        10600                           // 电机最大转速(rpm)

/* 保护机制相关参数 */
#define RATE_CUR            RATE_CURRENT                    // 额定电流(A)          CL[1]
#define MAX_CURRENT         PEAK_CURRENT                    // 峰值电流(A)          PL[1]
#define MAX_PM_SPEED        (MAX_VOLOCITY*2000/60)          // 最大平滑速度(cnt/s)  VH[2]
#define MIN_PM_SPEED        (u32)(-MAX_PM_SPEED)            // 最小平滑速度(cnt/s)  VL[2]
#define MAX_FB_SPEED        ((MAX_VOLOCITY+1000)*2000/60)   // 最大反馈速度(cnt/s)  HL[2]
#define MIN_FB_SPEED        (u32)(-MAX_FB_SPEED)            // 最小反馈速度(cnt/s)  LL[2]

/* 平滑运动相关参数 */ 
#define PM_ACC              3500000                         // 平滑加速度(cnt/s^2)  AC
#define PM_DEC              3500000                         // 平滑减速度(cnt/s^2)  DC
#define QUICKSTOP_DEC       1000000000                      // 急停减速度(cnt/s^2)  SD
#define POSITION_LIMIT_MAX  1000000000                      // 最大位置极限         VH[3] HL[3]
#define POSITION_LIMIT_MIN  (u32)-1000000000                // 最小位置极限         VL[3] LL[3]

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
#define UM_TCM              0x01                      // Torque control mode,力矩控制模式
#define UM_PCM              0x05                      // Position control mode，位置控制模式
#define UM_UNC              0x06                      // 不能确定模式，适用于全部电机调用

#if JV_IN_PCM
   #define UM_SCM           0x05                      // Position control mode，
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
//根据选择的电机ID，为电机建立缓存队列
/*****************************************************************************/

/*电机选择*/
#define ID_0    //表示所有的电机 
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
* 结构体
*******************************************************************************/
/* CAN循环队列元素 */ 
typedef struct __CANDATA 
{
   u16 COBID;               // CANopen COB identifier + NodeID
   u8  DLC;                 // Data Length Code
   u8  DATA[8];             // Data
} CANDATA;

/* CAN循环队列结构体 */
typedef struct __CANQUEUE
{
   u16 Front;        
   u16 Rear;
   CANDATA CANBUF[CAN_BUF_NUM];
} CANQUEUE;

/* Elmo结构体,记录节点ID,状态和控制参数 */
typedef struct __Elmo
{
   u8 NodeID;         // elmo结点号
   u8 CurOPMode;      // 当前运行模式 
}Elmo;
/*数据转化共用体*/
typedef union 
{
    uint32_t u32_form;
    int32_t  s32_form;
    uint8_t  u8_form[4];
    int8_t   s8_form[4];
    float    float_form;
}DataConvert;


extern u8 CAN_State;													//CAN2状态
extern u32 Elmo_Init_Flag;

extern CanTxMsg CAN1TxMessage;
extern CanTxMsg CAN2TxMessage;

/*******************************************************************************
* 函数声明
*******************************************************************************/
/* ELMO初始化函数,对外调用 */
extern u32 Elmo_Init( CAN_TypeDef* CANx, uint8_t PPr, uint8_t SPr);
extern void Elmo_Reinit(u8 elmoID);
u32 Elmo_Init_Fake(CAN_TypeDef* CANx, uint8_t PPr, uint8_t SPr);

/* ELMO控制函数，对外调用 */
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


/* CANOpen的实现函数,不对外调用 */
static void NMTCmd(Elmo *elmo, u8 MNTCmd);
static void RSDO(Elmo *elmo, u16 Index, u8 SubIndex, u32 Data);
static void RPDO2_Cmd_data(Elmo *elmo, u8 *Cmd, u8 Index, u8 Type, u32 Data);
static void RPDO2_Cmd_string(Elmo *elmo, u8 *Cmd);

/* 硬件初始化函数,不对外调用 */
void CAN_init(CAN_TypeDef* CANx);   
static void TIM7_init(uint8_t PPr, uint8_t SPr);
static int Self_test(void);
static void Variate_init(void);

/* 数据发送、转换与延时函数,不对外调用 */
static void Elmo_SendCmd(void);
static void Elmo_CANSend(CANDATA *pCANDATA);
static void Elmo_Delay100us_IDx( Elmo *elmo , u8 N100us);
static void Elmo_software_delay_ms(unsigned int t);
static u32 f2h(float x);


/*对外调用*/

//u8 CAN1_Send_Msg(u8* msg,u8 len,int id);
u8 CAN2_Send_Msg(u8* msg,u8 len,int id);
void SendCmd2Chassis(u8 Identifier);

#endif

