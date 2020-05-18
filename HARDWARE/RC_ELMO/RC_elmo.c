#include "RC_elmo.h"



#ifdef ID_0 
  static CANQUEUE QUEUE_CAN_ID0;
#endif
#ifdef ID_1
  static CANQUEUE QUEUE_CAN_ID1;
#endif
#ifdef ID_2
  static CANQUEUE QUEUE_CAN_ID2;
#endif
#ifdef ID_3
  static CANQUEUE QUEUE_CAN_ID3;
#endif
#ifdef ID_4
  static CANQUEUE QUEUE_CAN_ID4;
#endif
#ifdef ID_5
  static CANQUEUE QUEUE_CAN_ID5;
#endif
#ifdef ID_6
  static CANQUEUE QUEUE_CAN_ID6;
#endif
#ifdef ID_7
  static CANQUEUE QUEUE_CAN_ID7;
#endif
#ifdef ID_8
  static CANQUEUE QUEUE_CAN_ID8;
#endif
#ifdef ID_9
  static CANQUEUE QUEUE_CAN_ID9;
#endif
#ifdef ID_10
  static CANQUEUE QUEUE_CAN_ID10;
#endif
#ifdef ID_11
  static CANQUEUE QUEUE_CAN_ID11;
#endif
#ifdef ID_12
  static CANQUEUE QUEUE_CAN_ID12;
#endif
#ifdef ID_13
  static CANQUEUE QUEUE_CAN_ID13;
#endif
#ifdef ID_14
  static CANQUEUE QUEUE_CAN_ID14;
#endif
#ifdef ID_15
  static CANQUEUE QUEUE_CAN_ID15;
#endif

CANQUEUE *QUEUE_CAN_IDx;         //ָ����е�ָ��

static Elmo elmo[ELMO_NUM + 1];
static Elmo elmogroup;
static CAN_TypeDef* can;         //��ʼ��ѡ��CANx�Ľӿڱ���
static u8 CAN_Error = 0;         //���CAN����ʧ�ܣ�CAN_Error��1


u8 CAN_State;
u32 Elmo_Init_Flag;


/*
********************************************************************************
  *@  name      : CAN_init
  *@  function  : Initialization for CAN
  *@  input     : CANx      CAN1 or CAN2
  *@			  PPr       TIM7����ռ���ȼ�
  *@              SPr       TIM7�Ĵ����ȼ�
  *@  output    : 0	        ��ʼ���ɹ�
  *@              1         ��ʼ��ʧ��
********************************************************************************
*/
u32 Elmo_Init(CAN_TypeDef* CANx, uint8_t PPr, uint8_t SPr)
{  
	u8 i = 0;
	int temp = 0;
	/* Ӳ����ȫ�ֱ�����ʼ�� */
//	CAN_init( CANx );
 	TIM7_init( PPr, SPr);

	/*���������ʼ��*/
	Variate_init();

	/* ��elmo����ڵ�ID */
	for(i=0; i <= ELMO_NUM; i++)
	{
		elmo[i].NodeID = i;
	}

	/* ��elmo������ID */
	elmogroup.NodeID = 64;

	/* ��ȫ��ڵ����ͨ�Ÿ�λ */
	for(i=1; i <= ELMO_NUM; i++)
	{
		NMTCmd(&elmo[i], NMT_RESET_COMMUNICATION);
		Elmo_Delay100us_IDx(&elmo[i],50);
	}
	
	/* �Լ칦�ܺ�����CAN����ELMO��ʼ���ɹ���������ֵ0�������ӦELMOλ��0���������û�����ߣ�����0x80000000 */
	if((temp = Self_test()) == 0x80000000 )
	{
		return temp;		
	}
	

	//	/* �ȴ�Elmo�������,�����յ�Boot up���� */
	//	Elmo_Delay100us_IDx(&elmo[elmoID],50);

	/* CANOpenͨ�Ų�����ʼ�� */
	/* RPDO1->0x6040,ָ����,2�ֽ�,�첽���� */
	// ������Ĭ��ӳ��,����Ҫ�޸� //

	/* RPDO2->0x2012,�����Ʊ�������,4�ֽ�,�첽���� */
	// ������Ĭ��ӳ��,����Ҫ�޸� //

	/* ����TPDO,Debugʱ����,������ʱ��ùر� */
	RSDO(&elmo[0], 0x1A00, 0x00, 0);//����PDO1
	Elmo_Delay100us_IDx(&elmo[0],150);
	RSDO(&elmo[0], 0x1A01, 0x00, 0);//����PDO2 ���ʹ�õ���Pdo2
	Elmo_Delay100us_IDx(&elmo[0],150);

	/* ����NMT����״̬ */
	Elmo_Delay100us_IDx(&elmo[0],40);
	NMTCmd(&elmo[0], NMT_ENTER_OPERATIONAL);
	Elmo_Delay100us_IDx(&elmo[0],40);

	/* �ر����� */
	RPDO2_Cmd_data(&elmo[0], (u8 *)"MO", 0, TYPE_INTEGER, MO_OFF);
	Elmo_Delay100us_IDx(&elmo[0],80);

	/* ��ʼ�����ٶ� */
	RPDO2_Cmd_data(&elmo[0], (u8 *)"PM", 0, TYPE_INTEGER, 0x01);
	Elmo_Delay100us_IDx(&elmo[0],40);	
	RPDO2_Cmd_data(&elmo[0], (u8 *)"AC", 0, TYPE_INTEGER, 10000000);
	Elmo_Delay100us_IDx(&elmo[0],40);
	RPDO2_Cmd_data(&elmo[0], (u8 *)"DC", 0, TYPE_INTEGER, 10000000);
	Elmo_Delay100us_IDx(&elmo[0],40);

	/* Enter in SCM */
	RPDO2_Cmd_data(&elmo[0], (u8 *)"UM", 0, TYPE_INTEGER, UM_SCM);
	Elmo_Delay100us_IDx(&elmo[0],40);
	elmogroup.CurOPMode = UM_SCM;
	for(i=0;i<=ELMO_NUM;i++)
	{
		elmo[i].CurOPMode = UM_SCM;	
	}

	/* ʹ������ */
	RPDO2_Cmd_data(&elmo[0], (u8 *)"MO", 0, TYPE_INTEGER, MO_ON);
	Elmo_Delay100us_IDx(&elmo[0],250);
	Elmo_Delay100us_IDx(&elmo[0],250);
	Elmo_Delay100us_IDx(&elmo[0],50);
	return temp;
}

u32 Elmo_Init_Fake(CAN_TypeDef* CANx, uint8_t PPr, uint8_t SPr)
{
	u8 i = 0;
	/* Ӳ����ȫ�ֱ�����ʼ�� */ 
	CAN_init( CANx );
	TIM7_init( PPr, SPr);

	/*���������ʼ��*/
	Variate_init();

	/* ��elmo����ڵ�ID */
	for(i=0; i <= ELMO_NUM; i++)
	{
		elmo[i].NodeID = i;			
	}

	/* ��elmo������ID */
	elmogroup.NodeID = 64;
	return 0;
}

void Elmo_Reinit(u8 elmoID)
{
    
    RSDO(&elmo[elmoID], 0x6040, 0x00, 0x80);  //��������־λ
    Elmo_Delay100us_IDx(&elmo[elmoID],20);
    
    RSDO(&elmo[elmoID], 0x2f41, 0x00, 0x02);     //  ����PPM 
    Elmo_Delay100us_IDx(&elmo[elmoID],20);
    
    NMTCmd(&elmo[elmoID], NMT_RESET_COMMUNICATION);
    Elmo_Delay100us_IDx(&elmo[elmoID],30);
    
    /* ����NMT����״̬ */
	NMTCmd(&elmo[elmoID], NMT_ENTER_OPERATIONAL);
	Elmo_Delay100us_IDx(&elmo[elmoID],30);
    
    /* ��ʼ�����ٶ� */
	RPDO2_Cmd_data(&elmo[0], (u8 *)"PM", 0, TYPE_INTEGER, 0x01);
	Elmo_Delay100us_IDx(&elmo[elmoID],20); //�����ٶ�1000000000 1000000
	RPDO2_Cmd_data(&elmo[0], (u8 *)"AC", 0, TYPE_INTEGER, 1000000);
	Elmo_Delay100us_IDx(&elmo[elmoID],40); //�����ٶ�1000000000
	RPDO2_Cmd_data(&elmo[0], (u8 *)"DC", 0, TYPE_INTEGER, 1000000);
	Elmo_Delay100us_IDx(&elmo[elmoID],40);             
    
    /* �ر����� */
	RPDO2_Cmd_data(&elmo[elmoID], (u8 *)"MO", 0, TYPE_INTEGER, MO_OFF);
	Elmo_Delay100us_IDx(&elmo[elmoID],30);
    
    /* ʹ������ */
	RPDO2_Cmd_data(&elmo[elmoID], (u8 *)"MO", 0, TYPE_INTEGER, MO_ON);
	Elmo_Delay100us_IDx(&elmo[elmoID],60);
}
/*
********************************************************************************
  *@  name      : Elmo_PTM
  *@  function  : ��������ģʽ����
  *@  input     : elmoID    ȡelmo�ڵ�ID,�����ڵ��̿����е���elmo[1]~elmo[4]
  *@ 			  torque    Ŀ��ת��(A)
  *@  output    : 0         �������óɹ�
  *@              1         ��������ʧ��
********************************************************************************
*/
u8 Elmo_PTM(u8 elmoID, float torque)
{
    u8 i = 0;
	
	/* ���ʹ�ù㲥����, ��ǰģʽ��Ϊ����ģʽ,�л�Ϊ����ģʽ*/
	if(elmoID == 0 && elmo[elmoID].CurOPMode != UM_TCM)  
	{
		if( elmo[elmoID].CurOPMode != UM_IDLE)
		{
			RPDO2_Cmd_data( &elmo[elmoID], (u8 *)"MO", 0, TYPE_INTEGER, MO_OFF);
			Elmo_Delay100us_IDx(&elmo[elmoID],100);
		}		
		
		/* ������״̬������ֵΪTCM */
		elmogroup.CurOPMode = UM_TCM;			
		for(i=0;i<=ELMO_NUM;i++)
		{
			elmo[i].CurOPMode = UM_TCM;			
		}
		
		RPDO2_Cmd_data( &elmo[elmoID], (u8 *)"UM", 0, TYPE_INTEGER, UM_TCM);
		Elmo_Delay100us_IDx(&elmo[elmoID],10);
		
		RPDO2_Cmd_data( &elmo[elmoID], (u8 *)"MO", 0, TYPE_INTEGER, MO_ON);
		Elmo_Delay100us_IDx(&elmo[elmoID],30);
	}  
	/* ʹ�õ���ELMO����,��ǰģʽ��Ϊ����ģʽ,�л�Ϊ����ģʽ */
 	else if( elmo[elmoID].CurOPMode != UM_TCM)         
	{
		if( elmo[elmoID].CurOPMode != UM_IDLE)
		{
			RPDO2_Cmd_data( &elmo[elmoID], (u8 *)"MO", 0, TYPE_INTEGER, MO_OFF);
			Elmo_Delay100us_IDx(&elmo[elmoID],100);
		}
		elmo[elmoID].CurOPMode = UM_TCM;
		RPDO2_Cmd_data( &elmo[elmoID], (u8 *)"UM", 0, TYPE_INTEGER, UM_TCM);
		Elmo_Delay100us_IDx(&elmo[elmoID],10);

		RPDO2_Cmd_data( &elmo[elmoID], (u8 *)"MO", 0, TYPE_INTEGER, MO_ON);
		Elmo_Delay100us_IDx(&elmo[elmoID],30);
		
        /* ���ELMOֻ������1����������ô�����ģʽ��elmo[0]��ģʽ����һ�� */		
		if(ELMO_NUM == 1)
		{
			elmo[0].CurOPMode = UM_TCM;
		}	
		/* �����ж��Ƿ����е��ģʽ��ͬ���Ӷ�ȷ��elmo[0]��ģʽ */
		else 
		{
			for(i=1;i<ELMO_NUM;i++)  
			{
				if( elmo[i].CurOPMode == elmo[i+1].CurOPMode )
				{
					elmo[0].CurOPMode = UM_TCM;
				}
				else
				{
					elmo[0].CurOPMode = UM_UNC;
					break;
				}
			}			
		}
	}
		
	/* ����Ŀ��ת�ز����е���ģʽ */
	RPDO2_Cmd_data( &elmo[elmoID], (u8 *)"TC", 0, TYPE_FLOAT, f2h(torque));
	Elmo_Delay100us_IDx(&elmo[elmoID],10);
	return 0;
}
/*
********************************************************************************
  *@  name      : Elmo_Pre_PVM
  *@  function  : �ٶ�ģʽԤ������
  *@  input     : elmoID   ȡelmo�ڵ�ID
  *@  output    : none
********************************************************************************
*/
void Elmo_Pre_PVM(u8 elmoID)
{
	 u8 i = 0;
	
	/* ���ʹ�ù㲥����,��ǰģʽ��Ϊ�ٶ�ģʽ,�л�Ϊ�ٶ�ģʽ */
	if(elmoID == 0 && elmo[elmoID].CurOPMode != UM_SCM)
	{
		if(elmo[elmoID].CurOPMode != UM_IDLE)
		{
			RPDO2_Cmd_data(&elmo[elmoID], (u8 *)"MO", 0, TYPE_INTEGER, MO_OFF);
			Elmo_Delay100us_IDx(&elmo[elmoID],100);
		}	
		
		/* ������״̬������ֵΪSCM */
		elmogroup.CurOPMode = UM_SCM;			
		for(i=0;i<=ELMO_NUM;i++)
		{
			elmo[i].CurOPMode = UM_SCM;			
		}

		RPDO2_Cmd_data(&elmo[elmoID], (u8 *)"UM", 0, TYPE_INTEGER, UM_SCM);
		Elmo_Delay100us_IDx(&elmo[elmoID],10);

	    RPDO2_Cmd_data(&elmo[elmoID], (u8 *)"MO", 0, TYPE_INTEGER, MO_ON);
		Elmo_Delay100us_IDx(&elmo[elmoID],30);
		
	}  
	/* ʹ�õ���ELMO����,��ǰģʽ��Ϊ�ٶ�ģʽ,�л�Ϊ�ٶ�ģʽ*/
	else if(elmo[elmoID].CurOPMode != UM_SCM)
	{
		if(elmo[elmoID].CurOPMode != UM_IDLE)
		{
			RPDO2_Cmd_data(&elmo[elmoID], (u8 *)"MO", 0, TYPE_INTEGER, MO_OFF);
			Elmo_Delay100us_IDx(&elmo[elmoID],100);
		}

		elmo[elmoID].CurOPMode = UM_SCM;
		RPDO2_Cmd_data(&elmo[elmoID], (u8 *)"UM", 0, TYPE_INTEGER, UM_SCM);
		Elmo_Delay100us_IDx(&elmo[elmoID],10);

	    RPDO2_Cmd_data(&elmo[elmoID], (u8 *)"MO", 0, TYPE_INTEGER, MO_ON);
		Elmo_Delay100us_IDx(&elmo[elmoID],30);
	
        /* ���ELMOֻ������1����������ô�����ģʽ��elmo[0]��ģʽ����һ�� */				
		if(ELMO_NUM == 1)       
		{
			elmo[0].CurOPMode = UM_SCM;
		}	
		/* �����ж��Ƿ����е��ģʽ��ͬ���Ӷ�ȷ��elmo[0]��ģʽ */
		else
		{
			for(i=1;i<ELMO_NUM;i++) 
			{
				if( elmo[i].CurOPMode == elmo[i+1].CurOPMode )
				{
					elmo[0].CurOPMode = UM_SCM;
				}
				else
				{
					elmo[0].CurOPMode = UM_UNC;
					break;
				}
			}				
		}
	}
}

/*
********************************************************************************
  *@  name      : Elmo_PVM
  *@  function  : �����ٶ�ģʽ����
  *@  input     : elmoID   ȡelmo�ڵ�ID
  *@			  speed    Ŀ���ٶ�(cnt/s)
  *@  output    : 0         �������óɹ�
  *@              1         ��������ʧ��
********************************************************************************
*/


u8 Elmo_PVM(u8 elmoID, s32 speed)
{
	 u8 i = 0;

	
	/* ���ʹ�ù㲥����,��ǰģʽ��Ϊ�ٶ�ģʽ,�л�Ϊ�ٶ�ģʽ */
	if(elmoID == 0 && elmo[elmoID].CurOPMode != UM_SCM)
	{
		if(elmo[elmoID].CurOPMode != UM_IDLE)
		{
			RPDO2_Cmd_data(&elmo[elmoID], (u8 *)"MO", 0, TYPE_INTEGER, MO_OFF);
			Elmo_Delay100us_IDx(&elmo[elmoID],100);
		}	
		
		/* ������״̬������ֵΪSCM */
		elmogroup.CurOPMode = UM_SCM;			
		for(i=0;i<=ELMO_NUM;i++)
		{
			elmo[i].CurOPMode = UM_SCM;			
		}

		RPDO2_Cmd_data(&elmo[elmoID], (u8 *)"UM", 0, TYPE_INTEGER, UM_SCM);
		Elmo_Delay100us_IDx(&elmo[elmoID],10);

	    RPDO2_Cmd_data(&elmo[elmoID], (u8 *)"MO", 0, TYPE_INTEGER, MO_ON);
		Elmo_Delay100us_IDx(&elmo[elmoID],30);
		
	}  
	/* ʹ�õ���ELMO����,��ǰģʽ��Ϊ�ٶ�ģʽ,�л�Ϊ�ٶ�ģʽ*/
	else if(elmo[elmoID].CurOPMode != UM_SCM)
	{
		if(elmo[elmoID].CurOPMode != UM_IDLE)
		{
			RPDO2_Cmd_data(&elmo[elmoID], (u8 *)"MO", 0, TYPE_INTEGER, MO_OFF);
			Elmo_Delay100us_IDx(&elmo[elmoID],100);
		}

		elmo[elmoID].CurOPMode = UM_SCM;
		RPDO2_Cmd_data(&elmo[elmoID], (u8 *)"UM", 0, TYPE_INTEGER, UM_SCM);
		Elmo_Delay100us_IDx(&elmo[elmoID],10);

	    RPDO2_Cmd_data(&elmo[elmoID], (u8 *)"MO", 0, TYPE_INTEGER, MO_ON);
		Elmo_Delay100us_IDx(&elmo[elmoID],30);
	
        /* ���ELMOֻ������1����������ô�����ģʽ��elmo[0]��ģʽ����һ�� */				
		if(ELMO_NUM == 1)       
		{
			elmo[0].CurOPMode = UM_SCM;
		}	
		/* �����ж��Ƿ����е��ģʽ��ͬ���Ӷ�ȷ��elmo[0]��ģʽ */
		else
		{
			for(i=1;i<ELMO_NUM;i++) 
			{
				if( elmo[i].CurOPMode == elmo[i+1].CurOPMode )
				{
					elmo[0].CurOPMode = UM_SCM;
				}
				else
				{
					elmo[0].CurOPMode = UM_UNC;
					break;
				}
			}				
		}
	}

	/* ����Ŀ���ٶ� */
	RPDO2_Cmd_data(&elmo[elmoID], (u8 *)"JV", 0, TYPE_INTEGER, speed);
	Elmo_Delay100us_IDx(&elmo[elmoID],10);

	/* �����ٶ�ģʽ */
	RPDO2_Cmd_string(&elmo[elmoID], (u8 *)"BG");
	Elmo_Delay100us_IDx(&elmo[elmoID],10);
	return 0;
}
/*
********************************************************************************
  *@  name      : Four_Elmo_PVM
  *@  function  : �����ٶ�ģʽ����
  *@  input     : elmoID   ȡelmo�ڵ�ID
  *@			  speed    Ŀ���ٶ�(cnt/s)
  *@  output    : 0         �������óɹ�
  *@              1         ��������ʧ��
********************************************************************************
*/
void Four_Elmo_PVM(u8 elmoID1, s32 speed1,u8 elmoID2, s32 speed2,u8 elmoID3, s32 speed3,u8 elmoID4, s32 speed4)
{
	/* ����Ŀ���ٶ� */
	RPDO2_Cmd_data(&elmo[elmoID1], (u8 *)"JV", 0, TYPE_INTEGER, speed1);
	RPDO2_Cmd_data(&elmo[elmoID2], (u8 *)"JV", 0, TYPE_INTEGER, speed2);
	RPDO2_Cmd_data(&elmo[elmoID3], (u8 *)"JV", 0, TYPE_INTEGER, speed3);
	RPDO2_Cmd_data(&elmo[elmoID4], (u8 *)"JV", 0, TYPE_INTEGER, speed4);
	Elmo_Delay100us_IDx(&elmo[elmoID1],10);
	Elmo_Delay100us_IDx(&elmo[elmoID2],10);
	Elmo_Delay100us_IDx(&elmo[elmoID3],10);
	Elmo_Delay100us_IDx(&elmo[elmoID4],10);

	/* �����ٶ�ģʽ */
	RPDO2_Cmd_string(&elmo[elmoID1], (u8 *)"BG");
	RPDO2_Cmd_string(&elmo[elmoID2], (u8 *)"BG");
	RPDO2_Cmd_string(&elmo[elmoID3], (u8 *)"BG");
	RPDO2_Cmd_string(&elmo[elmoID4], (u8 *)"BG");
	Elmo_Delay100us_IDx(&elmo[elmoID1],20);
	Elmo_Delay100us_IDx(&elmo[elmoID2],20);
	Elmo_Delay100us_IDx(&elmo[elmoID3],20);
	Elmo_Delay100us_IDx(&elmo[elmoID4],20);
}

/*
********************************************************************************
  *@  name      : Elmo_RunPPM
  *@  function  : ����ƽ���ٶȵ�λ��ģʽ
  *@  input     : elmoID    ȡelmo�ڵ�ID
  *@              speed     �ٶ�(cnt/s)
  *@              position  Ŀ��λ��(cnt)
  *@              PPMmode   ����ģʽ
  *@				    POS_ABS // PPM���з�ʽ:����λ��
  *@				    POS_REL // PPM���з�ʽ:���λ��
  *@  output    : 0         �������óɹ�
  *@              1         ��������ʧ��
********************************************************************************
*/
u8 Elmo_PPM(u8 elmoID, u32 speed, s32 position, u8 PPMmode)
{
	u8 i = 0;

	//����*******************************
//	#ifdef CASE5_ENABLE
//	if(elmoID == 8 && Im_Arm.Arm_Init_success == 1)
//	{
//		SLIDEWAY_ELMO_SAVE[SLIDEWAY_ELMO_SAVE_count].mode=2;
//		SLIDEWAY_ELMO_SAVE[SLIDEWAY_ELMO_SAVE_count].speed=0;
//		SLIDEWAY_ELMO_SAVE[SLIDEWAY_ELMO_SAVE_count].pos=position;
//		SLIDEWAY_ELMO_SAVE[SLIDEWAY_ELMO_SAVE_count].OS_t=OSTime;
//		SLIDEWAY_ELMO_SAVE_count++;
//		if(SLIDEWAY_ELMO_SAVE_count>=SLIDEWAY_ELMO_SAVE_Max)SLIDEWAY_ELMO_SAVE_count=SLIDEWAY_ELMO_SAVE_Max;  //��ֹԽ��
//	}
//		if(elmoID == 10 && Im_Arm.Arm_Init_success == 1)
//	{
//		SLIDEWAY_ELMO_SAVE_Y[SLIDEWAY_ELMO_SAVE_count_Y].mode=2;
//		SLIDEWAY_ELMO_SAVE_Y[SLIDEWAY_ELMO_SAVE_count_Y].speed=speed;
//		SLIDEWAY_ELMO_SAVE_Y[SLIDEWAY_ELMO_SAVE_count_Y].pos=0;
//		SLIDEWAY_ELMO_SAVE_Y[SLIDEWAY_ELMO_SAVE_count_Y].OS_t=OSTime;
//		SLIDEWAY_ELMO_SAVE_count_Y++;
//		if(SLIDEWAY_ELMO_SAVE_count_Y>=SLIDEWAY_ELMO_SAVE_Max)SLIDEWAY_ELMO_SAVE_count_Y=SLIDEWAY_ELMO_SAVE_Max;  //��ֹԽ��
//	}
//	#endif
		//����*******************************

	/* ���ʹ�ù㲥����,��ǰģʽ��Ϊλ��ģʽ,�л�Ϊλ��ģʽ */
	if(elmoID == 0 && elmo[elmoID].CurOPMode != UM_PCM)
	{
		if(elmo[elmoID].CurOPMode != UM_IDLE)
		{
			RPDO2_Cmd_data(&elmo[elmoID], (u8 *)"MO", 0, TYPE_INTEGER, MO_OFF);
			Elmo_Delay100us_IDx(&elmo[elmoID],90);
		}	
		/* ������״̬������ֵΪPCM */		
		elmogroup.CurOPMode = UM_PCM;			
		for(i=0;i<=ELMO_NUM;i++)
		{
			elmo[i].CurOPMode = UM_PCM;			
		}

		RPDO2_Cmd_data(&elmo[elmoID], (u8 *)"UM", 0, TYPE_INTEGER, UM_PCM);
		Elmo_Delay100us_IDx(&elmo[elmoID],20);
		
		RPDO2_Cmd_data(&elmo[elmoID], (u8 *)"MO", 0, TYPE_INTEGER, MO_ON);
		Elmo_Delay100us_IDx(&elmo[elmoID],30);
	}  
	/* ʹ�õ���ELMO����,��ǰģʽ��Ϊλ��ģʽ,�л�Ϊλ��ģʽ */
	else if(elmo[elmoID].CurOPMode != UM_PCM)
	{
		if(elmo[elmoID].CurOPMode != UM_IDLE)
		{
			RPDO2_Cmd_data(&elmo[elmoID], (u8 *)"MO", 0, TYPE_INTEGER, MO_OFF);
			Elmo_Delay100us_IDx(&elmo[elmoID],90);
		}

		elmo[elmoID].CurOPMode = UM_PCM;
		RPDO2_Cmd_data(&elmo[elmoID], (u8 *)"UM", 0, TYPE_INTEGER, UM_PCM);
		Elmo_Delay100us_IDx(&elmo[elmoID],200);

		RPDO2_Cmd_data(&elmo[elmoID], (u8 *)"MO", 0, TYPE_INTEGER, MO_ON);
		Elmo_Delay100us_IDx(&elmo[elmoID],30);


        /* ���ELMOֻ������1����������ô�����ģʽ��elmo[0]��ģʽ����һ�� */				
		if(ELMO_NUM == 1)      
		{
			elmo[0].CurOPMode = UM_PCM;
		}		
		/* �����ж��Ƿ����е��ģʽ��ͬ���Ӷ�ȷ��elmo[0]��ģʽ */
		else 
		{
			for(i=1;i<ELMO_NUM;i++) 
			{
				if( elmo[i].CurOPMode == elmo[i+1].CurOPMode )
				{
					elmo[0].CurOPMode = UM_PCM;
				}
				else
				{
					elmo[0].CurOPMode = UM_UNC;
					break;
				}
			}			
		}
	}
  //RPDO2_Cmd_data(Elmo *elmo, u8 *Cmd, u8 Index, u8 Type, u32 Data)
	/* ����Ŀ���ٶ� */
	RPDO2_Cmd_data(&elmo[elmoID], (u8 *)"SP", 0, TYPE_INTEGER, speed);
	Elmo_Delay100us_IDx(&elmo[elmoID],20);

	/* ��������ģʽ����λ�� */
	if(PPMmode)
	{
		RPDO2_Cmd_data(&elmo[elmoID], (u8 *)"PA", 0, TYPE_INTEGER, position);
		Elmo_Delay100us_IDx(&elmo[elmoID],20);
	}
	else
	{
		RPDO2_Cmd_data(&elmo[elmoID], (u8 *)"PR", 0, TYPE_INTEGER, position);
		Elmo_Delay100us_IDx(&elmo[elmoID],20);
	}
	
	/* ����λ��ģʽ */
	RPDO2_Cmd_string(&elmo[elmoID], (u8 *)"BG");
	Elmo_Delay100us_IDx(&elmo[elmoID],10);
	return 0;
}


/*
********************************************************************************
  *@  name      : Elmo_Close
  *@  function  : ��������رգ���������Լ�����ʻ
  *@  input     : elmoID    ȡelmo�ڵ�ID,�����ڵ��̿����е���elmo[1]~elmo[4]        
  *@  output    : 0         �������óɹ�
  *@              1         ��������ʧ��
********************************************************************************
*/
u8 Elmo_Close(u8 elmoID)
{
	u8 i = 0;
	
    /* ���ʹ�ù㲥����,������ELMOʧ�� */
	if(elmoID == 0)
	{
		RPDO2_Cmd_data(&elmo[elmoID], (u8 *)"MO", 0, TYPE_INTEGER, MO_OFF);
		Elmo_Delay100us_IDx(&elmo[elmoID],100);
		
		/* ������״̬������ֵΪIDLE */		
		elmogroup.CurOPMode = UM_IDLE;			
		for(i=0;i<=ELMO_NUM;i++)
		{
			elmo[i].CurOPMode = UM_IDLE;			
		}	    
	}
    /* ʹ�õ���ELMO���� */
	else 
	{
		RPDO2_Cmd_data(&elmo[elmoID], (u8 *)"MO", 0, TYPE_INTEGER, MO_OFF);
		Elmo_Delay100us_IDx(&elmo[elmoID],100);
		elmo[elmoID].CurOPMode = UM_IDLE;		

        /* ���ELMOֻ������1����������ô�����ģʽ��elmo[0]��ģʽ����һ�� */			
		if(ELMO_NUM == 1)       
		{
			elmo[0].CurOPMode = UM_IDLE;
		}		
		/* �����ж��Ƿ����е��ģʽ��ͬ���Ӷ�ȷ��elmo[0]��ģʽ */
		else 
		{
			for(i=1;i<ELMO_NUM;i++)  
			{
				if( elmo[i].CurOPMode == elmo[i+1].CurOPMode )
				{
					elmo[0].CurOPMode = UM_IDLE;
				}
				else
				{
					elmo[0].CurOPMode = UM_UNC;
					break;
				}
			}			
		}
	}
	return 0;
}


/*
********************************************************************************
  *@  name      : Elmo_Stop
  *@  function  : ɲ�����������
  *@  input     : elmoID      ȡelmo�ڵ�ID,�����ڵ��̿����е���elmo[1]~elmo[4]        
  *@  output    : 0         �������óɹ�
  *@              1         ��������ʧ��
********************************************************************************
*/
u8 Elmo_Stop(u8 elmoID)
{
	u8 i = 0;

		//����*******************************
	#ifdef CASE5_ENABLE
	if(elmoID == 8 && Im_Arm.Arm_Init_success == 1)
	{
		SLIDEWAY_ELMO_SAVE[SLIDEWAY_ELMO_SAVE_count].mode=3;
		SLIDEWAY_ELMO_SAVE[SLIDEWAY_ELMO_SAVE_count].speed=0;
		SLIDEWAY_ELMO_SAVE[SLIDEWAY_ELMO_SAVE_count].pos=0;
		SLIDEWAY_ELMO_SAVE[SLIDEWAY_ELMO_SAVE_count].OS_t=OSTime;
		SLIDEWAY_ELMO_SAVE_count++;
		if(SLIDEWAY_ELMO_SAVE_count>=SLIDEWAY_ELMO_SAVE_Max)SLIDEWAY_ELMO_SAVE_count=SLIDEWAY_ELMO_SAVE_Max;  //��ֹԽ��
	}
		if(elmoID == 10 && Im_Arm.Arm_Init_success == 1)
	{
		SLIDEWAY_ELMO_SAVE_Y[SLIDEWAY_ELMO_SAVE_count_Y].mode=3;
		SLIDEWAY_ELMO_SAVE_Y[SLIDEWAY_ELMO_SAVE_count_Y].speed=0;
		SLIDEWAY_ELMO_SAVE_Y[SLIDEWAY_ELMO_SAVE_count_Y].pos=0;
		SLIDEWAY_ELMO_SAVE_Y[SLIDEWAY_ELMO_SAVE_count_Y].OS_t=OSTime;
		SLIDEWAY_ELMO_SAVE_count_Y++;
		if(SLIDEWAY_ELMO_SAVE_count_Y>=SLIDEWAY_ELMO_SAVE_Max)SLIDEWAY_ELMO_SAVE_count_Y=SLIDEWAY_ELMO_SAVE_Max;  //��ֹԽ��
	}
	#endif
	/* ���CAN������������Ŀ���ģʽ����ͬ���ҷ����㲥ָ����ιر� */
	if(elmoID == 0 && elmo[elmoID].CurOPMode == UM_UNC)  
	{
		/* ѭ���жϸ��������״̬�����ر�ÿһ����� */
		for(i=1;i<=ELMO_NUM;i++)            
		{		
			/* ��ǰģʽΪ�ͷŵ��,�ȴ򿪵�����ٱ��� */		
			if (elmo[i].CurOPMode == UM_IDLE) 
			{
				RPDO2_Cmd_data(&elmo[i], (u8 *)"MO", 0, TYPE_INTEGER, MO_ON);
				Elmo_Delay100us_IDx(&elmo[elmoID],30);				
			}
			/* ��ǰģʽΪ����ģʽ,���л�Ϊ�ٶ�ģʽ���ٹر� */
			else if(elmo[i].CurOPMode == UM_TCM)
			{
				RPDO2_Cmd_data(&elmo[i], (u8 *)"MO", 0, TYPE_INTEGER, MO_OFF);	
				Elmo_Delay100us_IDx(&elmo[i],200);	
				Elmo_Delay100us_IDx(&elmo[i],200);	
				Elmo_Delay100us_IDx(&elmo[i],200);	
				Elmo_Delay100us_IDx(&elmo[i],200);			
				Elmo_Delay100us_IDx(&elmo[i],200);	
				Elmo_Delay100us_IDx(&elmo[i],200);	
				Elmo_Delay100us_IDx(&elmo[i],200);	
				Elmo_Delay100us_IDx(&elmo[i],200);	
				Elmo_Delay100us_IDx(&elmo[i],200);			
				Elmo_Delay100us_IDx(&elmo[i],200);	
				Elmo_Delay100us_IDx(&elmo[i],200);	
				Elmo_Delay100us_IDx(&elmo[i],200);	
				Elmo_Delay100us_IDx(&elmo[i],200);			

				elmo[i].CurOPMode = UM_SCM;			

				RPDO2_Cmd_data(&elmo[i], (u8 *)"UM", 0, TYPE_INTEGER, UM_SCM);
				Elmo_Delay100us_IDx(&elmo[i],10);

				RPDO2_Cmd_data(&elmo[i], (u8 *)"MO", 0, TYPE_INTEGER, MO_ON);
				Elmo_Delay100us_IDx(&elmo[i],30);

				/* ����Ŀ���ٶ� */
				RPDO2_Cmd_data(&elmo[i], (u8 *)"JV", 0, TYPE_INTEGER, 0);
				Elmo_Delay100us_IDx(&elmo[i],10);

				/* �����ٶ�ģʽ */
				RPDO2_Cmd_string(&elmo[i], (u8 *)"BG");
				Elmo_Delay100us_IDx(&elmo[i],10);
			}	
				
			RPDO2_Cmd_string(&elmo[i], (u8 *)"ST");
			Elmo_Delay100us_IDx(&elmo[i],10);
		}	
		
		/* �����ж��Ƿ����е��ģʽ��ͬ���Ӷ�ȷ��elmo[0]��ģʽ,�ܹ��������if�Ĳ���������CAN����ֻ�е��� */
		for(i=1;i<ELMO_NUM;i++)  
		{
			if( elmo[i].CurOPMode == elmo[i+1].CurOPMode )
			{
				elmo[0].CurOPMode = UM_SCM;         //�����STOPģʽ������ģʽͳһ�ˣ�ֻ�п�����SCMģʽ
			}
			else
			{
				elmo[0].CurOPMode = UM_UNC;
				break;
			}
		}											
	}
	/* �����㲥ָ����������϶���Ŀ���ģʽ����ͬ����ʱ�����һ��ر� */
    else if(elmoID == 0 && elmo[elmoID].CurOPMode != UM_UNC)  
	{
		/* ��ǰģʽΪ�ͷŵ��,�ȴ򿪵�����ٱ��� */		
		if (elmo[elmoID].CurOPMode == UM_IDLE) 
		{
			RPDO2_Cmd_data(&elmo[elmoID], (u8 *)"MO", 0, TYPE_INTEGER, MO_ON);
			Elmo_Delay100us_IDx(&elmo[elmoID],30);				
		}
		/* ��ǰģʽΪ����ģʽ,���л�Ϊ�ٶ�ģʽ���ٹر� */
		else if(elmo[elmoID].CurOPMode == UM_TCM)
		{
			RPDO2_Cmd_data(&elmo[elmoID], (u8 *)"MO", 0, TYPE_INTEGER, MO_OFF);	
			Elmo_Delay100us_IDx(&elmo[elmoID],200);	
			Elmo_Delay100us_IDx(&elmo[elmoID],200);	
			Elmo_Delay100us_IDx(&elmo[elmoID],200);	
			Elmo_Delay100us_IDx(&elmo[elmoID],200);			
			Elmo_Delay100us_IDx(&elmo[elmoID],200);	
			Elmo_Delay100us_IDx(&elmo[elmoID],200);	
			Elmo_Delay100us_IDx(&elmo[elmoID],200);	
			Elmo_Delay100us_IDx(&elmo[elmoID],200);	
			Elmo_Delay100us_IDx(&elmo[elmoID],200);			
			Elmo_Delay100us_IDx(&elmo[elmoID],200);	
			Elmo_Delay100us_IDx(&elmo[elmoID],200);	
			Elmo_Delay100us_IDx(&elmo[elmoID],200);	
			Elmo_Delay100us_IDx(&elmo[elmoID],200);			

			/* �㲥ģʽ�£������ٶ�ģʽ����ΪSCM */
			for(i=0;i<=ELMO_NUM;i++) 
			{
				elmo[i].CurOPMode = UM_SCM;			
			}
		
			RPDO2_Cmd_data(&elmo[elmoID], (u8 *)"UM", 0, TYPE_INTEGER, UM_SCM);
			Elmo_Delay100us_IDx(&elmo[elmoID],10);

			RPDO2_Cmd_data(&elmo[elmoID], (u8 *)"MO", 0, TYPE_INTEGER, MO_ON);
			Elmo_Delay100us_IDx(&elmo[elmoID],30);

			/* ����Ŀ���ٶ� */
			RPDO2_Cmd_data(&elmo[elmoID], (u8 *)"JV", 0, TYPE_INTEGER, 0);
			Elmo_Delay100us_IDx(&elmo[elmoID],10);

			/* �����ٶ�ģʽ */
			RPDO2_Cmd_string(&elmo[elmoID], (u8 *)"BG");
			Elmo_Delay100us_IDx(&elmo[elmoID],10);
		}	
			
		RPDO2_Cmd_string(&elmo[elmoID], (u8 *)"ST");
		Elmo_Delay100us_IDx(&elmo[elmoID],10);	
        
    }
	/* ʹ�õ���ELMO���� */
	else    
	{
		/* ��ǰģʽΪ�ͷŵ��,�ȴ򿪵�����ٱ��� */		
		if (elmo[elmoID].CurOPMode == UM_IDLE) 
		{
			RPDO2_Cmd_data(&elmo[elmoID], (u8 *)"MO", 0, TYPE_INTEGER, MO_ON);
			Elmo_Delay100us_IDx(&elmo[elmoID],30);				
		}
		/* ��ǰģʽΪ����ģʽ,���л�Ϊ�ٶ�ģʽ���ٹر� */
		else if(elmo[elmoID].CurOPMode == UM_TCM)
		{
			RPDO2_Cmd_data(&elmo[elmoID], (u8 *)"MO", 0, TYPE_INTEGER, MO_OFF);	
			Elmo_Delay100us_IDx(&elmo[elmoID],200);	
			Elmo_Delay100us_IDx(&elmo[elmoID],200);	
			Elmo_Delay100us_IDx(&elmo[elmoID],200);	
			Elmo_Delay100us_IDx(&elmo[elmoID],200);			
			Elmo_Delay100us_IDx(&elmo[elmoID],200);	
			Elmo_Delay100us_IDx(&elmo[elmoID],200);	
			Elmo_Delay100us_IDx(&elmo[elmoID],200);	
			Elmo_Delay100us_IDx(&elmo[elmoID],200);	
			Elmo_Delay100us_IDx(&elmo[elmoID],200);			
			Elmo_Delay100us_IDx(&elmo[elmoID],200);	
			Elmo_Delay100us_IDx(&elmo[elmoID],200);	
			Elmo_Delay100us_IDx(&elmo[elmoID],200);	
			Elmo_Delay100us_IDx(&elmo[elmoID],200);			

			elmo[elmoID].CurOPMode = UM_SCM;			

			RPDO2_Cmd_data(&elmo[elmoID], (u8 *)"UM", 0, TYPE_INTEGER, UM_SCM);
			Elmo_Delay100us_IDx(&elmo[elmoID],10);

			RPDO2_Cmd_data(&elmo[elmoID], (u8 *)"MO", 0, TYPE_INTEGER, MO_ON);
			Elmo_Delay100us_IDx(&elmo[elmoID],30);

			/* ����Ŀ���ٶ� */
			RPDO2_Cmd_data(&elmo[elmoID], (u8 *)"JV", 0, TYPE_INTEGER, 0);
			Elmo_Delay100us_IDx(&elmo[elmoID],10);

			/* �����ٶ�ģʽ */
			RPDO2_Cmd_string(&elmo[elmoID], (u8 *)"BG");
			Elmo_Delay100us_IDx(&elmo[elmoID],10);

            /* ���ELMOֻ������1����������ô�����ģʽ��elmo[0]��ģʽ����һ�� */			
			for(i=1;i<ELMO_NUM;i++)  //�����ж��Ƿ����е��ģʽ��ͬ���Ӷ�ȷ��elmo[0]��ģʽ
			{
				if( elmo[i].CurOPMode == elmo[i+1].CurOPMode )
				{
					elmo[0].CurOPMode = UM_SCM;    //�����STOPģʽ������ģʽͳһ�ˣ�ֻ�п�����SCMģʽ
				}
				else
				{
					elmo[0].CurOPMode = UM_UNC;
					break;
				}
			}				
		}	
			
		RPDO2_Cmd_string(&elmo[elmoID], (u8 *)"ST");
		Elmo_Delay100us_IDx(&elmo[elmoID],10);		
	}
	return 0;		
}

/*
********************************************************************************
  *@  name      : Four_Elmo_Stop
  *@  function  : ɲ�����������
  *@  input     : elmoID      ȡelmo�ڵ�ID,�����ڵ��̿����е���elmo[1]~elmo[4]        
  *@  output    : 0         �������óɹ�
  *@              1         ��������ʧ��
********************************************************************************
*/
void Four_Elmo_Stop(u8 elmoID1,u8 elmoID2,u8 elmoID3,u8 elmoID4)
{
	RPDO2_Cmd_string(&elmo[elmoID1], (u8 *)"ST");
	RPDO2_Cmd_string(&elmo[elmoID2], (u8 *)"ST");
	RPDO2_Cmd_string(&elmo[elmoID3], (u8 *)"ST");
	RPDO2_Cmd_string(&elmo[elmoID4], (u8 *)"ST");
	Elmo_Delay100us_IDx(&elmo[elmoID1],10);
	Elmo_Delay100us_IDx(&elmo[elmoID2],10);
	Elmo_Delay100us_IDx(&elmo[elmoID3],10);
	Elmo_Delay100us_IDx(&elmo[elmoID4],10);
}

/*
********************************************************************************
  *@  name      : Elmo_SetAcc(u8 elmoID, u32 acc, u32 dec)
  *@  function  : �����ٶ�ģʽ��λ��ģʽ�ĵ���Ӽ���
  *@  input     : elmoID     ȡelmo�Ľڵ�ID
  *@                 acc        ���ٶ�,���ٶ�����ܳ���1000000000,ͬʱӦ���ǵ������
  *@                 dec        ���ٶ�,���ٶ�����ܳ���1000000000,ͬʱӦ���ǵ������
  *@  output    : 0         �������óɹ�
  *@              1         ��������ʧ��
********************************************************************************
*/
u8 Elmo_SetAcc(u8 elmoID, u32 acc, u32 dec)
{ 
	RPDO2_Cmd_data(&elmo[elmoID], (u8 *)"MO", 0, TYPE_INTEGER, MO_OFF);
	Elmo_Delay100us_IDx(&elmo[elmoID],30);	
	RPDO2_Cmd_data(&elmo[elmoID], (u8 *)"PM", 0, TYPE_INTEGER, 0x01);
	Elmo_Delay100us_IDx(&elmo[elmoID],10);	
	RPDO2_Cmd_data(&elmo[elmoID], (u8 *)"AC", 0, TYPE_INTEGER, acc);
	Elmo_Delay100us_IDx(&elmo[elmoID],10);
	RPDO2_Cmd_data(&elmo[elmoID], (u8 *)"DC", 0, TYPE_INTEGER, dec);
	Elmo_Delay100us_IDx(&elmo[elmoID],10);
	RPDO2_Cmd_data(&elmo[elmoID], (u8 *)"MO", 0, TYPE_INTEGER, MO_ON);
	Elmo_Delay100us_IDx(&elmo[elmoID],100);
	return 0;
}


/*
********************************************************************************
  *@  name      : NMTCmd
  *@  function  : ����CANOPEN��NMT״̬����
  *@  input     : elmo     ȡelmo�Ľڵ�ID
                  MNTCmd   NMTָ��,NMT_xxx
  *@  output    : None
********************************************************************************
*/
static void NMTCmd(Elmo *elmo, u8 MNTCmd)
{
	u16 tmp_rear;

	switch(elmo->NodeID)
	{
		#ifdef ID_0 
		    case 0:  QUEUE_CAN_IDx = &QUEUE_CAN_ID0;  break;
		#endif
		#ifdef ID_1
		    case 1:  QUEUE_CAN_IDx = &QUEUE_CAN_ID1;  break;
		#endif
		#ifdef ID_2
		    case 2:  QUEUE_CAN_IDx = &QUEUE_CAN_ID2;  break;
		#endif
		#ifdef ID_3
		    case 3:  QUEUE_CAN_IDx = &QUEUE_CAN_ID3;  break;
		#endif
		#ifdef ID_4
		    case 4:  QUEUE_CAN_IDx = &QUEUE_CAN_ID4;  break;
		#endif
		#ifdef ID_5
		    case 5:  QUEUE_CAN_IDx = &QUEUE_CAN_ID5;  break;
		#endif
		#ifdef ID_6
		    case 6:  QUEUE_CAN_IDx = &QUEUE_CAN_ID6;  break;
		#endif
		#ifdef ID_7
		    case 7:  QUEUE_CAN_IDx = &QUEUE_CAN_ID7;  break;
		#endif
		#ifdef ID_8
		    case 8:  QUEUE_CAN_IDx = &QUEUE_CAN_ID8;  break;
		#endif
		#ifdef ID_9
		    case 9:  QUEUE_CAN_IDx = &QUEUE_CAN_ID9;  break;
		#endif
		#ifdef ID_10
		    case 10: QUEUE_CAN_IDx = &QUEUE_CAN_ID10; break;
		#endif
		#ifdef ID_11
		    case 11: QUEUE_CAN_IDx = &QUEUE_CAN_ID11; break;
		#endif
		#ifdef ID_12
		    case 12: QUEUE_CAN_IDx = &QUEUE_CAN_ID12; break;
		#endif
		#ifdef ID_13
		    case 13: QUEUE_CAN_IDx = &QUEUE_CAN_ID13; break;
		#endif
		#ifdef ID_14
		    case 14: QUEUE_CAN_IDx = &QUEUE_CAN_ID14; break;
		#endif
		#ifdef ID_15
		    case 15: QUEUE_CAN_IDx = &QUEUE_CAN_ID15; break;
		#endif
	}
	tmp_rear = QUEUE_CAN_IDx->Rear + 1;
	if(tmp_rear >= CAN_BUF_NUM)
	{
		tmp_rear = 0;
	}
	if(tmp_rear == QUEUE_CAN_IDx->Front)
	{
		/* ���������� */
		return;
	}
		/* ��仺���� */
	QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].COBID   =  COBID_NMT_SERVICE;
	QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].DLC     =  2;
	QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].DATA[0] =  MNTCmd;
	QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].DATA[1] =  elmo->NodeID;

	/* ��Ч���ݼ�1 */
	QUEUE_CAN_IDx->Rear++;
	if(QUEUE_CAN_IDx->Rear >= CAN_BUF_NUM)
	{
		QUEUE_CAN_IDx->Rear = 0;
	}
}


/*
********************************************************************************
  *@  name      : RSDO
  *@  function  : ʹ������SDO����ָ���
  *@  input     : elmo      ȡelmo�ڵ�ID
				  Index     ����
                  SubIndex  ������
                  Data      ����
  *@  output    : None
********************************************************************************
*/
static void RSDO(Elmo *elmo, u16 Index, u8 SubIndex, u32 Data)
{
	u16 tmp_rear;

	switch(elmo->NodeID)
	{
		#ifdef ID_0 
		    case 0:  QUEUE_CAN_IDx = &QUEUE_CAN_ID0;  break;
		#endif
		#ifdef ID_1
		    case 1:  QUEUE_CAN_IDx = &QUEUE_CAN_ID1;  break;
		#endif
		#ifdef ID_2
		    case 2:  QUEUE_CAN_IDx = &QUEUE_CAN_ID2;  break;
		#endif
		#ifdef ID_3
		    case 3:  QUEUE_CAN_IDx = &QUEUE_CAN_ID3;  break;
		#endif
		#ifdef ID_4
		    case 4:  QUEUE_CAN_IDx = &QUEUE_CAN_ID4;  break;
		#endif
		#ifdef ID_5
		    case 5:  QUEUE_CAN_IDx = &QUEUE_CAN_ID5;  break;
		#endif
		#ifdef ID_6
		    case 6:  QUEUE_CAN_IDx = &QUEUE_CAN_ID6;  break;
		#endif
		#ifdef ID_7
		    case 7:  QUEUE_CAN_IDx = &QUEUE_CAN_ID7;  break;
		#endif
		#ifdef ID_8
		    case 8:  QUEUE_CAN_IDx = &QUEUE_CAN_ID8;  break;
		#endif
		#ifdef ID_9
		    case 9:  QUEUE_CAN_IDx = &QUEUE_CAN_ID9;  break;
		#endif
		#ifdef ID_10
		    case 10: QUEUE_CAN_IDx = &QUEUE_CAN_ID10; break;
		#endif
		#ifdef ID_11
		    case 11: QUEUE_CAN_IDx = &QUEUE_CAN_ID11; break;
		#endif
		#ifdef ID_12
		    case 12: QUEUE_CAN_IDx = &QUEUE_CAN_ID12; break;
		#endif
		#ifdef ID_13
		    case 13: QUEUE_CAN_IDx = &QUEUE_CAN_ID13; break;
		#endif
		#ifdef ID_14
		    case 14: QUEUE_CAN_IDx = &QUEUE_CAN_ID14; break;
		#endif
		#ifdef ID_15
		    case 15: QUEUE_CAN_IDx = &QUEUE_CAN_ID15; break;
		#endif
	}
	tmp_rear = QUEUE_CAN_IDx->Rear + 1;
	if(tmp_rear >= CAN_BUF_NUM)
	{
		tmp_rear = 0;
	}
	if(tmp_rear == QUEUE_CAN_IDx->Front)
	{
		/* ���������� */
		return;
	}
	/* ��仺���� */
	QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].COBID   = COBID_RSDO + elmo->NodeID;  // COBID
	QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].DLC     = 8;                          // DLC
	QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].DATA[0] =  0x22;                       // CS,������δȷ��
	QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].DATA[1] = (Index&0xFF);               // Index
	QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].DATA[2] = (Index&0xFF00)>>8;
	QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].DATA[3] = (SubIndex);                 // SubIndex
	QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].DATA[4] = (Data&0xFF);                // Data
	QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].DATA[5] = (Data&0xFF00)>>8;
	QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].DATA[6] = (Data&0xFF0000)>>16;
	QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].DATA[7] = (Data&0xFF000000)>>24;

	/* ��Ч���ݼ�1 */
	QUEUE_CAN_IDx->Rear++;
	if(QUEUE_CAN_IDx->Rear >= CAN_BUF_NUM)
	{
		QUEUE_CAN_IDx->Rear = 0;
	}
}


/*
********************************************************************************
  *@  name      : RPDO2_Cmd_data
  *@  function  : ʹ�ö����Ʊ����ELMO��������ָ��
                  CANopen RPDO2 -> 0x2012 ����������-���ù���
  *@  input     : elmo   ȡelmo�ڵ�ID
                  Cmd    ����,���ַ�����ʽ����
                  Index  ������±�           
                  Type   ��������
                  Data   ����
  *@  output    : None
********************************************************************************
*/
static void RPDO2_Cmd_data(Elmo *elmo, u8 *Cmd, u8 Index, u8 Type, u32 Data)
{
	u16 tmp_rear;
	switch(elmo->NodeID)
	{
		#ifdef ID_0 
		    case 0:  QUEUE_CAN_IDx = &QUEUE_CAN_ID0;  break;
		#endif
		#ifdef ID_1
		    case 1:  QUEUE_CAN_IDx = &QUEUE_CAN_ID1;  break;
		#endif
		#ifdef ID_2
		    case 2:  QUEUE_CAN_IDx = &QUEUE_CAN_ID2;  break;
		#endif
		#ifdef ID_3
		    case 3:  QUEUE_CAN_IDx = &QUEUE_CAN_ID3;  break;
		#endif
		#ifdef ID_4
		    case 4:  QUEUE_CAN_IDx = &QUEUE_CAN_ID4;  break;
		#endif
		#ifdef ID_5
		    case 5:  QUEUE_CAN_IDx = &QUEUE_CAN_ID5;  break;
		#endif
		#ifdef ID_6
		    case 6:  QUEUE_CAN_IDx = &QUEUE_CAN_ID6;  break;
		#endif
		#ifdef ID_7
		    case 7:  QUEUE_CAN_IDx = &QUEUE_CAN_ID7;  break;
		#endif
		#ifdef ID_8
		    case 8:  QUEUE_CAN_IDx = &QUEUE_CAN_ID8;  break;
		#endif
		#ifdef ID_9
		    case 9:  QUEUE_CAN_IDx = &QUEUE_CAN_ID9;  break;
		#endif
		#ifdef ID_10
		    case 10: QUEUE_CAN_IDx = &QUEUE_CAN_ID10; break;
		#endif
		#ifdef ID_11
		    case 11: QUEUE_CAN_IDx = &QUEUE_CAN_ID11; break;
		#endif
		#ifdef ID_12
		    case 12: QUEUE_CAN_IDx = &QUEUE_CAN_ID12; break;
		#endif
		#ifdef ID_13
		    case 13: QUEUE_CAN_IDx = &QUEUE_CAN_ID13; break;
		#endif
		#ifdef ID_14
		    case 14: QUEUE_CAN_IDx = &QUEUE_CAN_ID14; break;
		#endif
		#ifdef ID_15
		    case 15: QUEUE_CAN_IDx = &QUEUE_CAN_ID15; break;
		#endif
	}
	tmp_rear = QUEUE_CAN_IDx->Rear + 1;
	if(tmp_rear >= CAN_BUF_NUM)
	{
		tmp_rear = 0;
	}
	if(tmp_rear == QUEUE_CAN_IDx->Front)
	{
		/* ���������� */
		return;
	}
	/* ��仺���� */
	QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].COBID   = COBID_RPDO2 + elmo->NodeID;
	QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].DLC     = 8;
	QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].DATA[0] = (*Cmd++);
	QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].DATA[1] = (*Cmd);
	QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].DATA[2] = (Index);
	QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].DATA[3] = (Type<<7);
	QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].DATA[4] = (Data&0xFF);
	QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].DATA[5] = (Data&0xFF00)>>8;
	QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].DATA[6] = (Data&0xFF0000)>>16;
	QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].DATA[7] = (Data&0xFF000000)>>24;

	/* ��Ч���ݼ�1 */
	QUEUE_CAN_IDx->Rear++;
	if(QUEUE_CAN_IDx->Rear >= CAN_BUF_NUM)
	{
		QUEUE_CAN_IDx->Rear = 0;
	}
}


/*
********************************************************************************
  *@  name      : RPDO2_Cmd_string
  *@  function  : ʹ�ö����Ʊ����ELMO�����ַ���ָ��
                  CANopen RPDO2 -> 0x2012 ����������-ִ�й���
  *@  input     : elmo   ȡelmo�ڵ�ID
					cmd    ����,���ַ�����ʽ����
  *@  output    : None
********************************************************************************
*/
static void RPDO2_Cmd_string(Elmo *elmo, u8 *Cmd)
{
	u16 tmp_rear;

	switch(elmo->NodeID)
	{
		#ifdef ID_0 
		    case 0:  QUEUE_CAN_IDx = &QUEUE_CAN_ID0;  break;
		#endif
		#ifdef ID_1
		    case 1:  QUEUE_CAN_IDx = &QUEUE_CAN_ID1;  break;
		#endif
		#ifdef ID_2
		    case 2:  QUEUE_CAN_IDx = &QUEUE_CAN_ID2;  break;
		#endif
		#ifdef ID_3
		    case 3:  QUEUE_CAN_IDx = &QUEUE_CAN_ID3;  break;
		#endif
		#ifdef ID_4
		    case 4:  QUEUE_CAN_IDx = &QUEUE_CAN_ID4;  break;
		#endif
		#ifdef ID_5
		    case 5:  QUEUE_CAN_IDx = &QUEUE_CAN_ID5;  break;
		#endif
		#ifdef ID_6
		    case 6:  QUEUE_CAN_IDx = &QUEUE_CAN_ID6;  break;
		#endif
		#ifdef ID_7
		    case 7:  QUEUE_CAN_IDx = &QUEUE_CAN_ID7;  break;
		#endif
		#ifdef ID_8
		    case 8:  QUEUE_CAN_IDx = &QUEUE_CAN_ID8;  break;
		#endif
		#ifdef ID_9
		    case 9:  QUEUE_CAN_IDx = &QUEUE_CAN_ID9;  break;
		#endif
		#ifdef ID_10
		    case 10: QUEUE_CAN_IDx = &QUEUE_CAN_ID10; break;
		#endif
		#ifdef ID_11
		    case 11: QUEUE_CAN_IDx = &QUEUE_CAN_ID11; break;
		#endif
		#ifdef ID_12
		    case 12: QUEUE_CAN_IDx = &QUEUE_CAN_ID12; break;
		#endif
		#ifdef ID_13
		    case 13: QUEUE_CAN_IDx = &QUEUE_CAN_ID13; break;
		#endif
		#ifdef ID_14
		    case 14: QUEUE_CAN_IDx = &QUEUE_CAN_ID14; break;
		#endif
		#ifdef ID_15
		    case 15: QUEUE_CAN_IDx = &QUEUE_CAN_ID15; break;
		#endif
	}
	tmp_rear = QUEUE_CAN_IDx->Rear + 1;
	if(tmp_rear >= CAN_BUF_NUM)
	{
		tmp_rear = 0;
	}
	if(tmp_rear == QUEUE_CAN_IDx->Front)
	{
		/* ���������� */
		return;
	}

	/* ��仺���� */
	QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].COBID   = COBID_RPDO2 + elmo->NodeID;
	QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].DLC     = 4;
	QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].DATA[0] = (*Cmd++);
	QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].DATA[1] = (*Cmd);
	QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].DATA[2] = 0x00;
	QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].DATA[3] = 0x00;
	
	QUEUE_CAN_IDx->Rear++;
	if(QUEUE_CAN_IDx->Rear >= CAN_BUF_NUM)
	{
		QUEUE_CAN_IDx->Rear = 0;
	}
}



/*
********************************************************************************
  *@  name      : CAN_init
  *@  function  : Initialization for CAN
  *@  input     : None
  *@  output    : None
********************************************************************************
*/
void CAN_init(CAN_TypeDef* CANx)
{
	GPIO_InitTypeDef         GPIO_InitStructure;
	CAN_InitTypeDef          CAN_InitStructure;
	CAN_FilterInitTypeDef    CAN_FilterInitStructure;      
	NVIC_InitTypeDef         NVIC_InitStructure;
	if( CANx == CAN1 )
	{
		can = CAN1;				//Ϊ�ײ㷢�ͱ���ѡ��can��
		/* Enable GPIO clock */
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

		/* Enable CAN clock */
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

		/* Configure CAN RX and TX pins */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
		GPIO_Init(GPIOA, &GPIO_InitStructure);

		/* Connect CAN pins to AF9 */
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_CAN1);
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_CAN1);	
		/* CAN filter init */
		CAN_FilterInitStructure.CAN_FilterNumber = 0;                        
		CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;      
		CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;     
		CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000<<5;               
		CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;                     
		CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000<<5;           
		CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0006;               
		CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;               
		CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
		CAN_FilterInit(&CAN_FilterInitStructure);		
	}
	else if( CANx == CAN2 )
	{
		can = CAN2;				//Ϊ�ײ㷢�ͱ���ѡ��can��
		/* Enable GPIO clock */
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

		/* Enable CAN clock */
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1 | RCC_APB1Periph_CAN2, ENABLE);

		/* Configure CAN RX and TX pins */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
		GPIO_Init(GPIOB, &GPIO_InitStructure);

		/* Connect CAN pins to AF9 */
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_CAN2);
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_CAN2);			
	}

	/* CAN register init */
	CAN_DeInit(CANx);
	CAN_StructInit(&CAN_InitStructure);	 

	/* CAN cell init */
	CAN_InitStructure.CAN_TTCM = DISABLE;    
	CAN_InitStructure.CAN_ABOM = DISABLE;    
	CAN_InitStructure.CAN_AWUM = DISABLE;  
	CAN_InitStructure.CAN_NART = DISABLE;    
	CAN_InitStructure.CAN_RFLM = DISABLE;  
	CAN_InitStructure.CAN_TXFP = ENABLE;    
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;

	/* Baudrate = 1Mbps (CAN clocked at 42 MHz) */
	CAN_InitStructure.CAN_SJW  = CAN_SJW_1tq;
	CAN_InitStructure.CAN_BS1 = CAN_BS1_9tq;
	CAN_InitStructure.CAN_BS2 = CAN_BS2_4tq;
	CAN_InitStructure.CAN_Prescaler = 3;     //CAN������42/(1+9+4)/3=1Mbps
	CAN_Init(CANx, &CAN_InitStructure);

	/* CAN filter init */
	CAN_FilterInitStructure.CAN_FilterNumber = 0;                        
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;      
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;     
	CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000<<5;               
	CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;                     
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000<<5;           
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0006;               
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;               
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure);

	if( CANx == CAN1 )//�����Լ�
	{
		/* CAN FIFO0 message pending interrupt enable */ 
		CAN_ITConfig(CAN1,CAN_IT_FMP0, ENABLE);

		/* Enable the CAN1 global Interrupt */
		NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
	}
	else if( CANx == CAN2 )
	{
		/* CAN FIFO0 message pending interrupt enable */ 
		CAN_ITConfig(CAN2,CAN_IT_FMP0, ENABLE);

		/* Enable the CAN2 global Interrupt */
		NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 8;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);	
	}
}


/*
********************************************************************************
  *@  name      : TIM7_init
  *@  function  : TIM7��ʼ����ʹCAN����ÿ100us����һ��
  *@  input     : None
  *@  output    : None
********************************************************************************
*/
static void TIM7_init(uint8_t PPr, uint8_t SPr)
{
	TIM_TimeBaseInitTypeDef   TIM_TimeBaseStructure;
	NVIC_InitTypeDef          NVIC_InitStructure;

	/* TIM7 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7 , ENABLE);

	/* Enable the TIM7 global Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = PPr;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = SPr;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Time base configuration (TIM7 clocked at 84 MHz)*/
	TIM_TimeBaseStructure.TIM_Period = 840-1;
	TIM_TimeBaseStructure.TIM_Prescaler = 10-1;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);

	/* TIM7 IT enable */
	TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
	TIM_ClearFlag(TIM7, TIM_FLAG_Update); 

	/* TIM7 enable counter */
	TIM_Cmd(TIM7, ENABLE);
}


/*
********************************************************************************
  *@  name      : TIM7_IRQHandler
  *@  function  : TIM7�жϴ�������ÿ100us��������CAN����
  *@  input     : None
  *@  output    : None
********************************************************************************
*/
void TIM7_IRQHandler(void)
{
	OSIntEnter();
	
	TIM_ClearFlag(TIM7, TIM_FLAG_Update);
	Elmo_SendCmd();
	
	OSIntExit();
}


/*
********************************************************************************
  *@  name      : Self_test
  *@  function  : Elmo�ϵ��Լ�
  *@  input     : None
  *@  output    : None
                ���CAN�޷���elmo�����źţ���CAN_Error=1���ر�TIM�жϣ�����ֵ0x80000000
                ���ĳһ��elmoû�������ش�����Ϣ������ֵΪ��elmo�ı��
                �����������������0
********************************************************************************
*/
int Self_test(void)
{
	u32 cnt_delay = 0x3FFF;
	
	do
	{
	   __nop();
	}
    while( --cnt_delay );
    if( CAN_Error == 1) //�����ʼ��ʧ�ܣ��ر�TIM7���жϺ�TIM7
	{
		TIM_ITConfig(TIM7, TIM_IT_Update, DISABLE);
		TIM_Cmd(TIM7, DISABLE);		
		return 0x80000000;		
	}
	
	Elmo_software_delay_ms(500);
	
    while((Elmo_Init_Flag & ((1<<ELMO_NUM) - 1)) != ((1<<ELMO_NUM) - 1))
    {
	   //canopen�豸����֮��ᷢ�ͱ��ģ����ĺ���������һ�������Լ���豸�������ߡ�
	  	return Elmo_Init_Flag;
    }
//  CAN_ITConfig(CAN1,CAN_IT_FMP0, DISABLE);
//	CAN_ITConfig(CAN2,CAN_IT_FMP0, DISABLE);

	return 0;
}


/*
********************************************************************************
  *@  name      : Variate_init
  *@  function  : ������ʼ��
  *@  input     : None
  *@  output    : None
********************************************************************************
*/

static void Variate_init_QUEUE_CAN_IDx(CANQUEUE *QUEUE_CAN_IDx)
{
	memset (&QUEUE_CAN_IDx, 0, sizeof(QUEUE_CAN_IDx));
	memset (elmo, 0, sizeof(Elmo)*(ELMO_NUM + 1));
	memset (&elmogroup, 0, sizeof(Elmo));
	QUEUE_CAN_IDx->Front = 0;
	QUEUE_CAN_IDx->Rear  = 0;

}

static void Variate_init(void)
{
	#ifdef ID_0 
	    Variate_init_QUEUE_CAN_IDx(&QUEUE_CAN_ID0);
	#endif
	#ifdef ID_1
	    Variate_init_QUEUE_CAN_IDx(&QUEUE_CAN_ID1);
	#endif
	#ifdef ID_2
	    Variate_init_QUEUE_CAN_IDx(&QUEUE_CAN_ID2);
	#endif
	#ifdef ID_3
	    Variate_init_QUEUE_CAN_IDx(&QUEUE_CAN_ID3);
	#endif
	#ifdef ID_4
	    Variate_init_QUEUE_CAN_IDx(&QUEUE_CAN_ID4);
	#endif
	#ifdef ID_5
	    Variate_init_QUEUE_CAN_IDx(&QUEUE_CAN_ID5);
	#endif
	#ifdef ID_6
	    Variate_init_QUEUE_CAN_IDx(&QUEUE_CAN_ID6);
	#endif
	#ifdef ID_7
	    Variate_init_QUEUE_CAN_IDx(&QUEUE_CAN_ID7);
	#endif
	#ifdef ID_8
	    Variate_init_QUEUE_CAN_IDx(&QUEUE_CAN_ID8);
	#endif
	#ifdef ID_9
	    Variate_init_QUEUE_CAN_IDx(&QUEUE_CAN_ID9);
	#endif
	#ifdef ID_10
	    Variate_init_QUEUE_CAN_IDx(&QUEUE_CAN_ID10);
	#endif
	#ifdef ID_11
	    Variate_init_QUEUE_CAN_IDx(&QUEUE_CAN_ID11);
	#endif
	#ifdef ID_12
	    Variate_init_QUEUE_CAN_IDx(&QUEUE_CAN_ID12);
	#endif
	#ifdef ID_13
	    Variate_init_QUEUE_CAN_IDx(&QUEUE_CAN_ID13);
	#endif
	#ifdef ID_14
	    Variate_init_QUEUE_CAN_IDx(&QUEUE_CAN_ID14);
	#endif
	#ifdef ID_15
	    Variate_init_QUEUE_CAN_IDx(&QUEUE_CAN_ID15);
	#endif
}


/*
********************************************************************************
  *@  name      : Elmo_SendCmd_QUEUE_CAN_IDx
  *@  function  : ��������п�����CAN���ģ�Ҳ�п������ӳٱ���
  *@  input     : CANQUEUE *QUEUE_CAN_IDx Ҫ���͵Ļ�����
  *@  output    : None
********************************************************************************
*/



//extern u8 TIM7_OSTimeSave_Enable;
//u16 TIM7_OSTime_Count=0;
//u16 TIM7_OSTime_Count_ten=0;

void Elmo_SendCmd_QUEUE_CAN_IDx(CANQUEUE *QUEUE_CAN_IDx)   //����ָ��ָ��Ļ�����������
{
 	/* �жϻ������Ƿ������� */
	if(QUEUE_CAN_IDx->Rear != QUEUE_CAN_IDx->Front)
	{		
		/* ������,�ж��Ƿ�����ʱ���� */
		if(QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Front].COBID == CAN_ID_DELAY)
		{
			/* ����ʱָ��,�ж��Ƿ���ʱ��� */
			if(QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Front].DATA[0] > 1) //����һ��BUG������Ч���úܶ�
			{
				/* ��ʱδ��,��ʱʱ���1 */
				QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Front].DATA[0]--;
			}
			else
			{
				/* ��ʱ���,���׼�1 */      
				QUEUE_CAN_IDx->Front++;
				if( QUEUE_CAN_IDx->Front >= CAN_BUF_NUM)
				{
					QUEUE_CAN_IDx->Front = 0;
				}
			}
		}
		else
		{
			/* ������ʱָ��,����CAN���� */
			Elmo_CANSend( &QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Front] );

			/*���׼�1*/   
			QUEUE_CAN_IDx->Front++;
			if( QUEUE_CAN_IDx->Front >= CAN_BUF_NUM)
			{
				QUEUE_CAN_IDx->Front = 0;
			}
		}
	}
	else
	{
		/* ����Ϊ�� */ 
		return;   
	}

}


/*
********************************************************************************
  *@  name      : Elmo_SendCmd
  *@  function  : ��������п�����CAN���ģ�Ҳ�п������ӳٱ���
  *@  input     : None
  *@  output    : None
********************************************************************************
*/

static void Elmo_SendCmd(void)
{
	#ifdef ID_0 
	    Elmo_SendCmd_QUEUE_CAN_IDx(&QUEUE_CAN_ID0);
	#endif
    #ifdef ID_1
	    Elmo_SendCmd_QUEUE_CAN_IDx(&QUEUE_CAN_ID1);
	#endif
	#ifdef ID_2
	    Elmo_SendCmd_QUEUE_CAN_IDx(&QUEUE_CAN_ID2);
	#endif
	#ifdef ID_3
	    Elmo_SendCmd_QUEUE_CAN_IDx(&QUEUE_CAN_ID3);
	#endif
	#ifdef ID_4
	    Elmo_SendCmd_QUEUE_CAN_IDx(&QUEUE_CAN_ID4);	
	#endif
	#ifdef ID_5
	    Elmo_SendCmd_QUEUE_CAN_IDx(&QUEUE_CAN_ID5);
	#endif
	#ifdef ID_6
	    Elmo_SendCmd_QUEUE_CAN_IDx(&QUEUE_CAN_ID6);
	#endif
	#ifdef ID_7
	    Elmo_SendCmd_QUEUE_CAN_IDx(&QUEUE_CAN_ID7);
	#endif
	#ifdef ID_8
	    Elmo_SendCmd_QUEUE_CAN_IDx(&QUEUE_CAN_ID8);
	#endif
	#ifdef ID_9
	    Elmo_SendCmd_QUEUE_CAN_IDx(&QUEUE_CAN_ID9);
	#endif
	#ifdef ID_10
	    Elmo_SendCmd_QUEUE_CAN_IDx(&QUEUE_CAN_ID10);
	#endif
	#ifdef ID_11
	    Elmo_SendCmd_QUEUE_CAN_IDx(&QUEUE_CAN_ID11);
	#endif
	#ifdef ID_12
	    Elmo_SendCmd_QUEUE_CAN_IDx(&QUEUE_CAN_ID12);
	#endif
	#ifdef ID_13
	    Elmo_SendCmd_QUEUE_CAN_IDx(&QUEUE_CAN_ID13);
	#endif
	#ifdef ID_14
	    Elmo_SendCmd_QUEUE_CAN_IDx(&QUEUE_CAN_ID14);
	#endif
	#ifdef ID_15
	    Elmo_SendCmd_QUEUE_CAN_IDx(&QUEUE_CAN_ID15);
	#endif
	

}


/*
********************************************************************************
  *@  name      : Elmo_CANSend
  *@  function  : �ײ㷢��CAN����
  *@  input     : None
  *@  output    : None
********************************************************************************
*/
static void Elmo_CANSend(CANDATA *pCANDATA)
{
	u8 i, TransmitMailbox;
	u32 cnt_delay;
	CanTxMsg elmoCAN;

	elmoCAN.IDE    =  CAN_ID_STD;                          // ��׼֡
	elmoCAN.RTR    =  CAN_RTR_DATA;                        // ����֡
	elmoCAN.StdId  =  pCANDATA->COBID;                     // COBID
	elmoCAN.DLC    =  pCANDATA->DLC;                       // DLC
	for(i=0;i<elmoCAN.DLC;i++)                             // Data
	{
		elmoCAN.Data[i] = pCANDATA->DATA[i];		
	}
	TransmitMailbox = CAN_Transmit(can, &elmoCAN);                          // ���÷��ͱ��ĺ���   
	
	cnt_delay = 0x2FFF;
	do
	{
	   __nop();
	}
    while((CAN_TransmitStatus(can,TransmitMailbox) != CANTXOK) && (--cnt_delay));
	if (cnt_delay <= 0x01 )
		CAN_Error = 1;
    else
		CAN_Error = 0;
	
}


/*
********************************************************************************
  *@  name      : Elmo_Delay100us_IDx
  *@  function  : �ӳٱ��������ӳ���һ�����ĵķ��ͣ���߷��ͱ��ĵ��ȶ���
                  N���Ϊ255��N��100us��
  *@  input     : None
  *@  output    : None
********************************************************************************
*/
static void Elmo_Delay100us_IDx( Elmo *elmo , u8 N100us)
{
	uint16_t tmp_rear;
	switch(elmo->NodeID)
	{
		#ifdef ID_0 
		    case 0:  QUEUE_CAN_IDx = &QUEUE_CAN_ID0;  break;
		#endif
		#ifdef ID_1
		    case 1:  QUEUE_CAN_IDx = &QUEUE_CAN_ID1;  break;
		#endif
		#ifdef ID_2
		    case 2:  QUEUE_CAN_IDx = &QUEUE_CAN_ID2;  break;
		#endif
		#ifdef ID_3
		    case 3:  QUEUE_CAN_IDx = &QUEUE_CAN_ID3;  break;
		#endif
		#ifdef ID_4
		    case 4:  QUEUE_CAN_IDx = &QUEUE_CAN_ID4;  break;
		#endif
		#ifdef ID_5
		    case 5:  QUEUE_CAN_IDx = &QUEUE_CAN_ID5;  break;
		#endif
		#ifdef ID_6
		    case 6:  QUEUE_CAN_IDx = &QUEUE_CAN_ID6;  break;
		#endif
		#ifdef ID_7
		    case 7:  QUEUE_CAN_IDx = &QUEUE_CAN_ID7;  break;
		#endif
		#ifdef ID_8
		    case 8:  QUEUE_CAN_IDx = &QUEUE_CAN_ID8;  break;
		#endif
		#ifdef ID_9
		    case 9:  QUEUE_CAN_IDx = &QUEUE_CAN_ID9;  break;
		#endif
		#ifdef ID_10
		    case 10: QUEUE_CAN_IDx = &QUEUE_CAN_ID10; break;
		#endif
		#ifdef ID_11
		    case 11: QUEUE_CAN_IDx = &QUEUE_CAN_ID11; break;
		#endif
		#ifdef ID_12
		    case 12: QUEUE_CAN_IDx = &QUEUE_CAN_ID12; break;
		#endif
		#ifdef ID_13
		    case 13: QUEUE_CAN_IDx = &QUEUE_CAN_ID13; break;
		#endif
		#ifdef ID_14
		    case 14: QUEUE_CAN_IDx = &QUEUE_CAN_ID14; break;
		#endif
		#ifdef ID_15
		    case 15: QUEUE_CAN_IDx = &QUEUE_CAN_ID15; break;
		#endif
	}
	
	/* �жϻ������Ƿ����� */
	tmp_rear = QUEUE_CAN_IDx->Rear + 1;
	if(tmp_rear >= CAN_BUF_NUM)
	{
		tmp_rear = 0;
	}
	if(tmp_rear == QUEUE_CAN_IDx->Rear)
	{
		/* ���������� */
		return ;
	}

	/* ��仺���� */
	QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].COBID   = CAN_ID_DELAY;        // ��ʱ�õ�COBID 
	QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].DLC     = 1;                   // DLCΪ1r 
	QUEUE_CAN_IDx->CANBUF[QUEUE_CAN_IDx->Rear].DATA[0] = N100us;              // ��ʱ,��¼ʱ�� 

	/* ��Ч���ݼ�1 */
	QUEUE_CAN_IDx->Rear++;
	if(QUEUE_CAN_IDx->Rear >= CAN_BUF_NUM)
	{
		QUEUE_CAN_IDx->Rear = 0;
	}
}


/*
********************************************************************************
  *@  name      : Elmo_software_delay_ms
  *@  function  : Elmo�����ʱ
  *@  input     : None
  *@  output    : None
********************************************************************************
*/
void Elmo_software_delay_ms(unsigned int t)
{
	int i;
	for( i=0;i<t;i++)
	{
		int a = 41580; //at 168MHz 41580 is ok
 		while(a--);
	}
}


/*
********************************************************************************
  *@  name      : f2h
  *@  function  : ��������ת��Ϊ8�ֽ�ʮ��������(IEEE754)
  *@  input     : x   ������ 
  *@  output    : None
********************************************************************************
*/
static u32 f2h(float x)
{
	u32 *p = (u32 *)&x;
	return ((u32)*p);
}
/*
********************************************************************************
  *@  name      : Elmo_Read_POS
  *@  function  : ��ȡ��Ӧ��elmo�ı�����������
  *@  input     : elmoID   �޷���8λ���� 
  *@  output    : None
                  ����շ����ٶ�ָ��Ͷ�ȡ����������һ�εĶ�����׼��֮��ͺ���
********************************************************************************
*/
void Elmo_Read_POS(u8 elmoID)
{
	
	//����TPDO2
    RSDO(&elmo[elmoID], 0x1A01, 0x00,1);
	Elmo_Delay100us_IDx(&elmo[elmoID],50);
    //��ȡelmo����������ֵ
    RPDO2_Cmd_string(&elmo[elmoID], (u8 *)"PX");
    Elmo_Delay100us_IDx(&elmo[elmoID],20);
    //�ر�TPDO2
    RSDO(&elmo[elmoID], 0x1A01, 0x00,0);
	Elmo_Delay100us_IDx(&elmo[elmoID],50);
}

/**
  * @brief  Elmo_Read_POS,��ȡ��Ӧ��elmo�ı�����������
  * @param  elmoID   �޷���8λ����
  *         pEncData int32_t ��ָ�룬ָ�򷵻�����
  * @retval 0   ���ɹ�
  *         1   ��ʧ��
  * @time   RSDORead
  *         -   1ms�����������1000�Σ�����ȡ(���������������յ�����)ʱ��δ���� 80000/84us = 952.38us
  *         -   �˴��ȴ�1.5ms
  * @note   �ж��ж�ȡ
  *         RxMsg.StdId == COBID_TSDO + ID;
  *         if ((RxMsg.Data[2]<<8 | RxMsg.Data[1]) == 0x6064)
  *             memcpy(&Encoder_Data, &RxMsg.Data[4],sizeof(int32_t));
  *         Ŀǰ��֧�� ID��MOTOR_ID_HIT_MAIN��MOTOR_ID_HIT_SERVE ������
  */
//uint32_t readElmo_Pos(u8 elmoID, int32_t *pEncData)
//{
//    uint32_t err = 1;
//    /* ����ʼ�տ���CAN1�����жϣ��ʲ����ٿ���CAN1�����ж� */
//    //CAN1->IER |= CAN_IT_FMP0;

//    /* ��ȡelmo����������ֵ */
//    Elmo_Read_POS(elmoID);
//    
////    if(elmoID == MOTOR_ID_ARM_X)
////    {
////        if(xSemaphoreTake(PosArmXSem, 3*portTICK_MS) == pdTRUE)   //1.5ms  CAN������
////        {
////            *pEncData = EncoderData_1;
////            err = 0;
////        }
////        else
////            err = 1;
////    }
////    else if(elmoID == MOTOR_ID_CLIP)
////    {
////        if(xSemaphoreTake(PosClipSem, 3*portTICK_MS) == pdTRUE)   //1.5ms
////        {
////            *pEncData = EncoderData_3;
////            err = 0;
////        }
////        else
////            err = 1;
////    }
//    
//    /* �ر�CAN1�����ж� */
//    //CAN1->IER &= ~CAN_IT_FMP0;
//    return err;
//}
/*
********************************************************************************
  *@  name      : Elmo_Set_POS
  *@  function  : ��ȡ��Ӧ��elmo�ı�����������
  *@  input     : elmoID   �޷���8λ���� 
  *@  output    : None
********************************************************************************
*/
void Elmo_Set_POS(u8 elmoID,s32 POS)
{
	  //�رյ������������λ��ֵ���Ⱦ�����
	  RPDO2_Cmd_data(&elmo[elmoID], (u8 *)"MO", 0, TYPE_INTEGER, MO_OFF);
		Elmo_Delay100us_IDx(&elmo[elmoID],90);
	
	  //�Ա�������ֵ��������
	  RPDO2_Cmd_data(&elmo[elmoID], (u8 *)"PX", 0, TYPE_INTEGER, POS);
		Elmo_Delay100us_IDx(&elmo[elmoID],90);
	
	  //�������
	  RPDO2_Cmd_data(&elmo[elmoID], (u8 *)"MO", 0, TYPE_INTEGER, MO_ON);
		Elmo_Delay100us_IDx(&elmo[elmoID],90);
	  //���µ�ǰ���״̬
	  elmo[elmoID].CurOPMode = UM_UNC;
	

}

/*
********************************************************************************
  *@  name      : Elmo_Read_ACT_CUR
  *@  function  : ��ȡ��Ӧ��elmo����Ч����
  *@  input     : elmoID   �޷���8λ���� 
  *@  output    : None
********************************************************************************
*/
void Elmo_Read_ACT_CUR(u8 elmoID)
{
	//����TPDO2
    RSDO(&elmo[0], 0x1A01, 0x00,1);
	Elmo_Delay100us_IDx(&elmo[elmoID],50);
    //��ȡelmo����������ֵ
    RPDO2_Cmd_string(&elmo[elmoID], (u8 *)"IQ");
	Elmo_Delay100us_IDx(&elmo[elmoID],20);
	//�ر�TPDO2
    RSDO(&elmo[0], 0x1A01, 0x00,0);
	Elmo_Delay100us_IDx(&elmo[elmoID],50);

}


/*
********************************************************************************
  *@  name      : CAN1_Send_Msg
  *@  function  : CAN1���ͺ���
  *@  input     : 	msg		DATA to send
					len		length of DATA
					id		ID
  *@  output    : 1|0		whether success
********************************************************************************
*/
//	CanTxMsg CAN1TxMessage;

//u8 CAN1_Send_Msg(u8* msg,u8 len,int id)
//{	
//	u16 i=0;
//	u8 mbox = 0;
//	
//	CAN1TxMessage.StdId=id;	 	// ��׼��ʶ��Ϊ0
//	CAN1TxMessage.IDE=0;		// ʹ����չ��ʶ��
//	CAN1TxMessage.RTR=0;		// ��Ϣ����Ϊ����֡��һ֡8λ
//	CAN1TxMessage.DLC=len;	 	// ����len֡��Ϣ
//	
//	for(i=0;i<len;i++)CAN1TxMessage.Data[i]=msg[i];
//	mbox= CAN_Transmit(CAN1, &CAN1TxMessage);   
//	i = 0;
//	while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//�ȴ����ͽ���
//	if(i>=0XFFF)return 1;
//	return 0;		
//}


/*
********************************************************************************
  *@  name      : CAN2_Send_Msg
  *@  function  : CAN2���ͺ���
  *@  input     : 	msg		DATA to send
					len		length of DATA
					id		ID
  *@  output    : 1|0		whether success
********************************************************************************
*/
	CanTxMsg CAN2TxMessage;

u8 CAN2_Send_Msg(u8* msg,u8 len,int id)
{	
	u16 i=0;
	u8 mbox = 0;
	CAN2TxMessage.StdId=id;	 	// ��׼��ʶ��Ϊ0
	CAN2TxMessage.IDE=0;		// ʹ����չ��ʶ��
	CAN2TxMessage.RTR=0;		// ��Ϣ����Ϊ����֡��һ֡8λ
	CAN2TxMessage.DLC=len;	 	// ����len֡��Ϣ
	
	for(i=0;i<len;i++)CAN2TxMessage.Data[i]=msg[i];
	mbox= CAN_Transmit(CAN2, &CAN2TxMessage);   
	i = 0;
	while((CAN_TransmitStatus(CAN2, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//�ȴ����ͽ���
	if(i>=0XFFF)return 1;
	return 0;		
}

