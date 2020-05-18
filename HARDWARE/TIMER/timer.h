#ifndef _TIMER_H
#define _TIMER_H

#include "sys.h"
#include "includes.h"
#include "exchange.h"

#define SYSCLK 168 			//MCU����Ƶ��216MHz
#define ONE_CYCLE_TIME 3	//һ��ѭ��������������
#define TOTAL_CYCLE_TIME 3	//���á���ʼ���������ܹ����õ�������
#define Delay_us(nus) wait(((nus) * (SYSCLK) - (TOTAL_CYCLE_TIME)) / (ONE_CYCLE_TIME))
#define Delay_ms(nms) Delay_us((nms)*1000)
#define Delay_s(ns) Delay_ms((ns)*1000)

void TIM2_Init(void);
void HBCheck_TIM4_Init(void);
void HBSelfCheck_TIM5_Init(void);
void EndHSCheck_TIM6_Init(void);
void wait(uint32_t n);

#endif

