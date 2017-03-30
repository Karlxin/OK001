#include "tim.h"
#include "usart.h"
#include "imu.h"
#include "sys.h"
#include "stm32f10x_tim.h"

u8 m1,m5;
/**************************ʵ�ֺ���********************************************
*����ԭ��:		
*��������:1ms�ж�һ��,������Ϊ1000		
*******************************************************************************/
void Tim3_Init(u16 period_num)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	//�������ã�ʱ���ͱȽ�������ã���������ֻ�趨ʱ�����Բ���OC�Ƚ����
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	
	TIM_DeInit(TIM3);

	TIM_TimeBaseStructure.TIM_Period=period_num;//װ��ֵ
	//prescaler is 1200,that is 72000000/72/500=2000Hz;
	TIM_TimeBaseStructure.TIM_Prescaler=72-1;//��Ƶϵ��
	//set clock division 
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; //or TIM_CKD_DIV2 or TIM_CKD_DIV4
	//count up
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);
	//clear the TIM2 overflow interrupt flag
	TIM_ClearFlag(TIM3,TIM_FLAG_Update);
	//TIM2 overflow interrupt enable
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
	//enable TIM2
	TIM_Cmd(TIM3,ENABLE);
}

/*
void TIM3_IRQHandler(void)		//0.5ms�ж�һ��
{
	static u8 ms1 = 0,ms2 = 0,ms5 = 0,ms10 = 0,ms100=0;				//�жϴ���������
	if(TIM3->SR & TIM_IT_Update)		//if ( TIM_GetITStatus(TIM3 , TIM_IT_Update) != RESET ) 
	{     
		TIM3->SR = ~TIM_FLAG_Update;//TIM_ClearITPendingBit(TIM3 , TIM_FLAG_Update);   //����жϱ�־
		
		
		//ÿ���ж϶�ִ��,0.5ms
		ms1++;
		ms2++;
		ms5++;
		ms10++;
		if(ms1==2)				//ÿ�����ж�ִ��һ��,1ms
		{
			ms1=0;

		 //��������
		}
		if(ms2==4)				//ÿ�Ĵ��ж�ִ��һ��,2ms
		{
			ms2=0;
		   //�õ���̬
			
		}
		if(ms5==10)
		{
			ms5=0;					//ÿʮ���ж�ִ��һ��,5ms
			 m5=1;             //PC��λ������ʱ������̬���ֻ�������������̬
		
		}
		if(ms10==20)
		{
			ms10=0;					//û��ʮ���ж�ִ��һ��,10ms
			ms100 ++;
			if(ms100==5)
			{
							
				ms100 = 0;				
				
				//����ң���Լ����ת�ٵ�ѹ����
			}
		}
	}
}

*/

