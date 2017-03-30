#include "tim.h"
#include "usart.h"
#include "imu.h"
#include "sys.h"
#include "stm32f10x_tim.h"

u8 m1,m5;
/**************************实现函数********************************************
*函数原型:		
*功　　能:1ms中断一次,计数器为1000		
*******************************************************************************/
void Tim3_Init(u16 period_num)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	//基础设置，时基和比较输出设置，由于这里只需定时，所以不用OC比较输出
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	
	TIM_DeInit(TIM3);

	TIM_TimeBaseStructure.TIM_Period=period_num;//装载值
	//prescaler is 1200,that is 72000000/72/500=2000Hz;
	TIM_TimeBaseStructure.TIM_Prescaler=72-1;//分频系数
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
void TIM3_IRQHandler(void)		//0.5ms中断一次
{
	static u8 ms1 = 0,ms2 = 0,ms5 = 0,ms10 = 0,ms100=0;				//中断次数计数器
	if(TIM3->SR & TIM_IT_Update)		//if ( TIM_GetITStatus(TIM3 , TIM_IT_Update) != RESET ) 
	{     
		TIM3->SR = ~TIM_FLAG_Update;//TIM_ClearITPendingBit(TIM3 , TIM_FLAG_Update);   //清除中断标志
		
		
		//每次中断都执行,0.5ms
		ms1++;
		ms2++;
		ms5++;
		ms10++;
		if(ms1==2)				//每两次中断执行一次,1ms
		{
			ms1=0;

		 //更新数据
		}
		if(ms2==4)				//每四次中断执行一次,2ms
		{
			ms2=0;
		   //得到姿态
			
		}
		if(ms5==10)
		{
			ms5=0;					//每十次中断执行一次,5ms
			 m5=1;             //PC上位机解锁时发送姿态，手机解锁不发送姿态
		
		}
		if(ms10==20)
		{
			ms10=0;					//没二十次中断执行一次,10ms
			ms100 ++;
			if(ms100==5)
			{
							
				ms100 = 0;				
				
				//发送遥控以及电机转速电压数据
			}
		}
	}
}

*/

