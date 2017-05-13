/*--------------------------------version information top-------------------------------------------
Started at 2016
Created by Karlxin(410824290@qq.com)
Github:https://github.com/Karlxin/OK001.git
OpenKarlCopter
























--------------------------------version information bottom-------------------------------------------*/
#include "pwm.h"

/*******************************************************************************
	* @Name				TIM3_PWM_Init
	* @Description		TIM3 outputing four PWM
	* @Input			arr,psc
	* @Use				None
	* @Output			None
	* @Return			None
*******************************************************************************/
void TIM3_PWM_Init(u16 arr, u16 psc)
{
    GPIO_InitTypeDef GPIO_InitStructure;//define a structure of GPIO_InitTypeDef to initialize timer 3
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;//define a structure of TIM_TimeBaseInitTypeDef to initialize timer 3
    TIM_OCInitTypeDef  TIM_OCInitStructure;//define a structure of TIM_OCInitTypeDef to initialize timer 3

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);//Enable timer 3 clock of advanced peripheral bus 1
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC  | RCC_APB2Periph_AFIO, ENABLE);  //Enable GPIO and AFIO clock

    GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE); //Timer3 reflected to GPIOC-> 6,7,8,9

	//setting pins as Alternative Function Input Output,outputting TIM3 CH1 CH2 CH3 CH4 PWM
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9; //Define the GPIO pins desired
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //Alternative Function Push Pull output
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//define frequency as 50 MHz
    GPIO_Init(GPIOC, &GPIO_InitStructure);//initialize GPIO with setting of GPIO_InitStructure

    TIM_TimeBaseStructure.TIM_Period = arr; //setting auto reload register value
    TIM_TimeBaseStructure.TIM_Prescaler = psc; //setting prescaler avlue
    TIM_TimeBaseStructure.TIM_ClockDivision = 0; //setting clock division as DIV1
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //setting TIM counter mode as up
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //initialize TIM with setting of TIM_TimeBaseInitStruct

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //setting Output Compare TIM as PWM1 mode
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //Enable Output State
    TIM_OCInitStructure.TIM_Pulse = 0; //Specifies the pulse value to be loaded into the Capture Compare Register as 0x0000
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //Specifies the output polarity as High
    TIM_OC1Init(TIM3, &TIM_OCInitStructure);  //initialize TIM with setting of TIM_OCInitStruct
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);  //Enable TIM3 preload register in CCR1


    TIM_OC2Init(TIM3, &TIM_OCInitStructure);  
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);  //Enable TIM3 preload register in CCR2

    TIM_OC3Init(TIM3, &TIM_OCInitStructure);  
    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);  

    TIM_OC4Init(TIM3, &TIM_OCInitStructure);  
    TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);  

    TIM_ARRPreloadConfig(TIM3, ENABLE); //Enable TIM3 preload register in ARR

    TIM_Cmd(TIM3, ENABLE);  //Enable TIM3 peripheral
}



