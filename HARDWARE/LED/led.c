/*--------------------------------version information top-------------------------------------------
Started at 2016
Created by Karlxin(410824290@qq.com)
Github:https://github.com/Karlxin/OK001.git
OpenKarlCopter
























--------------------------------version information bottom-------------------------------------------*/

#include "led.h"


/*******************************************************************************
	* @Name				LED_Init
	* @Description		LED IO init
	* @Input    		None
	* @Use    			None
	* @Output   		None
	* @Return   		None
*******************************************************************************/
void LED_Init(void)
{

    GPIO_InitTypeDef  GPIO_InitStructure;//for init

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOD, ENABLE);//Enable PA,PD clock

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;				 //LED0-->PA.8
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //Push Pull Ouput
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO frequency 50MHz
    GPIO_Init(GPIOA, &GPIO_InitStructure);					 //initialize GPIOA.8 with setting of GPIO_InitStructure
    GPIO_SetBits(GPIOA, GPIO_Pin_8);						 //set PA.8

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;	    		 //LED1-->PD.2
    GPIO_Init(GPIOD, &GPIO_InitStructure);	  				 
    GPIO_SetBits(GPIOD, GPIO_Pin_2); 						 //set PD.2
}
