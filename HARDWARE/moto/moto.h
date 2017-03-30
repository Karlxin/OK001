#ifndef __MOTO_H
#define __MOTO_H
#include "sys.h"
#include "stm32f10x.h"

#define Moto_PwmMax 2000
void Moto_PwmRflash(int16_t MOTO1_PWM,int16_t MOTO2_PWM,int16_t MOTO3_PWM,int16_t MOTO4_PWM);
void cyberNation(void);
void Moto_RPY(int16_t desthrottle);
#endif


