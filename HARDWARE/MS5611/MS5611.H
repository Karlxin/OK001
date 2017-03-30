#ifndef __MS5611_H_
#define __MS5611_H_

#include "stm32f10x.h"
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "myiic.h"


/* 函数申明 -----------------------------------------------*/

#define MS561101BA_ADC_RD          0x00
#define	MS561101BA_PROM_RD 	       0xA0
#define MS561101BA_PROM_CRC        0xAE

#define MS561101BA_SlaveAddress    0xEE  //MS5611的地址
#define MS561101BA_RST             0x1E  //cmd 复位

#define	MS561101BA_D2_OSR_4096   	 0x58	// 9.04 mSec conversion time ( 110.62 Hz)
#define	MS561101BA_D1_OSR_4096   	 0x48

#define MS5611_OSR256					 		 0x40
#define MS5611_OSR512					 		 0x42
#define MS5611_OSR1024					   0x44
#define MS5611_OSR2048					   0x46
#define MS5611_OSR4096					   0x48
#define FILTER_num 20

u8  MS5611_init(void);
float Get_High(void);
void Filter_Hight(unsigned int set_hight);
void Hight_PwmOut(void);
void MS561101BA_getPressure(void);
void MS561101BA_getTemperature(void);
void MS561101BA_RESET(void);



unsigned long MS561101BA_getConversion(uint8_t command);
void MS561101BA_GetTemperature(void);

extern int32_t dT,TEMP;
extern uint32_t Pressure;
extern uint16_t  Cal_C[7];

#endif

