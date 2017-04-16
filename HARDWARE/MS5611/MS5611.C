#include "MS5611.h"
#include "math.h"

//status for MS5611
#define SCTemperature    0x01	  //starting conversion of temperature
#define CTemperatureing  0x02 //converting of temperature
#define SCPressure  		 0x03	    //starting conversion of pressure
#define SCPressureing    0x04	  //converting of temperature

extern int64_t OFF_;
extern float MS5611_Pressure;

extern uint16_t  Cal_C[7];	        

extern uint32_t D1_Pres, D2_Temp;	


extern int32_t dT, TEMP;

extern int64_t OFF, SENS;

extern int32_t P;

extern int64_t T2, TEMP2;	
extern int64_t OFF2, SENS2;



extern float scaling;
extern float temp_jisuan ;

extern float MS5611_Altitude;

/*******************************************************************************
  * @函数名称	MS561101BA_RESET
  * @函数说明   reset MS5611
  * @输入参数   无
  * @输出参数   无
  * @返回参数   无
*******************************************************************************/
void MS561101BA_RESET(void)
{
    IIC_Start();
    IIC_Send_Byte(0xEE);//CSB接地，主机地址：0XEE，否则 0X77
    IIC_Wait_Ack();
    IIC_Send_Byte(0x1E);//发送复位命令
    IIC_Wait_Ack();
    IIC_Stop();

}
/*******************************************************************************
  * @函数名称	MS5611_init
  * @函数说明   初始化5611
  * @输入参数  	无
  * @输出参数   无
  * @返回参数   无
*******************************************************************************/
u8 MS5611_init(void)
{
    u8  inth, intl;
    int i;
    for (i = 1; i <= 6; i++)
    {

        IIC_Start();
        IIC_Send_Byte(0xEE);
        IIC_Wait_Ack();

        IIC_Send_Byte(0xA0 + (i * 2));
        IIC_Wait_Ack();

        IIC_Stop();
        delay_us(5);
        IIC_Start();
        IIC_Send_Byte(0xEE + 0x01); //进入接收模式
        delay_us(1);
        IIC_Wait_Ack();
        inth = IIC_Read_Byte(1);  		//带ACK的读数据
        delay_us(1);
        intl = IIC_Read_Byte(0); 			//最后一个字节NACK
        IIC_Stop();

        delay_ms(5);

        Cal_C[i] = (((uint16_t)inth << 8) | intl);
    }
    return !Cal_C[0];
}


/**************************实现函数********************************************
*函数原型:unsigned long MS561101BA_getConversion(void)
*功　　能:    读取 MS561101B 的转换结果
*******************************************************************************/
unsigned long MS561101BA_getConversion(uint8_t command)
{

    unsigned long conversion = 0;
    u8 temp[3];

    IIC_Start();
    IIC_Send_Byte(0xEE); 		//写地址
    IIC_Wait_Ack();
    IIC_Send_Byte(command); //写转换命令
    IIC_Wait_Ack();
    IIC_Stop();

    delay_ms(9);
    delay_us(40);

    IIC_Start();
    IIC_Send_Byte(0xEE); 		//写地址
    IIC_Wait_Ack();
    IIC_Send_Byte(0);				// start read sequence
    IIC_Wait_Ack();
    IIC_Stop();

    IIC_Start();
    IIC_Send_Byte(0xEE + 0x01); //进入接收模式
    IIC_Wait_Ack();
    temp[0] = IIC_Read_Byte(1);  //带ACK的读数据  bit 23-16
    temp[1] = IIC_Read_Byte(1);  //带ACK的读数据  bit 8-15
    temp[2] = IIC_Read_Byte(0);  //带NACK的读数据 bit 0-7
    IIC_Stop();

    conversion = (unsigned long)temp[0] * 65536 + (unsigned long)temp[1] * 256 + (unsigned long)temp[2];
    return conversion;

}


/**************************实现函数********************************************
*函数原型:void MS561101BA_GetTemperature(void)
*功　　能:    读取 温度转换结果
*******************************************************************************/

void MS561101BA_GetTemperature(void)
{

    D2_Temp = MS561101BA_getConversion(0x58);
    delay_us(5);

    dT = D2_Temp - (((uint32_t)Cal_C[5]) << 8);
    TEMP = 2000 + (dT * (uint32_t)Cal_C[6] >> 23) ;
}

///***********************************************
//  * @brief  读取气压
//  * @param  None
//  * @retval None
//************************************************/


void MS561101BA_getPressure(void)
{
    D1_Pres = MS561101BA_getConversion(0x48);
    delay_us(5);

    OFF_ = ((uint32_t)Cal_C[2] << 16) + (((uint32_t)Cal_C[4] * dT) >> 7);
    SENS = ((uint32_t)Cal_C[1] << 15) + (((uint32_t)Cal_C[3] * dT) >> 8);
    P = ((D1_Pres * SENS >> 21) - OFF_) >> 15;

    if(TEMP < 2000)
    {
        T2 = (dT * dT) >> 31;
        OFF2 = 5 * (((TEMP - 2000) * (TEMP - 2000)) >> 1);
        SENS2 = 5 * (((TEMP - 2000) * (TEMP - 2000)) >> 2);

        if(TEMP < -1500)
        {
            OFF2 = OFF2 + 7 * (TEMP + 1500) * (TEMP + 1500);
            SENS2 = SENS2 + 11 * (((TEMP + 1500) * (TEMP + 1500)) >> 1);
        }
    }
    else
    {
        T2 = 0;
        OFF2 = 0;
        SENS2 = 0;
    }

    TEMP = TEMP - T2;
    OFF_ = OFF_ - OFF2;
    SENS = SENS - SENS2;

    MS5611_Pressure =  ((D1_Pres * SENS >> 21) - OFF_) >> 15;
}
//return in centimeter
float MS561101BA_get_altitude(float scaling)
{
    MS5611_Altitude = 15384.62f * temp_jisuan * (1.0f - expf(0.190259f * logf(scaling)));
    return MS5611_Altitude;
}

extern float Altitude_dt;
extern float Altitude_R;
extern float Altitude_Q;
extern float Altitude_K;//kalman gain
extern float Altitude_X_hat;//init predict
extern float Altitude_X_hat_minus;//previous
extern float Altitude_P;//error variance

//MS5611_Altitude in m
void Kalman_filter_alt(void)//Altitude Kalman filter
{
    //time update
    Altitude_X_hat_minus = Altitude_X_hat;
    Altitude_P = Altitude_P + Altitude_Q;

    //predict update
    Altitude_K = Altitude_P / (Altitude_P + Altitude_R);
    Altitude_X_hat = Altitude_X_hat_minus + Altitude_K * (MS5611_Altitude - Altitude_X_hat_minus);
    Altitude_P = (1 - Altitude_K) * Altitude_P;
}


extern float baro_climb_R;
extern float baro_climb_Q;
extern float baro_climb_K;//kalman gain
extern float baro_climb_X_hat;//init predict
extern float baro_climb_X_hat_minus;//previous predict
extern float baro_climb_P;//error variance

extern float Altitude_minus;
extern float baro_climb_rate;

void Kalman_filter_baro_climb(void)
{
    //time update
    baro_climb_X_hat_minus = baro_climb_X_hat;
    baro_climb_P = baro_climb_P + baro_climb_Q;

    //predict update
    baro_climb_K = baro_climb_P / (baro_climb_P + baro_climb_R);
    baro_climb_X_hat = baro_climb_X_hat_minus + baro_climb_K * (baro_climb_rate - baro_climb_X_hat_minus);
    baro_climb_P = (1 - baro_climb_K) * baro_climb_P;
}





extern float pressure_R;
extern float pressure_Q;
extern float pressure_K;
extern float pressure_X_hat;
extern float pressure_X_hat_minus;
extern float pressure_P;

void Kalman_filter_pressure(void)
{
    //time update
    pressure_X_hat_minus = pressure_X_hat;
    pressure_P = pressure_P + pressure_Q;

    //predict update
    pressure_K = pressure_P / (pressure_P + pressure_R);
    pressure_X_hat = pressure_X_hat_minus + pressure_K * (MS5611_Pressure - pressure_X_hat_minus);
    pressure_P = (1 - pressure_K) * pressure_P;
}

extern float Altitude_samples[7];
extern u8 Altitude_sample_index;
extern float Altitude_samples_time_stamps[7];

#define SAMPLES_RANGE 7
#define f(i) Altitude_samples[(((i+1)+3*SAMPLES_RANGE/2)%SAMPLES_RANGE)]
#define x(i) Altitude_samples_time_stamps[(((i+1)+3*SAMPLES_RANGE/2)%SAMPLES_RANGE)]

extern float baro_climb_rate;

//baro_climb_rate in cm/s2
void Derivative_Filter(void)
{
    baro_climb_rate = (10.0f * (f(1) - f(-1)) / (x(1) - x(-1)) + 16.0f * (f(2) - f(-2)) / (x(2) - x(-2)) + 6.0f * (f(3) - f(-3)) / (x(3) - x(-3))) * 0.03125f;
}


