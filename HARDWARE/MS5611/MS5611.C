#include "MS5611.h"
#include "math.h"

//气压计状态机
#define SCTemperature    0x01	  //开始温度转换
#define CTemperatureing  0x02 //正在转换温度
#define SCPressure  		 0x03	    //开始气压转换
#define SCPressureing    0x04	  //正在转换气压

extern int64_t OFF_;
extern float MS5611_Pressure;

/*
C1 压力灵敏度 SENS|T1
C2  压力补偿  OFF|T1
C3	温度压力灵敏度系数 TCS
C4	温度系数的压力补偿 TCO
C5	参考温度 T|REF
C6 	温度系数的温度 TEMPSENS
*/
extern uint16_t  Cal_C[7];	        //用于存放PROM中的6组数据1-6

extern uint32_t D1_Pres, D2_Temp;	// 数字压力值,数字温度值

/*
dT 实际和参考温度之间的差异
TEMP 实际温度
*/
extern int32_t dT, TEMP;
/*
OFF 实际温度补偿
SENS 实际温度灵敏度
*/
extern int64_t OFF, SENS;

extern int32_t P;//单位0.01mbar

extern int64_t T2, TEMP2;	//温度校验值
extern int64_t OFF2, SENS2;

extern uint32_t Pres_BUFFER[20];     //数据组
extern uint32_t Temp_BUFFER[10];     //数据组

extern float scaling;
extern float temp_jisuan ;

extern float MS5611_Altitude;

/*******************************************************************************
  * @函数名称	MS561101BA_RESET
  * @函数说明   复位MS5611
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
    delay_us(100);

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

void MS561101BA_get_altitude(void)
{
    MS5611_Altitude = 153.8462f * temp_jisuan * (1.0f - expf(0.190259f * logf(scaling)));
}

extern float Altitude_dt;
extern float Altitude_R;
extern float Altitude_Q;
extern float Altitude_K;//kalman gain
extern float Altitude_X_hat;//init predict
extern float Altitude_X_hat_minus;//previous
extern float Altitude_P;//error variance


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


extern float Climb_R;
extern float Climb_Q;
extern float Climb_K;//kalman gain
extern float Climb_X_hat;//init predict
extern float Climb_X_hat_minus;//previous predict
extern float Climb_P;//error variance

extern float Altitude_minus;

void Kalman_filter_climb(void)
{
    //time update
    Climb_X_hat_minus = Climb_X_hat;
    Climb_P = Climb_P + Climb_Q;

    //predict update
    Climb_K = Climb_P / (Climb_P + Climb_R);
    Climb_X_hat = Climb_X_hat_minus + Climb_K * ((MS5611_Altitude - Altitude_minus) / Altitude_dt - Climb_X_hat_minus);
    Climb_P = (1 - Climb_K) * Climb_P;
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






