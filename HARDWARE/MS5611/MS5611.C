/*--------------------------------version information top-------------------------------------------
Started at 2016
Created by Karlxin(410824290@qq.com)
Github:https://github.com/Karlxin/OK001.git
OpenKarlCopter
























--------------------------------version information bottom-------------------------------------------*/
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
	* @Name				MS561101BA_RESET
	* @Description		reset MS5611
	* @Input    		None
	* @Use				None
	* @Output   		None
	* @Return   		None
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
	* @Name				MS5611_init
	* @Description		initialize MS5611
	* @Input    		None
	* @Use				Cal_C[6:1]
	* @Output   		None
	* @Return   		!Cal_C[0]
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


/*******************************************************************************
	* @Name				MS561101BA_getConversion
	* @Description		get conversion from MS5611 barometer
	* @Input    		command
	* @Use				Cal_C[6:5]
	* @Output   		None
	* @Return   		conversion
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



/*******************************************************************************
	* @Name				MS561101BA_GetTemperature
	* @Description		read temperature from MS5611 barometer
	* @Input    		None
	* @Use				Cal_C[6:5]
	* @Output   		D2_Temp,dT,TEMP
	* @Return   		None
*******************************************************************************/
void MS561101BA_GetTemperature(void)
{

    D2_Temp = MS561101BA_getConversion(0x58);
    delay_us(5);

    dT = D2_Temp - (((uint32_t)Cal_C[5]) << 8);
    TEMP = 2000 + (dT * (uint32_t)Cal_C[6] >> 23) ;
}

/*******************************************************************************
	* @Name				MS561101BA_getPressure
	* @Description		read pressure from MS5611 barometer
	* @Input    		None
	* @Use				Cal_C[4:1],TEMP,dT
	* @Output   		D1_Pres,OFF_,SENS,P,T2,OFF2,SENS2,TEMP,MS5611_Pressure
	* @Return   		None
*******************************************************************************/
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

/*******************************************************************************
	* @Name				MS561101BA_get_altitude
	* @Description		calculate altitude
	* @Input    		scaling
	* @Use				temp_jisuan,scaling
	* @Output   		MS5611_Altitude
	* @Return   		MS5611_Altitude
*******************************************************************************/
float MS561101BA_get_altitude(float scaling)
{
	//return in centimeter
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
/*******************************************************************************
	* @Name				Kalman_filter_alt
	* @Description		kalman filter for altitude
	* @Input    		None
	* @Use				Altitude_X_hat,Altitude_P,Altitude_Q,Altitude_R,MS5611_Altitude
	* @Output   		Altitude_X_hat_minus,Altitude_P,Altitude_K,Altitude_X_hat
	* @Return   		None
*******************************************************************************/
void Kalman_filter_alt(void)
{
	//MS5611_Altitude in m
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
/*******************************************************************************
	* @Name				Kalman_filter_baro_climb
	* @Description		kalman filter for baro climb rate
	* @Input    		None
	* @Use				baro_X_hat,baro_P,baro_Q,baro_R,baro_climb_rate
	* @Output   		baro_X_hat_minus,baro_P,baro_K,baro_X_hat
	* @Return   		None
*******************************************************************************/
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
/*******************************************************************************
	* @Name				Kalman_filter_pressure
	* @Description		kalman filter for baro pressure
	* @Input    		None
	* @Use				pressure_X_hat,pressure_P,pressure_Q,pressure_R,MS5611_Pressure
	* @Output   		pressure_X_hat_minus,pressure_P,pressure_K,pressure_X_hat
	* @Return   		None
*******************************************************************************/
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
/*******************************************************************************
	* @Name				Derivative_Filter_baro_climb_rate
	* @Description		for climb rate
	* @Input    		None
	* @Use				Altitude_samples[6:0],Altitude_samples_time_stamps[6:0]
	* @Output   		baro_climb_rate
	* @Return   		None
*******************************************************************************/
void Derivative_Filter_baro_climb_rate(void)
{
	//baro_climb_rate in cm/s2
    baro_climb_rate = (10.0f * (f(1) - f(-1)) / (x(1) - x(-1)) + 16.0f * (f(2) - f(-2)) / (x(2) - x(-2)) + 6.0f * (f(3) - f(-3)) / (x(3) - x(-3))) * 0.03125f;
}



extern float gyrox_samples[7];
extern float gyroy_samples[7];
extern float gyroz_samples[7];
extern u8 gyro_sample_index;
extern float gyro_samples_time_stamps[7];
#define SAMPLES_RANGE2 7
#define f_gx(i) gyrox_samples[(((i+1)+3*SAMPLES_RANGE2/2)%SAMPLES_RANGE2)]
#define f_gy(i) gyroy_samples[(((i+1)+3*SAMPLES_RANGE2/2)%SAMPLES_RANGE2)]
#define f_gz(i) gyroz_samples[(((i+1)+3*SAMPLES_RANGE2/2)%SAMPLES_RANGE2)]
#define x_g(i) gyro_samples_time_stamps[(((i+1)+3*SAMPLES_RANGE2/2)%SAMPLES_RANGE2)]
extern float alpha[3];
/*******************************************************************************
	* @Name				Derivative_Filter_alpha
	* @Description		for angle acceleration
	* @Input    		None
	* @Use				gyrox_samples[6:0],gyroy_samples[6:0],gyroz_samples[6:0],gyro_samples_time_stamps[6:0]
	* @Output   		alpha[1:0]
	* @Return   		None
*******************************************************************************/
void Derivative_Filter_alpha(void)
{
	//degree per sencond*second by multiplier 1/32767*2000=0.0610370 we put this in p
    alpha[0] = (10.0f * (f_gx(1) - f_gx(-1)) / (x_g(1) - x_g(-1)) + 16.0f * (f_gx(2) - f_gx(-2)) / (x_g(2) - x_g(-2)) + 6.0f * (f_gx(3) - f_gx(-3)) / (x_g(3) - x_g(-3))) * 0.0003125f;

	alpha[1] = (10.0f * (f_gy(1) - f_gy(-1)) / (x_g(1) - x_g(-1)) + 16.0f * (f_gy(2) - f_gy(-2)) / (x_g(2) - x_g(-2)) + 6.0f * (f_gy(3) - f_gy(-3)) / (x_g(3) - x_g(-3))) * 0.0003125f;
}










