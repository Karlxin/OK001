/*--------------------------------version information top-------------------------------------------
Started at 2016
Created by Karlxin(410824290@qq.com)
Github:https://github.com/Karlxin/OK001.git
OpenKarlCopter
Version:dev_001

Notice:dev version created without experiment.If we want a stable version,we could go to github and
checkout a test version or a stable version.

features:more stable,more concise,more essential.

PS:
I am very grateful to Andrew Ng for the dreams of Artificial Intelligence and the open but valuable
course of machine learning in Coursera.

If you wanted experiments videos,please send emails to Karl with 410824290@qq.com.

You time is valuable,please please please do not do the things waste your time!

I hope some day we will meet each other with our dreams achieved.

Good Luck!

EL PSY CONGROO


--------------------------------version information top-------------------------------------------*/

#include "led.h"//light emitting diode head file 
#include "delay.h"//delay head file
#include "sys.h"//system head file
#include "usart.h"//universal synchronous asynchronous receiver transmitter head file
#include "stm32f10x.h"//stm32f10 controller head file
#include "moto.h"//motor head file
#include "timer.h"//timer 5 and 4 head file,timer 5 for system time,timer 4 for capture of pulse width modulation wave 
#include "wdg.h"//watch dog head file
#include "pwm.h"//timer 3 for output of pulse width modulation wave 
#include "imu.h"//inertial measurement unit
#include "mpu6050.h"//motion process unit with accelerometer and gyrometer
#include "inv_mpu.h"//mpu hardware config and reading config head file
#include "inv_mpu_dmp_motion_driver.h"//mpu hardware config and reading config head file
#include "math.h"//math head file
#include <stdio.h>//standard input output with buffer

extern void TIM3_PWM_Init(u16 arr, u16 psc);
extern void TIM4_Cap_Init(u16 arr, u16 psc);
extern void TIM5_Int_Init(u16 arr, u16 psc);
extern void Moto_Throttle(int16_t desthrottle);
extern void MS5611_init(void);
extern void IIC_Init(void);

u32 channel1_in, channel2_in, channel3_in, channel4_in;//between 1000~2000

extern void MS561101BA_RESET(void);

extern void ANO_DT_Send_MotoPWM(u16 m_1, u16 m_2, u16 m_3, u16 m_4, u16 m_5, u16 m_6, u16 m_7, u16 m_8);
extern void ANO_DT_Send_Senser(s16 a_x, s16 a_y, s16 a_z, s16 g_x, s16 g_y, s16 g_z, s16 m_x, s16 m_y, s16 m_z, s32 bar);
extern void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, s32 alt, u8 fly_model, u8 armed);
extern void ag2q2rpy(float gx, float gy, float gz, float ax, float ay, float az, float *pitch, float *roll, float *yaw);
extern float KalmanFilter(const float ResrcData, float ProcessNiose_Q, float MeasureNoise_R, float InitialPrediction);

extern u8 MPU_Init(void);
extern u8 mpu_dmp_init(void);

extern u8 ARMED;

u32 xitongshijian = 0; //system timer,resolution of 0.1ms,119hour
u32 haomiao = 0; //millisecond resolution
u32 erhaomiao = 0;
u32 wuhaomiao = 0;
u32 shihaomiao = 0; //ten milliseconds resolution
u32 ershihaomiao = 0;
u32 wushihaomiao = 0;
u32 yibaihaomiao = 0;//hundred milliseconds resolution
u32 miaozhong = 0; //second resolution

float roll, pitch, yaw;//Tait-Bryan angles φθψ,DMP hardware resolving ,roll,pitch,yaw
short aacx, aacy, aacz;//accelerometer raw data
short gyrox, gyroy, gyroz;//gyrometer raw data
short temp;//temperature
short gyrox_chushi, gyroy_chushi, gyroz_chushi; //at the beginning we grab some data from gyrometer as the sensor zero offset
float temp_gyxc = 0, temp_gyyc = 0, temp_gyzc = 0; //the temp for gyrometer offset,use average filter
short aacx_chushi, aacy_chushi, aacz_chushi; 
float temp_axc = 0, temp_ayc = 0, temp_azc = 0; 

int16_t deadzone = 20; //remote controller deadzone

int16_t cNd1 = 0, cNd2 = 0, cNd3 = 0, cNd4 = 0; //cyberNation

u32 gyxt = 0; //gyroxitong,use for getting the gyrometer offset,it record the system time

extern void MS561101BA_get_altitude(void);//calculate the altitude

float MS5611_Altitude;//using MS5611 pressure and temperature to calculate the Altitude.

extern void MS561101BA_getPressure(void);
extern void MS561101BA_GetTemperature(void);
float MS5611_Pressure;
int32_t  TEMP;//barometer temperature
float Pressure_chushi;
int32_t TEMP_chushi;
float temp_Pressure = 0;
float temp_TEMP = 0;

float roll_err, pitch_err, yaw_err;//error,roll_err=roll-desroll,desroll is the conversion of remote controller channel input
float desroll, despitch, desyaw;//expectation

short gyrox_out, gyroy_out, gyroz_out;//after sliding windows filter
short accz_out;//after 

extern void Gyro_filter(void);
extern void ANO_DT_Send_RCData(u16 thr, u16 yaw, u16 rol, u16 pit, u16 aux1, u16 aux2, u16 aux3, u16 aux4, u16 aux5, u16 aux6);
extern void Accz_filter(void);

float ACC_IIR_FACTOR;
extern void Calculate_FilteringCoefficient(float Time, float Cut_Off);
extern void ACC_IIR_Filter(void);
extern int16_t Throttle_constrain(int16_t Throttle);

extern void Altitude_hold_update(void);

//-----------barometer top
int64_t OFF_;


/*
C1 压力灵敏度 SENS|T1
C2  压力补偿  OFF|T1
C3  温度压力灵敏度系数 TCS
C4  温度系数的压力补偿 TCO
C5  参考温度 T|REF
C6  温度系数的温度 TEMPSENS
*/
uint16_t  Cal_C[7];//用于存放PROM中的6组数据1-6

uint32_t D1_Pres, D2_Temp;//数字压力值,数字温度值

/*
dT 实际和参考温度之间的差异
TEMP 实际温度
*/
int32_t dT, TEMP;
/*
OFF 实际温度补偿
SENS 实际温度灵敏度
*/
int64_t OFF, SENS;

int32_t P;//单位0.01mbar

uint32_t Pressure, Pressure_old, qqp;//大气压//单位0.01mbar

int64_t T2, TEMP2;//温度校验值
int64_t OFF2, SENS2;

uint32_t Pres_BUFFER[20];//数据组
uint32_t Temp_BUFFER[10];//数据组

//-----------barometer bottom

float scaling;
float temp_jisuan;

float temp_Altitude = 0;
float Altitude_chushi;


//---Alt karlman top
extern void Kalman_filter_alt(void);
float Altitude_minus = 0;//last barometer altitude converted by pressure
float Altitude_dt = 0.1;//the delta time
u32 Altitude_temp_time = 0; //record the time
float Altitude_R = 0.07; //3sigma 0.07*3=0.21,measuring variance.
float Altitude_Q = 0.0004; //process variance
float Altitude_K = 0; //kalman gain
float Altitude_X_hat = 0; //init predict
float Altitude_X_hat_minus = 0; //previous predict
float Altitude_P = 1; //error variance
//---Alt karlman bottom

//---Climb Karlman top
extern void Kalman_filter_climb(void);
float Climb_R = 0.03; //3sigma 0.03*3=0.09,measuring variance.
float Climb_Q = 0.0004; //process Variance
float Climb_K = 0; //kalman gain
float Climb_X_hat = 0; //init predict for Climb rate
float Climb_X_hat_minus = 0; //previous predict for Climb rate
float Climb_P = 1; //error variance
//---Climb karlman bottom

//---Kalman_filter_accz top
extern void Kalman_filter_accz(void);
float accz_dt = 0.01; //the delta time
u32 accz_temp_time = 0; //the record time
float accz_R = 23; //3sigma 23*3=69;
float accz_Q = 0.0004; //process Variance
float accz_K = 0; //kalman gain
float accz_X_hat = 15300; //init predict for accz
float accz_X_hat_minus = 0; //previous predict for accz
float accz_P = 140; //error variance

float acc_Climb = 0; //the climb rate
float acc_Climb_err = 0;
float acc_Climb_out = 0;
//---Kalman_filter_accz bottom

//---Kalman_filter_accy top
extern void Kalman_filter_accy(void);
float accy_dt = 0.01; //the delta time
u32 accy_temp_time = 0; //the record time
float accy_R = 23; //3sigma 23*3=69;
float accy_Q = 0.0004; //process Variance
float accy_K = 0; //kalman gain
float accy_X_hat = 300; //init predict for accy
float accy_X_hat_minus = 0; //previous predict for accy
float accy_P = 140; //error variance
//---Kalman_filter_accy bottom

//---Kalman_filter_accx top
extern void Kalman_filter_accx(void);
float accx_dt = 0.01; //the delta time
u32 accx_temp_time = 0; //the record time
float accx_R = 23; //3sigma 23*3=69;
float accx_Q = 0.0004; //process Variance
float accx_K = 0; //kalman gain
float accx_X_hat = -170; //init predict for accx
float accx_X_hat_minus = 0; //previous predict for accx
float accx_P = 140; //error variance
//---Kalman_filter_accx bottom

//---pressure kalman top
extern void Kalman_filter_pressure(void);
float pressure_dt = 0.01; //the delta time
u32 pressure_temp_time = 0; //the record time
float pressure_R = 7; //3sigma 7*3=21
float pressure_Q = 0.0004; //process Variance
float pressure_K = 0; //kalman gain
float pressure_X_hat = 103050; //init predict
float pressure_X_hat_minus = 0; //previous predict
float pressure_P = 40; //error variance
//---pressure kalman bottom

extern void complementation_filter(void);

float Ahd = 0; //altitude hold throttle offset 

extern void Sink_compensation(void);//sink update
float Scd = 0; //sink throttle offset

u32 debug[10]={0,0,0,0,0,0,0,0,0,0};//time pin,to observe the real frequency.

//deviation between 0.1ms，little boy do not be afraid

int main(void)
{
    //------------------------------initiation top------------------------------
    u8 jiesuokeyi = 0;//sign for ARMED
    u16 baochijiesuo = 0;//counter for holding ARMED channel input
    u16 baochijiasuo = 0;//counter for holding DISARMED channel input
    int16_t temp1, temp2,  desthrottle, temp4; //to convert channel pulse width modulation wave to expectation angle
    u8 i;//for for loop
    u8 USART1_Open = 0;//Open Serial by 500000 baud rate by setting it.
    u8 USART2_Open = 0;//Open Serial by 115200 baud rate by setting it.

    SystemInit();//over 0.02628ms
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//priority,parent divided by 0，1，2，3；subclass by 0,1(2:0),over 0.0004ms
    delay_init();//over 0.00114s

    TIM3_PWM_Init(19999, 71);  //50Hz,over 0.02154ms

    TIM_SetCompare1(TIM3, 2000);//set duty ratio,over 0.00047ms
    TIM_SetCompare2(TIM3, 2000);
    TIM_SetCompare3(TIM3, 2000);
    TIM_SetCompare4(TIM3, 2000);

    delay_ms(5000);

    TIM_SetCompare1(TIM3, 1000);
    TIM_SetCompare2(TIM3, 1000);
    TIM_SetCompare3(TIM3, 1000);
    TIM_SetCompare4(TIM3, 1000);

    delay_ms(3000);

    MPU_Init();//over 0.103s
    while(mpu_dmp_init())//is mpu initialization done
    {
        delay_ms(200);//延迟200ms
        delay_ms(200);//延迟200ms
    }//over 1.91s
    delay_ms(1000);
    IIC_Init();//inter integrated circuit bus protocol,over 0.00546ms
    delay_ms(100);
    MS561101BA_RESET();//over 0.35881ms
    delay_ms(100);
    MS5611_init();//over 0.0324s
    delay_ms(1000);

    TIM4_Cap_Init(0xffff, 72 - 1); //pulse width modulation capturer,1MHz,over 0.02819ms
    TIM5_Int_Init(9, 719); //system timer,resolution of 0.1ms

    delay_ms(300);


    //record the data of gyrometer averaged by 2 using 3s
    gyxt = xitongshijian; //record the system time,over 0.00172ms
    while(xitongshijian * 0.001f < gyxt * 0.001f + 3) 
    {
        if(!MPU_Get_Gyroscope(&gyrox, &gyroy, &gyroz))//get gyrometer raw data
        {
            temp_gyxc = ((float)gyrox  + temp_gyxc) * 0.5;//to get averaged initial data
            temp_gyyc = ((float)gyroy  + temp_gyyc) * 0.5;
            temp_gyzc = ((float)gyroz  + temp_gyzc) * 0.5;
        }

        if(!MPU_Get_Accelerometer(&aacx, &aacy, &aacz)) //over 0.6ms
        {
            temp_axc = ((float)aacx + temp_axc) * 0.5;
            temp_ayc = ((float)aacy + temp_ayc) * 0.5;
            temp_azc = ((float)aacz + temp_azc) * 0.5;
            Kalman_filter_accz();//call the Kalman filter
        }
    }//over 3s
    gyrox_chushi = (short)temp_gyxc;//record the initial data,over 0.00211ms
    gyroy_chushi = (short)temp_gyyc;
    gyroz_chushi = (short)temp_gyzc;

    aacx_chushi = (short)temp_axc;
    aacy_chushi = (short)temp_ayc;
    aacz_chushi = (short)accz_X_hat;

    for(i = 0; i < 20; i++)//20*100=2000milliseconds
    {
        delay_ms(100);

        MS561101BA_GetTemperature();//over 9.7ms
        temp_TEMP = ((float)TEMP + temp_TEMP) * 0.5;
    }//over 2.195s

    TEMP_chushi = (int32_t)temp_TEMP;//over 0.00203ms
    temp_jisuan    = (float)TEMP_chushi / 100.00 + 273.15f;//for the altitude conversion

    for(i = 0; i < 20; i++)
    {
        delay_ms(100);

        MS561101BA_getPressure();//over 9.7ms
        Kalman_filter_pressure();
    }//over 2.1945s

    Pressure_chushi = pressure_X_hat_minus;

    scaling = pressure_X_hat_minus / Pressure_chushi;//for the altitude conversion
    MS561101BA_get_altitude();//over 0.02ms
    Altitude_chushi = MS5611_Altitude;

    if(USART1_Open)
    {
        Uart1_Init(500000);//anonymous 4.06
    }
    else if(USART2_Open)
    {
        Uart1_Init(115200);//ATKXCOMV2.0
    }

    //Calculate_FilteringCoefficient(0.0050000, 10.0000000);//calculating aacz using low pass filter

    LED_Init();//over 0.01044ms
    LED0 = 1; //Darkening red LED,showing ARMED
    LED1 = 0; //Lightening green LED，showing DISARMED

    haomiao = xitongshijian * 0.1f;
    erhaomiao = xitongshijian * 0.05f;//noticing that we disorder the schedule before
    wuhaomiao = xitongshijian * 0.02f;
    shihaomiao = xitongshijian * 0.01f;
    ershihaomiao = xitongshijian * 0.005f;
    wushihaomiao = xitongshijian * 0.002f;
    yibaihaomiao = xitongshijian * 0.001f;
    miaozhong = xitongshijian * 0.0001f;
    //------------------------------initiation bottom-----------------------------

    while(1)//using 12s to get there
    {
        //ms top
        if(xitongshijian * 0.1f > haomiao + 1)
        {
            haomiao = xitongshijian * 0.1f;
			debug[0]++;
			
            //read MPU top
            if(!MPU_Get_Accelerometer(&aacx, &aacy, &aacz)) //get acc data,over 0.6ms
            {
                Kalman_filter_accz();
                accz_dt = (float)(xitongshijian - accz_temp_time) * 0.001;//how much time between two visit
                accz_temp_time = xitongshijian;//record the time
                acc_Climb += (accz_X_hat_minus - aacz_chushi) * 0.0005978 * accz_dt;//accelerometer integral
                acc_Climb_out = acc_Climb - acc_Climb_err; //error adjust by using baro derivative.
                //Kalman_filter_accy();
                //Kalman_filter_accx();
            }

            if(!MPU_Get_Gyroscope(&gyrox, &gyroy, &gyroz))  //get gyro data,over 0.6ms
            {
                Gyro_filter();//sliding window filter,over 0.02ms
            }
            //read MPU bottom
        }
        //ms bottom

        //two ms top
        if(xitongshijian * 0.05f > erhaomiao + 1)
        {
            erhaomiao = xitongshijian * 0.05f; //visiting by two milliseconds resolution
			debug[1]++;
        }
        //two ms bottom

        //five ms top
        if(xitongshijian * 0.02f > wuhaomiao + 1)
        {
            wuhaomiao = xitongshijian * 0.02f; //visiting by five milliseconds resolution
			debug[2]++;
            mpu_dmp_get_data(&pitch, &roll, &yaw);//over amazing 52ms,move 50ms delay and we got 2.1ms,use dmp hardware
            roll_err = roll - desroll; //get roll_err
            pitch_err = pitch - despitch;
            yaw_err = yaw - desyaw;
            //ag2q2rpy(gyrox_sr+0.0553938, gyroy_sr-0.0170442, gyroz_sr-0.0159790, aacx-960, aacy-350, aacz+1085, &pitch, &roll, &yaw);//计算耗时0.25ms;
        }
        //five ms bottom

        //ten ms top
        if(xitongshijian * 0.01f > shihaomiao + 1)
        {
            shihaomiao = xitongshijian * 0.01f; //visiting by ten milliseconds resolution
			debug[3]++;
			

            //0.09*65536/4=1475(short为int16_t,65536 presentation for ±2g)
            //desthrottle -= (int16_t)(0.03 * (float)(accz_out - aacz_chushi)); //当aacz大于初始时，说明飞机向上,油门应该减小,这个太恐怖了，伤到我了

            //-----------------------------Control and Throttle update top----------------------------------------------
            if(jiesuokeyi) //if ARMED,over 0.01204ms
            {
                if(channel3_in > 1100)//over 0.01519ms
                {
                    desthrottle = channel3_in;//over 0.01208ms

                    if(channel1_in < 1507 - deadzone || channel1_in > 1507 + deadzone)//channel1 is not in deadzone ,over 0.01558ms
                    {
                        temp1 =  channel1_in - 1507 ; 
                        desroll = (float)temp1 * 0.0361446; //covert to ±15°

                    }
                    else//channel1 is in deadzone
                    {
                        desroll = 0;
                    }

                    if(channel2_in < 1508 - deadzone || channel2_in > 1508 + deadzone)
                    {
                        temp2 = 1508 - channel2_in;
                        despitch = (float)temp2 * 0.0361446; 

                    }
                    else//channel2 is in deadzone
                    {
                        despitch = 0;
                    }

                    if(channel4_in < 1507 - deadzone || channel4_in > 1507 + deadzone)
                    {
                        temp4 = 1507 - channel4_in;
                        desyaw = (float)temp4 * 0.0361446; 
                        desyaw = 0; //we just do not control yaw temporarily

                    }
                    else//channel4 is in deadzone
                    {
                        desyaw = 0;
                    }
                }
                else//channel3<1100
                {
                    desthrottle = 0;//等于1000太恐怖了，根本降不下油门，在发散的时候救都救不回
                    desroll = despitch = desyaw = 0;
                    Moto_PwmRflash(0, 0, 0, 0);//core 1 called in place 1
                }

                if(channel3_in < 1100 && channel4_in < 1100)//油门小于1100，偏航小于1100,也就是油门最下，偏航最左
                {
                    if(baochijiasuo == 0)//保持加锁为零
                    {
                        baochijiasuo = miaozhong;//保持加锁等于系统开机以来的秒钟值
                    }
                    if((miaozhong - baochijiasuo) >= 8)//油门最小，偏航最左已经过去八秒了
                    {
                        jiesuokeyi = 0;//遥控不可以解锁
                        LED1 = 0;//绿灯亮表示解锁不可以
                        LED0 = 1;//红灯灭表示不能让电机动了
                    }
                }
                else
                {
                    baochijiasuo = 0;//保持加锁计时器重新归零
                }
            }
            else//解锁不可以
            {
                desthrottle = 0;//等于1000太恐怖了，根本降不下油门，在发散的时候救都救不回
                desroll = despitch = desyaw = 0;//想要的横滚俯仰偏航均为零
                Moto_PwmRflash(0, 0, 0, 0);//全部油门最小化,over 0.02ms,core 1 called in 2 place

                if(channel3_in < 1100 && channel4_in > 1900)//油门小于1100，偏航角大于1900,也就是油门最下，偏航最右
                {
                    if(baochijiesuo == 0)//保持解锁计时器为零
                    {
                        baochijiesuo = miaozhong;//保持解锁计时器赋予它系统开启的秒钟值
                    }
                    if((miaozhong - baochijiesuo) >= 5)//距离保持解锁状态已经过去五秒了
                    {
                        jiesuokeyi = 1;//解锁可以标志重新赋值为可以
                        LED1 = 1; //绿灯灭表示解锁可
                        LED0 = 0; //红灯亮表示要小心电机可以转了
                    }
                }
                else//油门不小于1100或者偏航角不大于1900,也就是油门不是最下或者偏航不是最右
                {
                    baochijiesuo = 0;//保持解锁重新归零
                }
            }

            if(channel3_in > 1100)//只有油门大于1100时才允许更新油门和自动控制量
            {
                cyberNation();//update motor,over 0.06ms
                //Altitude_hold_update();//altitude holding superposition
                Sink_compensation();//sink offset superposition
                Moto_Throttle(desthrottle);//core 2 called in 1 place
            }
            //-----------------------------Control and Throttle update bottom--------------------------------------------

        }
        //ten ms bottom

        //twenty ms top
        if(xitongshijian * 0.005f > ershihaomiao + 1)
        {
            ershihaomiao = xitongshijian * 0.005f; //visiting by twenty milliseconds resolution
			debug[4]++;
        }
        //twenty ms bottom

        //fifty ms top
        if(xitongshijian * 0.002f > wushihaomiao + 1)
        {
            wushihaomiao = xitongshijian * 0.002f; //visiting by fifty milliseconds resolution
            debug[5]++;

            //serial top
            if(USART1_Open)
            {
                ANO_DT_Send_Status((float)roll, (float)pitch, (float)yaw, (s32)MS5611_Altitude, (u8)0, (u8)0);//over 0.4ms
                //ANO_DT_Send_Senser((s16)aacx * 0.05978, (s16)aacy * 0.05978, (s16)aacz * 0.05978, gyrox_out, gyroy_out, gyroz_out, (s16)Scd, (s16)Ahd, (s16)0, (s32)MS5611_Altitude * 100); //over 0.5ms
                ANO_DT_Send_MotoPWM((u16) cNd1, (u16) cNd2, (u16) cNd3, (u16) cNd4, (u16) 0, (u16) 0, (u16) 0, (u16) 0); //over 0.5ms
                ANO_DT_Send_RCData((u16)channel3_in, (u16) channel4_in, (u16) channel1_in, (u16) channel2_in, (u16) 0, (u16) 0, (u16) 0, (u16) 0, (u16) 0, (u16) 0); //0.5ms

                //ANO_DT_Send_Senser(accx_X_hat_minus, accy_X_hat_minus, accz_X_hat_minus, gyrox_out, gyroy_out, gyroz_out,(s16)Scd,(s16)0,(s16)0,(s32)0);//over 0.5ms
                //ANO_DT_Send_Status(acc_Climb*100, acc_Climb_out*100, Climb_X_hat_minus*100, (s32)MS5611_Altitude*100, (u8)0, (u8)0); //over 0.4ms
                //ANO_DT_Send_Senser((s16)channel1_in, (s16)channel2_in, (s16)channel3_in, (s16)channel4_in, gyroy_out, gyroz_out,(s16)Scd,(s16)0,(s16)0,(s32)0);//over 0.5ms
                //ANO_DT_Send_Senser((s16)(979.0f * (cosf(roll * 0.0174533)*cosf(pitch * 0.0174533))), (s16)aacy * 0.05978, (s16)aacz * 0.05978, gyrox_out, gyroy_out, gyroz_out, (s16)0, (s16)0, (s16)0, (s32)MS5611_Altitude * 100); //over 0.5ms
            }
            else if(USART2_Open)
            {
                //printf("  MS5611_Altitude =%fm\r\n", MS5611_Altitude);
            }
            //serial bottom
        }
        //fifty ms bottom

        //hundred ms top
        if(xitongshijian * 0.001f > yibaihaomiao + 1)
        {
            yibaihaomiao = xitongshijian * 0.001f; //visit by hundred milliseconds resolution
			 debug[6]++;
			
            //MS561101BA_GetTemperature();//over 9.1ms
            MS561101BA_getPressure();//over 9.1ms
            Kalman_filter_pressure();
            scaling = pressure_X_hat_minus / Pressure_chushi;//using to calculate the height
            MS561101BA_get_altitude();//0.02ms
            //Kalman_filter_alt();//0.01ms
            Altitude_dt = 0.001f * (float)(xitongshijian - Altitude_temp_time);
            Altitude_temp_time = xitongshijian;
            Kalman_filter_climb();
            complementation_filter();//update acc_Climb_err
            //ANO_DT_Send_Senser(aacx-aacx_chushi, aacy-aacy_chushi, acc_Climb_X_hat_minus-aacz_chushi, gyrox_out, gyroy_out, gyroz_out, (s16)0, (s16)0, (s16)0, (s32)MS5611_Pressure);
            //ANO_DT_Send_Status(MS5611_Altitude-Altitude_chushi, Altitude_X_hat_minus-Altitude_chushi, (Altitude_X_hat-Altitude_X_hat_minus)/Altitude_dt, (s32)MS5611_Altitude, (u8)0, (u8)0);//over 0.4ms
            //ANO_DT_Send_Status((Altitude_X_hat-Altitude_X_hat_minus)/Altitude_dt, acc_Climb, Climb_X_hat_minus, (s32)MS5611_Altitude, (u8)0, (u8)0);//over 0.4ms

            //ANO_DT_Send_Status(acc_Climb, acc_Climb_out, Climb_X_hat_minus, (s32)MS5611_Altitude, (u8)0, (u8)0); //over 0.4ms
            Altitude_minus = MS5611_Altitude;
        }
        //hundred ms bottom

        //seconds top
        if(xitongshijian * 0.0001f > miaozhong + 1)
        {
            miaozhong = xitongshijian * 0.0001f; //visit by seconds resolution
			debug[7]++;
			
			
            //printf("  TEMP =%.2f℃\r\n", (float)TEMP / 100.00);
            //printf("  MS5611_Pressure =%fmbar\r\n", MS5611_Pressure / 100);
            //printf("  MS5611_Altitude =%f\r\n", MS5611_Altitude-Altitude_chushi);
            //printf("  Altitude_X_hat =%f\r\n", Altitude_X_hat_minus-Altitude_chushi);

            //            temp=MPU_Get_Temperature();

            //            printf("  mpu temp=%.2f\r\n", temp*0.01f);//mpu温度
            //            printf("  Temp : %.2f ℃\r\n", TEMP * 0.01f);            //串口输出原始数据，气压计读到的温度
            //            printf("  height : %.2f m\r\n", (float)MS561101BA_get_altitude());
            //            printf("  Pressure : %.2f mbar\r\n\r\n\r\n", Pressure * 0.01f); //串口输出原始数据

            //            printf("  aacx=%f\r\n", aacx_s);//x轴加速度,printf可以输出给ATKCOM串口捕捉并在PC上显示
            //            printf("  aacy=%f\r\n", aacy_s);
            //            printf("  aacz=%f\r\n\r\n", aacz_s);

            //            printf("  gyrox=%f\r\n", gyrox_sd);
            //            printf("  gyroy=%f\r\n", gyroy_sd);
            //            printf("  gyroz=%f\r\n\r\n", gyroz_sd);
            //
            //            printf("    roll=%f 度\r\n", roll); //用软解姿态不需要转换
            //            printf("  pitch=%f 度\r\n", pitch);
            //            printf("  yaw=%f 度\r\n\r\n", yaw);

            //            printf("  aacx=%d\r\n", aacx);//x轴加速度
            //            printf("  aacy=%d\r\n", aacy);
            //            printf("  aacz=%d\r\n\r\n", aacz);

            //            printf("  gyrox=%d\r\n", gyrox);
            //            printf("  gyroy=%d\r\n", gyroy);
            //            printf("  gyroz=%d\r\n\r\n", gyroz);

            //            printf("  delta_gyrox=%d\r\n", gyrox - gyrox_chushi);
            //            printf("  delta_gyroy=%d\r\n", gyroy - gyroy_chushi);
            //            printf("  delta_gyroz=%d\r\n\r\n", gyroz - gyroz_chushi);

        }
        //seconds bottom
    }
}

//I am Hououin Kyouma,a mad scientist,and the destroyer of this world's ruling structure.
//Failure is out of the question.

//Okay.I believe in you.

//little butterfly do not be afraid,the turbulence can save the world.
//OORGNO CYSPLE


