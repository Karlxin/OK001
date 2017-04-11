/*--------------------------------version information top-------------------------------------------
Started at 2016
Created by Karlxin(410824290@qq.com)
Github:https://github.com/Karlxin/OK001.git
OpenKarlCopter
Version:dev_003

features:To be continued...

Notice:dev version created without experiment.If we want a stable version,we could go to github and
checkout a test version or a stable version.

postscript:
I am very grateful to Andrew Ng for the dreams of Artificial Intelligence and the open but valuable
course of machine learning in Coursera.

If you wanted experiments videos,please send emails to Karl with 410824290@qq.com.

Your time is valuable,please please please do not do the things wasting your time!

no time for us to waste.do our best to build the artificial intelligence

I hope some day we will meet each other with our dreams achieved.

Good Luck!

EL PSY CONGROO
--------------------------------version information bottom-------------------------------------------*/

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
extern void MS561101BA_RESET(void);

extern void ANO_DT_Send_MotoPWM(u16 m_1, u16 m_2, u16 m_3, u16 m_4, u16 m_5, u16 m_6, u16 m_7, u16 m_8);
extern void ANO_DT_Send_Senser(s16 a_x, s16 a_y, s16 a_z, s16 g_x, s16 g_y, s16 g_z, s16 m_x, s16 m_y, s16 m_z, s32 bar);
extern void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, s32 alt, u8 fly_model, u8 armed);

extern u8 MPU_Init(void);
extern u8 mpu_dmp_init(void);

u32 xitongshijian = 0; //system timer,resolution of 0.1ms,119hour
u32 haomiao = 0; //millisecond resolution
u32 erhaomiao = 0;
u32 wuhaomiao = 0;
u32 shihaomiao = 0; //ten milliseconds resolution
u32 ershihaomiao = 0;
u32 wushihaomiao = 0;
u32 yibaihaomiao = 0;//hundred milliseconds resolution
u32 yibaihaomiao2 = 0;
u32 miaozhong = 0; //second resolution
u32 ermiaozhong = 0;
u32 wumiaozhong = 0;
u32 shimiaozhong = 0;
u32 ershimiaozhong = 0;
u32 wushimiaozhong = 0;
u32 yibaimiaozhong = 0;
u32 erbaimiaozhong = 0;
u32 wubaimiaozhong = 0;
u32 yiqianmiaozhong = 0;

u32 channel1_in, channel2_in, channel3_in, channel4_in;//between 1000~2000
u32 STOPPING_THROTTLE = 1400;

float roll, pitch, yaw;//Tait-Bryan angles φθψ,DMP hardware resolving ,roll,pitch,yaw
short aacx, aacy, aacz;//accelerometer raw data
short gyrox, gyroy, gyroz;//gyrometer raw data
short gyro[3] = {0, 0, 0};
short temp;//temperature
short gyrox_chushi, gyroy_chushi, gyroz_chushi; //at the beginning we grab some data from gyrometer as the sensor zero offset
short gyro_chushi[3] = {0, 0, 0};
float temp_gyxc = 0, temp_gyyc = 0, temp_gyzc = 0; //the temp for gyrometer offset,use average filter
short aacx_chushi, aacy_chushi, aacz_chushi;
float temp_axc = 0, temp_ayc = 0, temp_azc = 0;

u32 gyro_temp_time = 0;
float gyro_temp_dt = 0;
short gyrox_temp = 0, gyroy_temp = 0, gyroz_temp = 0;


int16_t deadzone = 20; //remote controller deadzone

int16_t cNd1_theta = 0, cNd2_theta = 0, cNd3_theta = 0, cNd4_theta = 0; //cyberNation
int16_t cNd1_omega = 0, cNd2_omega = 0, cNd3_omega = 0, cNd4_omega = 0; //cyberNation
int16_t cNd1_alpha = 0, cNd2_alpha = 0, cNd3_alpha = 0, cNd4_alpha = 0; //cyberNation

extern float MS561101BA_get_altitude(float scaling);//calculate the altitude

float MS5611_Altitude;//using MS5611 pressure and temperature to calculate the Altitude.
float Altitude_out = 0;

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
short accz_out;//after filter, and trans

extern void Gyro_filter(void);
extern void ANO_DT_Send_RCData(u16 thr, u16 yaw, u16 rol, u16 pit, u16 aux1, u16 aux2, u16 aux3, u16 aux4, u16 aux5, u16 aux6);
extern void Accz_filter(void);

float ACC_IIR_FACTOR;
extern void Calculate_FilteringCoefficient(float Time, float Cut_Off);
extern void ACC_IIR_Filter(void);

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

//---baro Alt karlman top
extern void Kalman_filter_alt(void);
float Altitude_minus = 0;//last barometer altitude converted by pressure
float Altitude_dt = 0.1;//the delta time
u32 Altitude_temp_time = 0; //record the time
float Altitude_R = 20; //measurement variance ±20cm
float Altitude_Q = 0.01; //process variance 1/2*5*0.1^2
float Altitude_K = 0; //kalman gain
float Altitude_X_hat = 0; //init predict
float Altitude_X_hat_minus = 0; //previous predict
float Altitude_P = 0; //error variance
//---baro Alt karlman bottom

//---baro Climb Karlman top
extern void Kalman_filter_baro_climb(void);
float baro_climb_R = 1;
float baro_climb_Q = 1; //process Variance,3/10=0.3
float baro_climb_K = 0; //kalman gain
float baro_climb_X_hat = 0; //init predict for Climb rate
float baro_climb_X_hat_minus = 0; //previous predict for Climb rate
float baro_climb_P = 0; //error variance
//---baro Climb karlman bottom

//---Kalman_filter_accz top
extern void Kalman_filter_accz(void);//in 32767
float accz_dt = 0.01; //the delta time
u32 accz_temp_time = 0; //the record time
float accz_R = 83; //positive negative 67,measurement variance
float accz_Q = 0.01347916; //process Variance
float accz_K = 0; //kalman gain
float accz_X_hat = 15300; //init predict for accz
float accz_X_hat_minus = 0; //previous predict for accz
float accz_P = 140; //error variance

float acc_climb_rate = 0; //the climb rate
float acc_climb_abs_err = 0;
float baro_climb_rate = 0;
float acc_climb_rate_out = 0;
//---Kalman_filter_accz bottom

float rate2 = 0;
float errInt2 = 0;
float accz_IMU = 0;
float err_rate1_rate2 = 0;

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
float pressure_R = 20; //3sigma 7*3=21
float pressure_Q = 0.0169; //process Variance
float pressure_K = 0; //kalman gain
float pressure_X_hat = 0; //init predict
float pressure_X_hat_minus = 0; //previous predict
float pressure_P = 0.1; //error variance
//---pressure kalman bottom

//---gyrometer kalman top
extern void Kalman_filter_gyro(void);
//float gyro_dt[3] = {0.01,0.01,0.01}; //the delta time
//u32 gyro_temp_time = 0; //the record time
float gyro_R[3] = {1, 1, 1}; //3sigma measurement variance
float gyro_Q[3] = {3965497, 991221, 6729}; //process Variance
float gyro_K[3] = {0, 0, 0}; //kalman gain
float gyro_X_hat[3] = {0, 0, 0}; //init predict
float gyro_X_hat_minus[3] = {0, 0, 0}; //previous predict
float gyro_P[3] = {1, 1, 1}; //error variance,6sigma
//---gyrometer kalman bottom

extern void complementation_filter(void);

float Ahd = 0; //altitude hold throttle offset

extern void Sink_compensation(void);//sink update
float Scd = 0; //sink throttle offset

u32 debug[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; //time pin,to observe the real frequency.

//deviation between 0.1ms，little boy do not be afraid

int16_t d1 = 0, d2 = 0, d3 = 0, d4 = 0;

float alphax = 0, alphay = 0, alphaz = 0;

extern void cyberNation_theta(void);
extern void cyberNation_omega(void);
extern void cyberNation_alpha(void);

float alphax_out = 0;
float alphay_out = 0;
float alphaz_out = 0;

extern void Altitude_filter(void);

extern void Filter_Altitude(void);

float Altitude_samples[7] = {0, 0, 0, 0, 0, 0, 0};
float Altitude_samples_time_stamps[7] = {0, 0, 0, 0, 0, 0, 0};
u8 Altitude_sample_index = 0;
u8 Altitude_samples_full = 0;

extern void Derivative_Filter(void);

u32 complementary_count = 1;

u32 stopping_throttle_upper_bound_coarse = 1600;//the hover upper bound throttle
u32 stopping_throttle_lower_bound_coarse = 1400;
u32 stopping_throttle_upper_bound_fine=1600;
u32 stopping_throttle_lower_bound_fine=1400;
u32 stopping_throttle_temp;
float baro_trigger = 8; //the trigger to record channel3_in i.e. throttle in
u8 stopping_throttle_upper_recorded = 0;//flag for recording done
u8 stopping_throttle_lower_recorded = 0;
u8 stopping_throttle_both_recorded = 0;
u32 hover_range_top = 10;
u32 hover_range_bottom = 50;


short accz_integral_deadzone = 3; //to create a deadzone and deal with steady noise

//main top
int main(void)
{
    //------------------------------initiation top------------------------------
    u8 jiesuokeyi = 0;//sign for ARMED
    u16 baochijiesuo = 0;//counter for holding ARMED channel input
    u16 baochijiasuo = 0;//counter for holding DISARMED channel input
    u8 flymode = 0;
    int16_t temp1, temp2,  desthrottle, temp4; //to convert channel pulse width modulation wave to expectation angle
    u8 i;//for for loop
    u8 USART1_Open = 1;//Open Serial by 500000 baud rate by setting it.
    u8 USART2_Open = 0;//Open Serial by 115200 baud rate by setting it.
    u8 kalman_gyro_Open = 0; //Open kalman instead of sliding window for gyro.we set it to 1 to open.

    SystemInit();//over 0.02628ms
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//priority,parent divided by 0，1，2，3；subclass by 0,1(2:0),over 0.0004ms
    delay_init();//over 0.00114s

    TIM3_PWM_Init(19999, 71);  //50Hz,over 0.02154ms

    TIM_SetCompare1(TIM3, 2000);//set duty ratio,over 0.00047ms
    TIM_SetCompare2(TIM3, 2000);
    TIM_SetCompare3(TIM3, 2000);
    TIM_SetCompare4(TIM3, 2000);

    delay_ms(5000);//delay 5000 millisecond

    TIM_SetCompare1(TIM3, 1000);
    TIM_SetCompare2(TIM3, 1000);
    TIM_SetCompare3(TIM3, 1000);
    TIM_SetCompare4(TIM3, 1000);

    delay_ms(3000);

    MPU_Init();//over 0.103s
    while(mpu_dmp_init())//is mpu initialization done
    {
        delay_ms(200);
        delay_ms(200);
    }//over 1.91s
    delay_ms(1000);
    IIC_Init();//inter integrated circuit bus protocol,over 0.00546ms
    delay_ms(100);
    MS561101BA_RESET();//over 0.35881ms
    delay_ms(100);
    MS5611_init();//over 0.0324s
    delay_ms(300);


    for(i = 0; i < 100; i++)
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
    }
    gyro_chushi[0] = (short)temp_gyxc;//record the initial data,over 0.00211ms
    gyro_chushi[1] = (short)temp_gyyc;
    gyro_chushi[2] = (short)temp_gyzc;

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

    if(USART1_Open)
    {
        Uart1_Init(500000);//anonymous 4.06
    }
    else if(USART2_Open)
    {
        Uart1_Init(115200);//ATKXCOMV2.0
    }

    delay_ms(10);

    TIM4_Cap_Init(0xffff, 72 - 1); //pulse width modulation capturer,1MHz,over 0.02819ms
    TIM5_Int_Init(9, 719); //system timer,resolution of 0.1ms

    delay_ms(10);

    LED_Init();//over 0.01044ms
    LED0 = 1; //Darkening red LED,showing DISARMED
    LED1 = 0; //Lightening green LED，showing DISARMED

    delay_ms(10);

    haomiao = xitongshijian * 0.1f;
    erhaomiao = xitongshijian * 0.05f;//noticing that we disorder the schedule before
    wuhaomiao = xitongshijian * 0.02f;
    shihaomiao = xitongshijian * 0.01f;
    ershihaomiao = xitongshijian * 0.005f;
    wushihaomiao = xitongshijian * 0.002f;
    yibaihaomiao = xitongshijian * 0.001f;
    yibaihaomiao2 = xitongshijian * 0.001f;
    miaozhong = xitongshijian * 0.0001f;
    ermiaozhong = xitongshijian * 0.00005f;
    wumiaozhong = xitongshijian * 0.00002f;
    shimiaozhong = xitongshijian * 0.00001f;
    ershimiaozhong = xitongshijian * 0.000005f;
    wushimiaozhong = xitongshijian * 0.000002f;
    yibaimiaozhong = xitongshijian * 0.000001f;
    erbaimiaozhong = xitongshijian * 0.0000005f;
    wubaimiaozhong = xitongshijian * 0.0000002f;
    yiqianmiaozhong = xitongshijian * 0.0000001f;
    //------------------------------initiation bottom-----------------------------

    //loop top
    while(1)//using 8s to get there
    {
        //ms top
        if(xitongshijian * 0.1f > haomiao + 1)
        {
            haomiao = xitongshijian * 0.1f;
            debug[0]++;

            //read MPU top
            if(!MPU_Get_Accelerometer(&aacx, &aacy, &aacz)) //get acc data,over 0.6ms
            {
                Accz_filter();//sliding window filter
                //Kalman_filter_accz();
            }

            if(!MPU_Get_Gyroscope(&gyro[0], &gyro[1], &gyro[2]))  //get gyro data,over 0.6ms
            {
                Gyro_filter();//sliding window filter,over 0.02ms
                if(kalman_gyro_Open)
                {
                    //Kalman_filter_gyro();
                    gyrox_out = gyro_X_hat_minus[0];
                    gyroy_out = gyro_X_hat_minus[1];
                    gyroz_out = gyro_X_hat_minus[2];
                }
            }
            //read MPU bottom
        }
        //ms bottom


        //two ms top
        if(xitongshijian * 0.05f > erhaomiao + 1)
        {
            erhaomiao = xitongshijian * 0.05f; //visiting by two milliseconds resolution
            debug[1]++;

            if(!stopping_throttle_both_recorded)
            {
                if(jiesuokeyi&&(stopping_throttle_lower_bound_coarse<channel3_in)&&(channel3_in<stopping_throttle_upper_bound_coarse))
                {
                    if(!stopping_throttle_upper_recorded)
                    {
                        if(baro_trigger < baro_climb_rate)
                        {
                            stopping_throttle_upper_bound_fine = channel3_in + hover_range_top; //record the upper throttle
                            stopping_throttle_upper_recorded = 1; //set flag
                        }
                    }
                    if(!stopping_throttle_lower_recorded)
                    {
                        if(stopping_throttle_upper_recorded)
                        {
                            stopping_throttle_lower_bound_fine = stopping_throttle_upper_bound_fine -hover_range_top- hover_range_bottom; //record the upper throttle
                            stopping_throttle_lower_recorded = 1; //set flag
                        }
                    }
                    if(stopping_throttle_upper_recorded && stopping_throttle_lower_recorded)
                    {
                        stopping_throttle_both_recorded = 1; //set all done flag
                        LED0 = 1; //Darkening red LED，showing range recorded.
                    }
                }
            }

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

        }
        //five ms bottom

        //ten ms top
        if(xitongshijian * 0.01f > shihaomiao + 1)
        {
            shihaomiao = xitongshijian * 0.01f; //visiting by ten milliseconds resolution
            debug[3]++;

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
                    d1 = 0;
                    d2 = 0;
                    d3 = 0;
                    d4 = 0;
                    Moto_PwmRflash(0, 0, 0, 0);//core 1 called in place 1
                }

                if(channel3_in < 1100 && channel4_in < 1100)//throttle<1100,yaw<1100,i.e. stick of throttle to the very left and down
                {
                    if(baochijiasuo == 0)//counter for holding disarming =0
                    {
                        baochijiasuo = miaozhong;//set counter for holding disarming to system seconds
                    }
                    if((miaozhong - baochijiasuo) >= 8)//holding stick for disarming maintained over eight seconds
                    {
                        jiesuokeyi = 0;//reset flag of disarming
                        LED1 = 0;//lightening green led stand for disarming
                        LED0 = 1;//darkening red led stand for disarming
                    }
                }
                else
                {
                    baochijiasuo = 0;//reset counter of holding disarming
                }
            }
            else//status is disarming
            {
                d1 = 0;
                d2 = 0;
                d3 = 0;
                d4 = 0;
                Moto_PwmRflash(0, 0, 0, 0);//disarm all the motor,over 0.02ms,core 1 called in 2 place

                if(channel3_in < 1100 && channel4_in > 1900)//throttle<1100,yaw>1900,i.e.,the stick of throttle to the very right and bottom
                {
                    if(baochijiesuo == 0)//counter of arming=0
                    {
                        baochijiesuo = miaozhong;//get the system time
                    }
                    if((miaozhong - baochijiesuo) >= 5)//holding arming over five seconds
                    {
                        jiesuokeyi = 1;//set flag of arming
                        LED1 = 1; //green led dark stand for arming
                        LED0 = 0; //red led light stand for arming.please be careful of rotating propeller
                    }
                }
                else//channel3_in>1100,channel4_in<1900,that is throttle stick above bottom,yaw stick is not on the right
                {
                    baochijiesuo = 0;//holding ARMED counter reset.
                }
            }

            if(channel3_in > 1100 && jiesuokeyi) //only when channel3 >1100 and jiesuokeyi will update motor controlling
            {
                //cyberNation_alpha();//noise is too big
                cyberNation_omega();
                cyberNation_theta();
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
            if(channel3_in > 1100 && jiesuokeyi)
            {
                Altitude_hold_update();//this frequency may be enough.altitude holding superposition
            }
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
                ANO_DT_Send_Status((float)roll, (float)pitch, (float)yaw, (s32)MS5611_Altitude, (u8)flymode, (u8)jiesuokeyi); //over 0.4ms
                ANO_DT_Send_MotoPWM((u16) d1, (u16) d2, (u16) d3, (u16) d4, (u16) stopping_throttle_upper_recorded, (u16) stopping_throttle_lower_recorded, (u16) stopping_throttle_both_recorded, (u16) 0); //over 0.5ms
                ANO_DT_Send_RCData((u16)channel3_in, (u16) channel4_in, (u16) channel1_in, (u16) channel2_in, (u16) stopping_throttle_upper_bound_fine, (u16) stopping_throttle_lower_bound_fine, (u16) 0, (u16) 0, (u16) 0, (u16) 0); //0.5ms
                ANO_DT_Send_Senser((s16)aacx , (s16)aacy, (s16)(accz_X_hat_minus * cosf(pitch)*cosf(roll) - aacz_chushi) * 0.05978, (s16)gyrox_out, (s16)gyroy_out, (s16)Altitude_X_hat_minus, (s16)baro_climb_X_hat_minus, (s16)acc_climb_rate, (s16)baro_climb_rate, (s32)MS5611_Altitude);

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

            MS561101BA_getPressure();//over 9.1ms
            if(yibaihaomiao < 100)
            {
                Pressure_chushi = 0.5 * (Pressure_chushi + MS5611_Pressure);
            }
            else
            {
                Altitude_out = MS561101BA_get_altitude(MS5611_Pressure / Pressure_chushi);
                Kalman_filter_alt();
                Altitude_samples[Altitude_sample_index] = Altitude_X_hat_minus;
                //Altitude_samples[Altitude_sample_index] = MS5611_Altitude;
                Altitude_samples_time_stamps[Altitude_sample_index] = xitongshijian * 0.0001f;
                Altitude_sample_index++;
                if(Altitude_sample_index == 7)
                {
                    Altitude_sample_index = 0;
                    if(!Altitude_samples_full)
                    {
                        Altitude_samples_full = 1;
                    }
                }
                if(Altitude_samples_full)
                {
                    Derivative_Filter();
                    Kalman_filter_baro_climb();
                    /*
                    if(stopping_throttle_upper_bound_fine < channel3_in || channel3_in < stopping_throttle_lower_bound_fine )
                    {
                        complementary_count++;
                    }
                    else
                    {
                        complementation_filter();
                        complementary_count = 1;
                    }
                    */
                }
            }
        }
        //hundred ms bottom

        //seconds top
        if(xitongshijian * 0.0001f > miaozhong + 1)
        {
            miaozhong = xitongshijian * 0.0001f; //visit by seconds resolution
            debug[7]++;
        }
        //seconds bottom


        /*
        //five seconds top
        if(xitongshijian * 0.00002f > wumiaozhong + 1)
        {
            wumiaozhong++;
        }
        //five seconds bottom
        */


    }
    //loop bottom
}
//main bottom

//I am Hououin Kyouma,a mad scientist,and the destroyer of this world's ruling structure.
//Failure is out of the question.

//Okay.I believe in you.

//little butterfly do not be afraid,the turbulence can save the world.
//OORGNO CYSPLE


