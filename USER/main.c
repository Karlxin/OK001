/*--------------------------------version information top-------------------------------------------
Started at 2016
Created by Karlxin(410824290@qq.com)
Github:https://github.com/Karlxin/OK001.git
OpenKarlCopter
Version:dev_000

Notice:dev version created without experiment.If we want a stable version,we could go to github and
checkout a test version or a stable version.


















--------------------------------version information top-------------------------------------------*/

#include "led.h"//灯头文件
#include "delay.h"//延迟头文件
#include "sys.h"//系统头文件
#include "usart.h"//串口头文件
#include "stm32f10x.h"//stm32f10板子头文件
#include "moto.h"//控制电机用的头文件
#include "timer.h"//定时器五和四的头文件,定时器五是系统开机总时间，定时器四是捕捉脉宽调制波的定时器
#include "wdg.h"//看门狗头文件
#include "pwm.h"//定时器3用来输出pwm
#include "imu.h"//惯性测量单元
#include "mpu6050.h"//陀螺仪加速度计
#include "inv_mpu.h"//mpu的硬件配置以及读数相关头文件
#include "inv_mpu_dmp_motion_driver.h"//mpu的硬件配置以及读数相关头文件
#include "math.h"//核心数学函数库头文件
#include <stdio.h>//带缓冲的标准输入输出

extern void TIM3_PWM_Init(u16 arr, u16 psc);
extern void TIM4_Cap_Init(u16 arr, u16 psc);
extern void TIM5_Int_Init(u16 arr, u16 psc);
extern void Moto_Throttle(int16_t desthrottle);
extern void MS5611_init(void);
extern void IIC_Init(void);

u32 channel1_in, channel2_in, channel3_in, channel4_in;

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
u32 erhaomiao = 0;//two millisecond resolution
u32 wuhaomiao = 0;//five millisecond resolution
u32 shihaomiao = 0; //ten millisecond resolution
u32 ershihaomiao = 0; //twenty millisecond resolution
u32 wushihaomiao = 0;//fifty millisecond resolution
u32 yibaihaomiao = 0;//hundred millisecond resolution
u32 miaozhong = 0; //second resolution

float roll, pitch, yaw;//欧拉角,DMP硬解得到的角度,roll -180~180 pitch -90~90 yaw -180~180
short aacx, aacy, aacz;//加速度传感器原始数据
short gyrox, gyroy, gyroz;//陀螺仪原始数据
short temp;//温度
short gyrox_chushi, gyroy_chushi, gyroz_chushi; //陀螺仪最开始放在地面时候的一些数据,当作初始值要减去的
float temp_gyxc = 0, temp_gyyc = 0, temp_gyzc = 0; //用来存放陀螺仪数据，并计算出陀螺仪初始值,采用预估态与测量值平均权重平均
short aacx_chushi, aacy_chushi, aacz_chushi; //加速度计最开始放在地面时候的一些数据,当作初始值要减去的
float temp_axc = 0, temp_ayc = 0, temp_azc = 0; //用来存放加速度计数据，并计算出加速度计初始值,采用预估态与测量值平均权重平均

extern void filter_threeValue(void);//用来三值滤波的函数,现在只滤波陀螺仪角速度

float aacx_s, aacy_s, aacz_s, gyrox_sr, gyroy_sr, gyroz_sr; //将原始数据转换为标准数据,以弧度计
float gyrox_sd, gyroy_sd, gyroz_sd; //将原始数据转换为表准数据，以度计
float gyrox_sr_kf, gyroy_sr_kf, gyroz_sr_kf;//卡尔曼滤波后的值

int16_t deadzone = 20; //遥控死区

int16_t cNd1 = 0, cNd2 = 0, cNd3 = 0, cNd4 = 0; //cyberNation 纠正量
float rjz = 0, pjz = 0, yjz = 0; //将cNd1等数据分别转换为roll,pitch,yaw方向的纠正量，以便示波观察

extern void cN2rpy(void);

float x_last = 0; //for karlman
float p_last = 0;

u32 gyxt = 0; //gyroxitong,用来记录陀螺仪初始值用到的时间秒钟记录

u32 shihaomiao2 = 0; //存放非计时器中断处理函数内自增的十毫秒值
u32 ershihaomiao2 = 0; //存放非计时器中断处理函数内自增的二十毫秒值
u32 wushihaomiao2 = 0;//存放非计时器中断处理函数内自增的五十毫秒值
u32 miaozhong2 = 0; //用来存放非计时器中断处理函数内自增的系统秒钟值

extern void MS561101BA_get_altitude(void);//获得高度，其实是计算出高度

float MS5611_Altitude;//using MS5611 pressure and temperature to calculate the Altitude.

extern void MS561101BA_getPressure(void);
extern void MS561101BA_GetTemperature(void);
float MS5611_Pressure;
int32_t  TEMP;//气压计温度
float Pressure_chushi;
int32_t TEMP_chushi;
float temp_Pressure = 0;
float temp_TEMP = 0;

float roll_err, pitch_err, yaw_err;//误差值,roll_err=roll-desroll,其中desroll为遥控信号线性映射的角度值
float desroll, despitch, desyaw;//定义想要的横滚，俯仰，偏航，油门

short gyrox_out, gyroy_out, gyroz_out;
short accz_out;

extern void Gyro_filter(void);
extern void ANO_DT_Send_RCData(u16 thr, u16 yaw, u16 rol, u16 pit, u16 aux1, u16 aux2, u16 aux3, u16 aux4, u16 aux5, u16 aux6);
extern void Accz_filter(void);

float ACC_IIR_FACTOR;
extern void Calculate_FilteringCoefficient(float Time, float Cut_Off);
extern void ACC_IIR_Filter(void);
extern int16_t Throttle_constrain(int16_t Throttle);

extern void Altitude_hold_update(void);

//-----------气压计传感器用到的上界

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

//-----------气压计传感器用到的下界

float scaling;
float temp_jisuan;

float temp_Altitude = 0;
float Altitude_chushi;


//---Alt karlman top
extern void Kalman_filter_alt(void);
float Altitude_minus = 0;
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

float Ahd = 0; //定高油门补偿

extern void Sink_compensation(void);//掉高更新
float Scd = 0; //掉高油门补偿

float debug[10];

//deviation between 0.1ms，little boy do not be afraid

int main(void)
{
    //------------------------------初始化上界------------------------------
    u8 jiesuokeyi = 0;//定义解锁可以标志位
    u16 baochijiesuo = 0;//定义保持解锁计数器
    u16 baochijiasuo = 0;//定义保持加锁计数器
    int16_t temp1, temp2,  desthrottle, temp4; //用来作为中间变量,将遥控信号转换为预期角度
    u8 i;//for for loop
    u8 USART1_Open = 0;//Open Serial by 500000 baud rate by setting it.
    u8 USART2_Open = 0;//Open Serial by 115200 baud rate by setting it.

    SystemInit();//系统初始化,over 0.02628ms
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//抢先等级分为0，1，2，3；子等级分为0,1(2:0),over 0.0004ms
    delay_init();//延迟初始化,over 0.00114s

    TIM3_PWM_Init(19999, 71);  //50Hz,over 0.02154ms

    TIM_SetCompare1(TIM3, 2000);//设置占空比,over 0.00047ms
    TIM_SetCompare2(TIM3, 2000);//设置占空比
    TIM_SetCompare3(TIM3, 2000);//设置占空比
    TIM_SetCompare4(TIM3, 2000);//设置占空比

    delay_ms(5000);//延迟5s

    TIM_SetCompare1(TIM3, 1000);//设置占空比
    TIM_SetCompare2(TIM3, 1000);//设置占空比
    TIM_SetCompare3(TIM3, 1000);//设置占空比
    TIM_SetCompare4(TIM3, 1000);//设置占空比

    delay_ms(3000);//延迟3000ms

    MPU_Init();//初始化MPU6050,over 0.103s
    while(mpu_dmp_init())//加速度计和陀螺仪是否初始化
    {
        delay_ms(200);//延迟200ms
        delay_ms(200);//延迟200ms
    }//over 1.91s
    delay_ms(1000);//延迟1000ms
    IIC_Init();//集成电路总线初始化,over 0.00546ms
    delay_ms(100);//延迟零点一秒
    MS561101BA_RESET();//气压计清除,over 0.35881ms
    delay_ms(100);//延迟零点一秒
    MS5611_init();//气压计初始化,over 0.0324s
    delay_ms(1000);//延迟一秒

    TIM4_Cap_Init(0xffff, 72 - 1); //PWM捕获初始化,以1Mhz的频率计数,over 0.02819ms
    TIM5_Int_Init(9, 719); //系统计时开始,resolution of 0.1ms

    delay_ms(300);//delay 300 milliseconds


    //初始化陀螺仪，实质上做的事情是把最开始三秒内的陀螺仪数据作为零位记录下来
    gyxt = xitongshijian; //记录程序到这时的系统毫秒值,over 0.00172ms
    while(xitongshijian * 0.001f < gyxt * 0.001f + 3) //当程序离开上一句运行
    {
        if(!MPU_Get_Gyroscope(&gyrox, &gyroy, &gyroz))//读取陀螺仪原始数据
        {
            temp_gyxc = ((float)gyrox  + temp_gyxc) * 0.5;//to get averaged initial data
            temp_gyyc = ((float)gyroy  + temp_gyyc) * 0.5;
            temp_gyzc = ((float)gyroz  + temp_gyzc) * 0.5;
        }

        if(!MPU_Get_Accelerometer(&aacx, &aacy, &aacz)) //得到加速度传感器数据,耗时0.6ms
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
        Uart1_Init(500000);//给匿名4.06读数据时需要打开的速率
    }
    else if(USART2_Open)
    {
        Uart1_Init(115200);//给ATKXCOMV2.0读数据时需要打开的通用异步收发串口波特率速率
    }

    //Calculate_FilteringCoefficient(0.0050000, 10.0000000);//计算accz的低通滤波器增益

    LED_Init();//初始化结束咯,over 0.01044ms
    LED0 = 1; //Darkening red LED,showing ARMED
    LED1 = 0; //Lightening green LED，showing DISARMED

    erhaomiao = xitongshijian * 0.05f;//noticing that we disorder the schedule before
    wuhaomiao = xitongshijian * 0.02f;
    shihaomiao = xitongshijian * 0.01f;
    ershihaomiao = xitongshijian * 0.005f;
    wushihaomiao = xitongshijian * 0.002f;
    yibaihaomiao = xitongshijian * 0.001f;
    miaozhong = xitongshijian * 0.0001f;
    //------------------------------初始化下界------------------------------

    while(1)//using 12s to get there
    {
        //two ms top
        if(xitongshijian * 0.05f > erhaomiao + 1)
        {
            erhaomiao = xitongshijian * 0.05f; //visiting by two milliseconds resolution
        }
        //two ms bottom

        //five ms top
        if(xitongshijian * 0.02f > wuhaomiao + 1)
        {
            wuhaomiao = xitongshijian * 0.02f; //visiting by five milliseconds resolution

            if(!MPU_Get_Accelerometer(&aacx, &aacy, &aacz)) //得到加速度传感器数据,耗时0.6ms
            {
                Kalman_filter_accz();
                accz_dt = (float)(xitongshijian - accz_temp_time) * 0.001;//how much time between two visit
                accz_temp_time = xitongshijian;//record the time
                acc_Climb += (accz_X_hat_minus - aacz_chushi) * 0.0005978 * accz_dt;//accelerometer integral
                acc_Climb_out = acc_Climb - acc_Climb_err; //error adjust by using baro derivative.
                //Kalman_filter_accy();
                //Kalman_filter_accx();
            }

            if(!MPU_Get_Gyroscope(&gyrox, &gyroy, &gyroz))  //得到陀螺仪数据,耗时0.6ms
            {
                Gyro_filter();//滑动窗口滤波,耗时0.02ms
            }


        }
        //five ms bottom

        //ten ms top
        if(xitongshijian * 0.01f > shihaomiao + 1)
        {
            shihaomiao = xitongshijian * 0.01f; //visiting by ten milliseconds resolution
            mpu_dmp_get_data(&pitch, &roll, &yaw);//over amazing 52ms,move 50ms delay and we got 2.1ms,use dmp hardware
            roll_err = roll - desroll; //get roll_err
            pitch_err = pitch - despitch;
            yaw_err = yaw - desyaw;
            //ag2q2rpy(gyrox_sr+0.0553938, gyroy_sr-0.0170442, gyroz_sr-0.0159790, aacx-960, aacy-350, aacz+1085, &pitch, &roll, &yaw);//计算耗时0.25ms;
        }
        //ten ms bottom

        //twenty ms top
        if(xitongshijian * 0.005f > ershihaomiao + 1)
        {
            ershihaomiao = xitongshijian * 0.005f; //visiting by twenty milliseconds resolution
            //0.09*65536/4=1475(short为int16_t,65536代表正负2g)
            //desthrottle -= (int16_t)(0.03 * (float)(accz_out - aacz_chushi)); //当aacz大于初始时，说明飞机向上,油门应该减小,这个太恐怖了，伤到我了

            //-----------------------------控制与油门刷新上界----------------------------------------------
            if(jiesuokeyi) //如果可以解锁,over 0.01204ms
            {
                if(channel3_in > 1100)//通道三接受到超过1100占空比的脉宽调制波信号,over 0.01519ms
                {
                    desthrottle = channel3_in;//想要的油门等于通道三接收到的信号占空比,over 0.01208ms

                    if(channel1_in < 1507 - deadzone || channel1_in > 1507 + deadzone)//通道一接收信号不在工程意义中间,over 0.01558ms
                    {
                        temp1 =  channel1_in - 1507 ; //想要的横滚等于通道一中间差
                        desroll = (float)temp1 * 0.0361446; //转换为正负15

                    }
                    else//通道一接收信号在工程意义中间
                    {
                        desroll = 0;//想要的横滚为零
                    }

                    if(channel2_in < 1508 - deadzone || channel2_in > 1508 + deadzone)//通道二接收信号不在工程意义中间
                    {
                        temp2 = 1508 - channel2_in;//想要的俯仰等于通道二中位差
                        despitch = (float)temp2 * 0.0361446; //转换为正负15

                    }
                    else//通道二接收信号在工程意义中间
                    {
                        despitch = 0;//想要的俯仰为零
                    }

                    if(channel4_in < 1507 - deadzone || channel4_in > 1507 + deadzone)//通道四接收信号不在工程意义中间
                    {
                        temp4 = 1507 - channel4_in;//想要的偏航为通道四中位差
                        desyaw = (float)temp4 * 0.0361446; //转换为正负15
                        desyaw = 0; //我们暂时不想要手动控制yaw

                    }
                    else//通道四接收信号在工程意义中间
                    {
                        desyaw = 0;//想要的偏航等于零
                    }
                }
                else//通道三接受到不超过1100占空比的脉宽调制波信号
                {
                    //desthrottle = 1000;//想要的油门等于最小油门,因为(1000,2000)是接受信号值范围
                    desthrottle = 0;//等于1000太恐怖了，根本降不下油门，在发散的时候救都救不回
                    desroll = despitch = desyaw = 0;//想要的横滚俯仰偏航均为零
                    Moto_PwmRflash(0, 0, 0, 0);//全部油门最小化,这个函数不会频繁调用，别担心耗时.核心一调用一
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
                Moto_PwmRflash(0, 0, 0, 0);//全部油门最小化,over 0.02ms,核心一调用二

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
            //-----------------------------控制与油门刷新下界----------------------------------------------

            if(channel3_in > 1100)//只有油门大于1100时才允许更新油门和自动控制量
            {
                cyberNation();//更新电机,经典50Hz更新法,over 0.06ms
                //Altitude_hold_update();//定高叠加量
                Sink_compensation();//油门补偿叠加量
                Moto_Throttle(desthrottle);//只控制油门,但是这个函数会调用底层直接控制电机的函数.核心二调用一
            }
        }
        //twenty ms bottom

        //fifty ms top
        if(xitongshijian * 0.002f > wushihaomiao + 1)
        {
            wushihaomiao = xitongshijian * 0.002f; //visiting by fifty milliseconds resolution
            //debug[0]=cosf(roll*0.0174533);

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



