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
#include "inv_mpu.h"//mpu的硬件配置以及读数相关
#include "inv_mpu_dmp_motion_driver.h"//mpu的硬件配置以及读数相关

#include <stdio.h>//带缓冲的标准输入输出

extern void TIM3_PWM_Init(u16 arr, u16 psc);
extern void TIM4_Cap_Init(u16 arr, u16 psc);
extern void TIM5_Int_Init(u16 arr, u16 psc);
extern void Moto_RPY(int16_t roll, int16_t pitch, int16_t throttle, int16_t yaw);
extern void MS5611_init(void);
extern void IIC_Init(void);

extern int16_t channel1_in, channel2_in, channel3_in, channel4_in;

extern void MS561101BA_RESET(void);

extern void ANO_DT_Send_Senser(s16 a_x, s16 a_y, s16 a_z, s16 g_x, s16 g_y, s16 g_z, s16 m_x, s16 m_y, s16 m_z, s32 bar);
extern void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, s32 alt, u8 fly_model, u8 armed);
extern void ag2q2rpy(float gx, float gy, float gz, float ax, float ay, float az, float *pitch, float *roll, float *yaw);
extern float KalmanFilter(const float ResrcData, float ProcessNiose_Q, float MeasureNoise_R, float InitialPrediction);

extern u8 MPU_Init(void);
extern u8 mpu_dmp_init(void);

extern u8 ARMED;

u32 xitongshijian = 0; //系统运行时间,以毫秒计，最多计1193+小时
u32 erhaomiao = 0;//二毫秒
u32 wuhaomiao = 0;//五毫秒
u32 shihaomiao = 0; //系统运行时间，以十毫秒计,最多计119304+小时
u32 ershihaomiao = 0; //系统运行时间,以二十毫秒,最多计59652+小时
u32 wushihaomiao = 0;
u32 miaozhong = 0; //系统运行时间，以秒计,最多计1193000+小时

float roll, pitch, yaw;         //欧拉角,DMP硬解得到的角度,roll -180~180 pitch -90~90 yaw -180~180
short aacx, aacy, aacz;     //加速度传感器原始数据
short gyrox, gyroy, gyroz;  //陀螺仪原始数据
short temp;                 //温度
short gyrox_chushi, gyroy_chushi, gyroz_chushi; //陀螺仪最开始放在地面时候的一些数据,当作初始值要减去的
float temp_gyxc = 0, temp_gyyc = 0, temp_gyzc = 0; //用来存放陀螺仪数据，并计算出陀螺仪初始值,采用预估态与测量值平均权重平均
float gyrox_filter[7]= {0},gyroy_filter[7]= {0},gyroz_filter[7]= {0}; //用来存放陀螺仪角速度的三值滤波器暂存数据,第四位用来存放陀螺仪三值滤波的返回值，即经过滤波后的值
short gyro_jishu=0;//滤波计数,一般到3回0,即永远不能到3

extern void filter_threeValue(void);//用来三值滤波的函数,现在只滤波陀螺仪角速度

float aacx_s, aacy_s, aacz_s, gyrox_sr, gyroy_sr, gyroz_sr; //将原始数据转换为标准数据,以弧度计
float gyrox_sd, gyroy_sd, gyroz_sd; //将原始数据转换为表准数据，以度计
float gyrox_sr_kf, gyroy_sr_kf, gyroz_sr_kf;//卡尔曼滤波后的值

int16_t deadzone = 20; //遥控死区

int16_t cNd1 = 0, cNd2 = 0, cNd3 = 0, cNd4 = 0; //cyberNation 纠正量
float rjz=0,pjz=0,yjz=0;//将cNd1等数据分别转换为roll,pitch,yaw方向的纠正量，以便示波观察

extern void cN2rpy(void);

float x_last = 0; //for karlman
float p_last = 0;

u32 gyxt = 0; //用来初始化陀螺仪的时间记录,gyroxitong

u32 shihaomiao2 = 0; //存放非计时器中断处理函数内自增的十毫秒值
u32 ershihaomiao2 = 0; //存放非计时器中断处理函数内自增的二十毫秒值
u32 wushihaomiao2 = 0;//存放非计时器中断处理函数内自增的五十毫秒值
u32 miaozhong2 = 0; //用来存放非计时器中断处理函数内自增的系统秒钟值

extern int32_t  TEMP;                   //气压计温度
extern float MS561101BA_get_altitude(void);//获得高度，其实是计算出高度
extern uint32_t Pressure;               //大气压//单位0.01mbar
extern int32_t  TEMP;                   //气压计温度

extern void MS561101BA_getPressure(void);
extern void MS561101BA_GetTemperature(void);

u8 roll_in_flag=0;//绕X轴有有效遥控信号输入则为1，否则为0
u8 pitch_in_flag=0;
u8 yaw_in_flag=0;

int main(void)
{
    //------------------------------初始化上界------------------------------

    u8 jiesuokeyi = 0;//定义解锁可以标志位
    u16 baochijiesuo = 0;//定义保持解锁计数器
    u16 baochijiasuo = 0;//定义保持加锁计数器
    int16_t desroll, despitch, desyaw, desthrottle;//定义想要的横滚，俯仰，偏航，油门
    float temp1, temp2, temp3; //用来作为中间变量,以及将油门值不至于那么大

    SystemInit();//系统初始化
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//抢先等级分为0，1，2，3；子等级分为0,1(2:0)
    delay_init();//延迟初始化

    //Uart1_Init(115200);//给ATKXCOMV2.0读数据时需要打开的通用异步首发串口波特率速率
    //Uart1_Init(500000);//给匿名4.06读数据时需要打开的速率

    TIM3_PWM_Init(19999, 71);  //50Hz

    TIM_SetCompare1(TIM3, 2000);        //设置占空比
    TIM_SetCompare2(TIM3, 2000);        //设置占空比
    TIM_SetCompare3(TIM3, 2000);        //设置占空比
    TIM_SetCompare4(TIM3, 2000);        //设置占空比

    delay_ms(5000);//延迟5s

    TIM_SetCompare1(TIM3, 1000);        //设置占空比
    TIM_SetCompare2(TIM3, 1000);        //设置占空比
    TIM_SetCompare3(TIM3, 1000);        //设置占空比
    TIM_SetCompare4(TIM3, 1000);        //设置占空比

    delay_ms(3000);//延迟3000ms

    MPU_Init();                 //初始化MPU6050
    while(mpu_dmp_init())//加速度计和陀螺仪是否初始化
    {
        delay_ms(200);//延迟200ms
        delay_ms(200);//延迟200ms
    }
    delay_ms(1000);//延迟1000ms
    IIC_Init();//集成电路总线初始化
    delay_ms(100);//延迟零点一秒
    MS561101BA_RESET();//气压计清除
    delay_ms(100);//延迟零点一秒
    MS5611_init();//气压计初始化
    delay_ms(1000);//延迟一秒

    TIM4_Cap_Init(0xffff, 72 - 1); //PWM捕获初始化,以1Mhz的频率计数
    TIM5_Int_Init(9, 7199); //系统计时开始1ms溢出版本
    //TIM5_Int_Init(9, 3599); //系统计时开始0.5ms溢出版本

    delay_ms(300);//延迟个三百吧，也不知道可不可以消除这句


    //初始化陀螺仪，实质上做的事情是把最开始三秒内的陀螺仪数据作为零位记录下来

    gyxt = xitongshijian; //记录程序到这时的秒钟值
    while(xitongshijian * 0.001f < gyxt * 0.001f + 3) //当程序离开上一句运行三秒内
    {
        if(!MPU_Get_Gyroscope(&gyrox, &gyroy, &gyroz))
        {
            temp_gyxc = (float)gyrox * 0.5 + temp_gyxc * 0.5;
            temp_gyyc = (float)gyroy * 0.5 + temp_gyyc * 0.5;
            temp_gyzc = (float)gyroz * 0.5 + temp_gyzc * 0.5;
        }
        //ANO_DT_Send_Senser(0, 0, 0, gyrox-gyrox_chushi, gyroy-gyroy_chushi, gyroz-gyroz_chushi,0,0,0,0);
    }
    gyrox_chushi = (short)temp_gyxc;
    gyroy_chushi = (short)temp_gyyc;
    gyroz_chushi = (short)temp_gyzc;

    LED_Init();//初始化结束咯
    LED0 = 1; //灭掉红灯
    LED1 = 0; //绿灯亮着，表示解锁不可以

    //------------------------------初始化下界------------------------------


    while(1)
    {
        //-----------------------------控制与油门刷新上界----------------------------------------------

        if(jiesuokeyi) //如果可以解锁
        {
            if(channel3_in > 1100)//通道三接受到超过1100占空比的脉宽调制波信号
            {
                desthrottle = channel3_in;//想要的油门等于通道三接收到的信号占空比
                if(channel1_in < 1507 - deadzone || channel1_in > 1507 + deadzone)//通道一接收信号不在工程意义中间
                {
                    desroll = 1507 - channel1_in;//想要的横滚等于通道一中间差
                    roll_in_flag=1;//通道一有有效信号输入
                }
                else//通道一接收信号在工程意义中间
                {
                    desroll = 0;//想要的横滚为零
                    roll_in_flag=0;//通道一没有效信号输入
                }
                if(channel2_in < 1508 - deadzone || channel2_in > 1508 + deadzone)//通道二接收信号不在工程意义中间
                {
                    despitch = 1508 - channel2_in;//想要的俯仰等于通道二中位差
                    pitch_in_flag=1;//通道二有有效信号输入
                }
                else//通道二接收信号在工程意义中间
                {
                    despitch = 0;//想要的俯仰为零
                    pitch_in_flag=0;//通道二没有有效信号输入
                }
                if(channel4_in < 1507 - deadzone || channel4_in > 1507 + deadzone)//通道四接收信号不在工程意义中间
                {
                    desyaw = 1507 - channel4_in;//想要的偏航为通道四中位差
                    yaw_in_flag=1;//通道四有有效信号输入
                }
                else//通道四接收信号在工程意义中间
                {
                    desyaw = 0;//想要的偏航等于零
                    yaw_in_flag=0;//通道四没有有效信号输入
                }
            }
            else
            {
                //desthrottle = 1000;//想要的油门等于最小油门,因为(1000,2000)是接受信号值范围
                desthrottle = 0;//等于1000太恐怖了，根本降不下油门，在发散的时候救都救不回
                desroll = despitch = desyaw = 0;//想要的横滚俯仰偏航均为零
            }

            temp1 = (float)desroll * 0.200; //变为以前的五分之一
            temp2 = (float)despitch * 0.200;
            temp3 = (float)desyaw * 0.200;

            //下面这句话为尝试十三要做的事
            temp3 = 0;

            /*Moto_RPY(desroll, despitch, desthrottle, desyaw);//将想要的角度输入给角度到油门值函数*/
            Moto_RPY((int)temp1, (int)temp2, desthrottle, (int)temp3);//由于上面那句话遥控直接控制的量实在太大，所以改改

            //printf("temp1=%d\r\n temp2=%d\r\n temp3=%d\r\n\r\n",(int)temp1, (int)temp2,(int)temp3);//测试四通道输出的值
            //printf("temp1=%f\r\n temp2=%f\r\n temp3=%f\r\n\r\n",temp1,temp2,temp3);//测试四通道输出的值
            //printf("desroll=%d\r\n despitch=%d\r\n desyaw=%d\r\n\r\n",desroll,despitch,desyaw);//测试四通道输出的值

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
            Moto_PwmRflash(0, 0, 0, 0);//全部油门最小化

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
            else
            {
                baochijiesuo = 0;//保持解锁重新归零
            }
        }

        //-----------------------------控制与油门刷新下界----------------------------------------------


        //----------------------------上下位机交流上界-------------------------------------------------

        //ANO_DT_Send_Senser((s16)(aacx_s * 1000), (s16)(aacy_s * 1000), (s16)(aacz_s * 1000), (s16)(gyrox_sd * 10), (s16)(gyroy_sd * 10), (s16)(gyroz_sd * 10), (s16)0, (s16)0, (s16)0, (s32)0);
        //ANO_DT_Send_Status(roll, pitch, yaw, (s32)0, (u8)0, (u8)0);

        //ANO_DT_Send_Senser((s16)(aacx_s * 1000), (s16)(sliding_windows_filter(aacx_s * 1000)), (s16)(aacz_s * 1000), (s16)(gyrox_sd * 10), (s16)(gyroy_sd * 10), (s16)(gyroz_sd * 10), (s16)0, (s16)0, (s16)0, (s32)0);//测试滤波效果
        //printf("gyrox_chushi=%d\r\n gyroy_chushi=%d\r\n gyroz_chushi=%d\r\n\r\n",gyrox_chushi, gyroy_chushi,gyroz_chushi);//测试四通道输出的值

        //----------------------------上下位机交流下界-------------------------------------------------




        //----------------------------非中断函数内部的周期性执行程序上界-----------------------------------

        if(xitongshijian * 0.5f > erhaomiao + 1)
        {
            erhaomiao++;//每过二毫秒来这一次
        }

        if(xitongshijian * 0.2f > wuhaomiao + 1)
        {
            wuhaomiao++;//每过五毫秒来这一次
        }

        if(xitongshijian * 0.1f > shihaomiao + 1)
        {
            shihaomiao++;//每过十毫秒来这一次
            cyberNation();//更新电机
            mpu_dmp_get_data(&pitch, &roll, &yaw);//此句话消耗惊人的52ms,去掉50ms延迟后只需要2.1ms,这是mpu硬解姿态

            if(!MPU_Get_Accelerometer(&aacx, &aacy, &aacz)) //得到加速度传感器数据,耗时0.6ms
            {
                //aacx_s = (float)aacx * 0.0005978;
                //aacy_s = (float)aacy * 0.0005978;
                //aacz_s = (float)aacz * 0.0005978;
            }

            if(!MPU_Get_Gyroscope(&gyrox, &gyroy, &gyroz))  //得到陀螺仪数据,耗时0.6ms
            {

                //gyrox_sr = (float)gyrox * 0.0010653;
                //gyroy_sr = (float)gyroy * 0.0010653;
                //gyroz_sr = (float)gyroz * 0.0010653;

                //gyrox_sd = (float)gyrox * 0.0610352;
                //gyroy_sd = (float)gyroy * 0.0610352;
                //gyroz_sd = (float)gyroz * 0.0610352;
            }

            filter_threeValue();//三值滤波

            //ag2q2rpy(gyrox_sr+0.0553938, gyroy_sr-0.0170442, gyroz_sr-0.0159790, aacx-960, aacy-350, aacz+1085, &pitch, &roll, &yaw);//计算耗时0.25ms;
            //__nop();//上面三条总耗时1.4ms,每秒钟搞了101次
            //ANO_DT_Send_Senser(aacx,aacy,aacz,gyrox,gyroy,gyroz);
            //ANO_DT_Send_Senser((int8_t)roll*100,(int8_t)pitch*100,(int8_t)yaw,gyrox,gyroy,gyroz);
            //ANO_DT_Send_Status(roll, pitch, yaw, (s32)0, (u8)0, (u8)0);//耗时0.37ms
            //ANO_DT_Send_Senser(aacx, aacy, aacz, gyrox, gyroy, gyroz);
            //__nop();

            //cN2rpy();//示波查看纠正量,一般与角度为反值
            //ANO_DT_Send_Senser(aacx, aacy, aacz, gyrox-gyrox_chushi, gyroy-gyroy_chushi, gyroz-gyroz_chushi,(s32)rjz,(s32)pjz,(s32)yjz,0);
            //ANO_DT_Send_Senser((s16)gyrox_filter[6]-gyrox_chushi,(s16)gyroy_filter[6]-gyroy_chushi ,(s16)gyroz_filter[6]-gyroz_chushi , gyrox-gyrox_chushi, gyroy-gyroy_chushi, gyroz-gyroz_chushi,(s32)rjz,(s32)pjz,(s32)yjz,0);
            //ANO_DT_Send_Status(roll, pitch, yaw, (s32)0, (u8)0, (u8)0);


        }
        if(xitongshijian * 0.05f > ershihaomiao + 1)
        {
            ershihaomiao++;//每过二十毫秒来这一次
            //MS561101BA_GetTemperature();//获取温度,消耗20.6ms,去掉10ms延迟后还需要10ms,以及将10ms改为8ms后,只需要8ms
            //MS561101BA_getPressure();   //获取大气压,消耗20.6ms,去掉延迟后,去掉10ms延迟后还需要10ms,以及将10ms改为8ms后,只需要8ms

            //printf("电机在更新中");//测试在四通道捕捉的同时电机是否会更新

            //尝试17注释语句上端
            /*if(!MPU_Get_Accelerometer(&aacx, &aacy, &aacz))   //得到加速度传感器数据,耗时0.58ms
            {
                //aacx_s = (float)aacx * 0.0005978;
                //aacy_s = (float)aacy * 0.0005978;
                //aacz_s = (float)aacz * 0.0005978;
            }

            if(!MPU_Get_Gyroscope(&gyrox, &gyroy, &gyroz))  //得到陀螺仪数据,耗时0.58ms
            {

                //gyrox_sr = (float)gyrox * 0.0010653;
                //gyroy_sr = (float)gyroy * 0.0010653;
                //gyroz_sr = (float)gyroz * 0.0010653;

                //gyrox_sd = (float)gyrox * 0.0610352;
                //gyroy_sd = (float)gyroy * 0.0610352;
                //gyroz_sd = (float)gyroz * 0.0610352;
            }

            //ag2q2rpy(gyrox_sr+0.0553938, gyroy_sr-0.0170442, gyroz_sr-0.0159790, aacx-960, aacy-350, aacz+1085, &pitch, &roll, &yaw);//计算耗时0.25ms;
            //__nop();//上面三条总耗时1.4ms,每秒钟搞了101次
            //ANO_DT_Send_Senser(aacx,aacy,aacz,gyrox,gyroy,gyroz);
            //ANO_DT_Send_Senser((int8_t)roll*100,(int8_t)pitch*100,(int8_t)yaw,gyrox,gyroy,gyroz);
            //ANO_DT_Send_Status(roll, pitch, yaw, (s32)0, (u8)0, (u8)0);//耗时0.37ms
            //ANO_DT_Send_Senser(aacx, aacy, aacz, gyrox, gyroy, gyroz);
            //__nop();*/
            //尝试17注释语句下端
        }

        if(xitongshijian * 0.02f > wushihaomiao + 1)
        {
            wushihaomiao++;//五十毫秒
        }

        if(xitongshijian * 0.001f > miaozhong + 1)
        {
            miaozhong++;//每过一秒来这里一次
            //temp=MPU_Get_Temperature();

            //printf("  mpu temp=%.2f\r\n", temp*0.01f);//mpu温度
            //printf("  Temp : %.2f ℃\r\n", TEMP * 0.01f);            //串口输出原始数据，气压计读到的温度
            //printf("  height : %.2f m\r\n", (float)MS561101BA_get_altitude());
            //printf("  Pressure : %.2f mbar\r\n\r\n\r\n", Pressure * 0.01f); //串口输出原始数据




            //printf("  aacx=%f\r\n", aacx_s);//x轴加速度,printf可以输出给ATKCOM串口捕捉并在PC上显示
            //printf("  aacy=%f\r\n", aacy_s);
            //printf("  aacz=%f\r\n\r\n", aacz_s);

            //printf("  gyrox=%f\r\n", gyrox_sd);
            //printf("  gyroy=%f\r\n", gyroy_sd);
            //printf("  gyroz=%f\r\n\r\n", gyroz_sd);






//          printf("    roll=%f 度\r\n", roll); //用软解姿态不需要转换
//            printf("  pitch=%f 度\r\n", pitch);
//            printf("  yaw=%f 度\r\n\r\n", yaw);
//
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

        //----------------------------非中断函数内部的周期性执行程序下界-----------------------------------

    }
}



