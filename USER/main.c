/*--------------------------------version information top-------------------------------------------
Started at 2016
Created by Karlxin(410824290@qq.com)
Github:https://github.com/Karlxin/OK001.git
OpenKarlCopter
Version:dev_000

Notice:dev version created without experiment.If we want a stable version,we could go to github and
checkout a test version or a stable version.


















--------------------------------version information top-------------------------------------------*/

#include "led.h"//��ͷ�ļ�
#include "delay.h"//�ӳ�ͷ�ļ�
#include "sys.h"//ϵͳͷ�ļ�
#include "usart.h"//����ͷ�ļ�
#include "stm32f10x.h"//stm32f10����ͷ�ļ�
#include "moto.h"//���Ƶ���õ�ͷ�ļ�
#include "timer.h"//��ʱ������ĵ�ͷ�ļ�,��ʱ������ϵͳ������ʱ�䣬��ʱ�����ǲ�׽������Ʋ��Ķ�ʱ��
#include "wdg.h"//���Ź�ͷ�ļ�
#include "pwm.h"//��ʱ��3�������pwm
#include "imu.h"//���Բ�����Ԫ
#include "mpu6050.h"//�����Ǽ��ٶȼ�
#include "inv_mpu.h"//mpu��Ӳ�������Լ��������ͷ�ļ�
#include "inv_mpu_dmp_motion_driver.h"//mpu��Ӳ�������Լ��������ͷ�ļ�
#include "math.h"//������ѧ������ͷ�ļ�
#include <stdio.h>//������ı�׼�������

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

float roll, pitch, yaw;//ŷ����,DMPӲ��õ��ĽǶ�,roll -180~180 pitch -90~90 yaw -180~180
short aacx, aacy, aacz;//���ٶȴ�����ԭʼ����
short gyrox, gyroy, gyroz;//������ԭʼ����
short temp;//�¶�
short gyrox_chushi, gyroy_chushi, gyroz_chushi; //�������ʼ���ڵ���ʱ���һЩ����,������ʼֵҪ��ȥ��
float temp_gyxc = 0, temp_gyyc = 0, temp_gyzc = 0; //����������������ݣ�������������ǳ�ʼֵ,����Ԥ��̬�����ֵƽ��Ȩ��ƽ��
short aacx_chushi, aacy_chushi, aacz_chushi; //���ٶȼ��ʼ���ڵ���ʱ���һЩ����,������ʼֵҪ��ȥ��
float temp_axc = 0, temp_ayc = 0, temp_azc = 0; //������ż��ٶȼ����ݣ�����������ٶȼƳ�ʼֵ,����Ԥ��̬�����ֵƽ��Ȩ��ƽ��

extern void filter_threeValue(void);//������ֵ�˲��ĺ���,����ֻ�˲������ǽ��ٶ�

float aacx_s, aacy_s, aacz_s, gyrox_sr, gyroy_sr, gyroz_sr; //��ԭʼ����ת��Ϊ��׼����,�Ի��ȼ�
float gyrox_sd, gyroy_sd, gyroz_sd; //��ԭʼ����ת��Ϊ��׼���ݣ��Զȼ�
float gyrox_sr_kf, gyroy_sr_kf, gyroz_sr_kf;//�������˲����ֵ

int16_t deadzone = 20; //ң������

int16_t cNd1 = 0, cNd2 = 0, cNd3 = 0, cNd4 = 0; //cyberNation ������
float rjz = 0, pjz = 0, yjz = 0; //��cNd1�����ݷֱ�ת��Ϊroll,pitch,yaw����ľ��������Ա�ʾ���۲�

extern void cN2rpy(void);

float x_last = 0; //for karlman
float p_last = 0;

u32 gyxt = 0; //gyroxitong,������¼�����ǳ�ʼֵ�õ���ʱ�����Ӽ�¼

u32 shihaomiao2 = 0; //��ŷǼ�ʱ���жϴ�������������ʮ����ֵ
u32 ershihaomiao2 = 0; //��ŷǼ�ʱ���жϴ������������Ķ�ʮ����ֵ
u32 wushihaomiao2 = 0;//��ŷǼ�ʱ���жϴ���������������ʮ����ֵ
u32 miaozhong2 = 0; //������ŷǼ�ʱ���жϴ�������������ϵͳ����ֵ

extern void MS561101BA_get_altitude(void);//��ø߶ȣ���ʵ�Ǽ�����߶�

float MS5611_Altitude;//using MS5611 pressure and temperature to calculate the Altitude.

extern void MS561101BA_getPressure(void);
extern void MS561101BA_GetTemperature(void);
float MS5611_Pressure;
int32_t  TEMP;//��ѹ���¶�
float Pressure_chushi;
int32_t TEMP_chushi;
float temp_Pressure = 0;
float temp_TEMP = 0;

float roll_err, pitch_err, yaw_err;//���ֵ,roll_err=roll-desroll,����desrollΪң���ź�����ӳ��ĽǶ�ֵ
float desroll, despitch, desyaw;//������Ҫ�ĺ����������ƫ��������

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

//-----------��ѹ�ƴ������õ����Ͻ�

int64_t OFF_;


/*
C1 ѹ�������� SENS|T1
C2  ѹ������  OFF|T1
C3  �¶�ѹ��������ϵ�� TCS
C4  �¶�ϵ����ѹ������ TCO
C5  �ο��¶� T|REF
C6  �¶�ϵ�����¶� TEMPSENS
*/
uint16_t  Cal_C[7];//���ڴ��PROM�е�6������1-6

uint32_t D1_Pres, D2_Temp;//����ѹ��ֵ,�����¶�ֵ

/*
dT ʵ�ʺͲο��¶�֮��Ĳ���
TEMP ʵ���¶�
*/
int32_t dT, TEMP;
/*
OFF ʵ���¶Ȳ���
SENS ʵ���¶�������
*/
int64_t OFF, SENS;

int32_t P;//��λ0.01mbar

uint32_t Pressure, Pressure_old, qqp;//����ѹ//��λ0.01mbar

int64_t T2, TEMP2;//�¶�У��ֵ
int64_t OFF2, SENS2;

uint32_t Pres_BUFFER[20];//������
uint32_t Temp_BUFFER[10];//������

//-----------��ѹ�ƴ������õ����½�

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

float Ahd = 0; //�������Ų���

extern void Sink_compensation(void);//���߸���
float Scd = 0; //�������Ų���

float debug[10];

//deviation between 0.1ms��little boy do not be afraid

int main(void)
{
    //------------------------------��ʼ���Ͻ�------------------------------
    u8 jiesuokeyi = 0;//����������Ա�־λ
    u16 baochijiesuo = 0;//���屣�ֽ���������
    u16 baochijiasuo = 0;//���屣�ּ���������
    int16_t temp1, temp2,  desthrottle, temp4; //������Ϊ�м����,��ң���ź�ת��ΪԤ�ڽǶ�
    u8 i;//for for loop
    u8 USART1_Open = 0;//Open Serial by 500000 baud rate by setting it.
    u8 USART2_Open = 0;//Open Serial by 115200 baud rate by setting it.

    SystemInit();//ϵͳ��ʼ��,over 0.02628ms
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//���ȵȼ���Ϊ0��1��2��3���ӵȼ���Ϊ0,1(2:0),over 0.0004ms
    delay_init();//�ӳٳ�ʼ��,over 0.00114s

    TIM3_PWM_Init(19999, 71);  //50Hz,over 0.02154ms

    TIM_SetCompare1(TIM3, 2000);//����ռ�ձ�,over 0.00047ms
    TIM_SetCompare2(TIM3, 2000);//����ռ�ձ�
    TIM_SetCompare3(TIM3, 2000);//����ռ�ձ�
    TIM_SetCompare4(TIM3, 2000);//����ռ�ձ�

    delay_ms(5000);//�ӳ�5s

    TIM_SetCompare1(TIM3, 1000);//����ռ�ձ�
    TIM_SetCompare2(TIM3, 1000);//����ռ�ձ�
    TIM_SetCompare3(TIM3, 1000);//����ռ�ձ�
    TIM_SetCompare4(TIM3, 1000);//����ռ�ձ�

    delay_ms(3000);//�ӳ�3000ms

    MPU_Init();//��ʼ��MPU6050,over 0.103s
    while(mpu_dmp_init())//���ٶȼƺ��������Ƿ��ʼ��
    {
        delay_ms(200);//�ӳ�200ms
        delay_ms(200);//�ӳ�200ms
    }//over 1.91s
    delay_ms(1000);//�ӳ�1000ms
    IIC_Init();//���ɵ�·���߳�ʼ��,over 0.00546ms
    delay_ms(100);//�ӳ����һ��
    MS561101BA_RESET();//��ѹ�����,over 0.35881ms
    delay_ms(100);//�ӳ����һ��
    MS5611_init();//��ѹ�Ƴ�ʼ��,over 0.0324s
    delay_ms(1000);//�ӳ�һ��

    TIM4_Cap_Init(0xffff, 72 - 1); //PWM�����ʼ��,��1Mhz��Ƶ�ʼ���,over 0.02819ms
    TIM5_Int_Init(9, 719); //ϵͳ��ʱ��ʼ,resolution of 0.1ms

    delay_ms(300);//delay 300 milliseconds


    //��ʼ�������ǣ�ʵ�������������ǰ��ʼ�����ڵ�������������Ϊ��λ��¼����
    gyxt = xitongshijian; //��¼������ʱ��ϵͳ����ֵ,over 0.00172ms
    while(xitongshijian * 0.001f < gyxt * 0.001f + 3) //�������뿪��һ������
    {
        if(!MPU_Get_Gyroscope(&gyrox, &gyroy, &gyroz))//��ȡ������ԭʼ����
        {
            temp_gyxc = ((float)gyrox  + temp_gyxc) * 0.5;//to get averaged initial data
            temp_gyyc = ((float)gyroy  + temp_gyyc) * 0.5;
            temp_gyzc = ((float)gyroz  + temp_gyzc) * 0.5;
        }

        if(!MPU_Get_Accelerometer(&aacx, &aacy, &aacz)) //�õ����ٶȴ���������,��ʱ0.6ms
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
        Uart1_Init(500000);//������4.06������ʱ��Ҫ�򿪵�����
    }
    else if(USART2_Open)
    {
        Uart1_Init(115200);//��ATKXCOMV2.0������ʱ��Ҫ�򿪵�ͨ���첽�շ����ڲ���������
    }

    //Calculate_FilteringCoefficient(0.0050000, 10.0000000);//����accz�ĵ�ͨ�˲�������

    LED_Init();//��ʼ��������,over 0.01044ms
    LED0 = 1; //Darkening red LED,showing ARMED
    LED1 = 0; //Lightening green LED��showing DISARMED

    erhaomiao = xitongshijian * 0.05f;//noticing that we disorder the schedule before
    wuhaomiao = xitongshijian * 0.02f;
    shihaomiao = xitongshijian * 0.01f;
    ershihaomiao = xitongshijian * 0.005f;
    wushihaomiao = xitongshijian * 0.002f;
    yibaihaomiao = xitongshijian * 0.001f;
    miaozhong = xitongshijian * 0.0001f;
    //------------------------------��ʼ���½�------------------------------

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

            if(!MPU_Get_Accelerometer(&aacx, &aacy, &aacz)) //�õ����ٶȴ���������,��ʱ0.6ms
            {
                Kalman_filter_accz();
                accz_dt = (float)(xitongshijian - accz_temp_time) * 0.001;//how much time between two visit
                accz_temp_time = xitongshijian;//record the time
                acc_Climb += (accz_X_hat_minus - aacz_chushi) * 0.0005978 * accz_dt;//accelerometer integral
                acc_Climb_out = acc_Climb - acc_Climb_err; //error adjust by using baro derivative.
                //Kalman_filter_accy();
                //Kalman_filter_accx();
            }

            if(!MPU_Get_Gyroscope(&gyrox, &gyroy, &gyroz))  //�õ�����������,��ʱ0.6ms
            {
                Gyro_filter();//���������˲�,��ʱ0.02ms
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
            //ag2q2rpy(gyrox_sr+0.0553938, gyroy_sr-0.0170442, gyroz_sr-0.0159790, aacx-960, aacy-350, aacz+1085, &pitch, &roll, &yaw);//�����ʱ0.25ms;
        }
        //ten ms bottom

        //twenty ms top
        if(xitongshijian * 0.005f > ershihaomiao + 1)
        {
            ershihaomiao = xitongshijian * 0.005f; //visiting by twenty milliseconds resolution
            //0.09*65536/4=1475(shortΪint16_t,65536��������2g)
            //desthrottle -= (int16_t)(0.03 * (float)(accz_out - aacz_chushi)); //��aacz���ڳ�ʼʱ��˵���ɻ�����,����Ӧ�ü�С,���̫�ֲ��ˣ��˵�����

            //-----------------------------����������ˢ���Ͻ�----------------------------------------------
            if(jiesuokeyi) //������Խ���,over 0.01204ms
            {
                if(channel3_in > 1100)//ͨ�������ܵ�����1100ռ�ձȵ�������Ʋ��ź�,over 0.01519ms
                {
                    desthrottle = channel3_in;//��Ҫ�����ŵ���ͨ�������յ����ź�ռ�ձ�,over 0.01208ms

                    if(channel1_in < 1507 - deadzone || channel1_in > 1507 + deadzone)//ͨ��һ�����źŲ��ڹ��������м�,over 0.01558ms
                    {
                        temp1 =  channel1_in - 1507 ; //��Ҫ�ĺ������ͨ��һ�м��
                        desroll = (float)temp1 * 0.0361446; //ת��Ϊ����15

                    }
                    else//ͨ��һ�����ź��ڹ��������м�
                    {
                        desroll = 0;//��Ҫ�ĺ��Ϊ��
                    }

                    if(channel2_in < 1508 - deadzone || channel2_in > 1508 + deadzone)//ͨ���������źŲ��ڹ��������м�
                    {
                        temp2 = 1508 - channel2_in;//��Ҫ�ĸ�������ͨ������λ��
                        despitch = (float)temp2 * 0.0361446; //ת��Ϊ����15

                    }
                    else//ͨ���������ź��ڹ��������м�
                    {
                        despitch = 0;//��Ҫ�ĸ���Ϊ��
                    }

                    if(channel4_in < 1507 - deadzone || channel4_in > 1507 + deadzone)//ͨ���Ľ����źŲ��ڹ��������м�
                    {
                        temp4 = 1507 - channel4_in;//��Ҫ��ƫ��Ϊͨ������λ��
                        desyaw = (float)temp4 * 0.0361446; //ת��Ϊ����15
                        desyaw = 0; //������ʱ����Ҫ�ֶ�����yaw

                    }
                    else//ͨ���Ľ����ź��ڹ��������м�
                    {
                        desyaw = 0;//��Ҫ��ƫ��������
                    }
                }
                else//ͨ�������ܵ�������1100ռ�ձȵ�������Ʋ��ź�
                {
                    //desthrottle = 1000;//��Ҫ�����ŵ�����С����,��Ϊ(1000,2000)�ǽ����ź�ֵ��Χ
                    desthrottle = 0;//����1000̫�ֲ��ˣ��������������ţ��ڷ�ɢ��ʱ��ȶ��Ȳ���
                    desroll = despitch = desyaw = 0;//��Ҫ�ĺ������ƫ����Ϊ��
                    Moto_PwmRflash(0, 0, 0, 0);//ȫ��������С��,�����������Ƶ�����ã����ĺ�ʱ.����һ����һ
                }

                if(channel3_in < 1100 && channel4_in < 1100)//����С��1100��ƫ��С��1100,Ҳ�����������£�ƫ������
                {
                    if(baochijiasuo == 0)//���ּ���Ϊ��
                    {
                        baochijiasuo = miaozhong;//���ּ�������ϵͳ��������������ֵ
                    }
                    if((miaozhong - baochijiasuo) >= 8)//������С��ƫ�������Ѿ���ȥ������
                    {
                        jiesuokeyi = 0;//ң�ز����Խ���
                        LED1 = 0;//�̵�����ʾ����������
                        LED0 = 1;//������ʾ�����õ������
                    }
                }
                else
                {
                    baochijiasuo = 0;//���ּ�����ʱ�����¹���
                }
            }
            else//����������
            {
                desthrottle = 0;//����1000̫�ֲ��ˣ��������������ţ��ڷ�ɢ��ʱ��ȶ��Ȳ���
                desroll = despitch = desyaw = 0;//��Ҫ�ĺ������ƫ����Ϊ��
                Moto_PwmRflash(0, 0, 0, 0);//ȫ��������С��,over 0.02ms,����һ���ö�

                if(channel3_in < 1100 && channel4_in > 1900)//����С��1100��ƫ���Ǵ���1900,Ҳ�����������£�ƫ������
                {
                    if(baochijiesuo == 0)//���ֽ�����ʱ��Ϊ��
                    {
                        baochijiesuo = miaozhong;//���ֽ�����ʱ��������ϵͳ����������ֵ
                    }
                    if((miaozhong - baochijiesuo) >= 5)//���뱣�ֽ���״̬�Ѿ���ȥ������
                    {
                        jiesuokeyi = 1;//�������Ա�־���¸�ֵΪ����
                        LED1 = 1; //�̵����ʾ������
                        LED0 = 0; //�������ʾҪС�ĵ������ת��
                    }
                }
                else//���Ų�С��1100����ƫ���ǲ�����1900,Ҳ�������Ų������»���ƫ����������
                {
                    baochijiesuo = 0;//���ֽ������¹���
                }
            }
            //-----------------------------����������ˢ���½�----------------------------------------------

            if(channel3_in > 1100)//ֻ�����Ŵ���1100ʱ������������ź��Զ�������
            {
                cyberNation();//���µ��,����50Hz���·�,over 0.06ms
                //Altitude_hold_update();//���ߵ�����
                Sink_compensation();//���Ų���������
                Moto_Throttle(desthrottle);//ֻ��������,���������������õײ�ֱ�ӿ��Ƶ���ĺ���.���Ķ�����һ
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
            //printf("  TEMP =%.2f��\r\n", (float)TEMP / 100.00);
            //printf("  MS5611_Pressure =%fmbar\r\n", MS5611_Pressure / 100);
            //printf("  MS5611_Altitude =%f\r\n", MS5611_Altitude-Altitude_chushi);
            //printf("  Altitude_X_hat =%f\r\n", Altitude_X_hat_minus-Altitude_chushi);

            //            temp=MPU_Get_Temperature();

            //            printf("  mpu temp=%.2f\r\n", temp*0.01f);//mpu�¶�
            //            printf("  Temp : %.2f ��\r\n", TEMP * 0.01f);            //�������ԭʼ���ݣ���ѹ�ƶ������¶�
            //            printf("  height : %.2f m\r\n", (float)MS561101BA_get_altitude());
            //            printf("  Pressure : %.2f mbar\r\n\r\n\r\n", Pressure * 0.01f); //�������ԭʼ����

            //            printf("  aacx=%f\r\n", aacx_s);//x����ٶ�,printf���������ATKCOM���ڲ�׽����PC����ʾ
            //            printf("  aacy=%f\r\n", aacy_s);
            //            printf("  aacz=%f\r\n\r\n", aacz_s);

            //            printf("  gyrox=%f\r\n", gyrox_sd);
            //            printf("  gyroy=%f\r\n", gyroy_sd);
            //            printf("  gyroz=%f\r\n\r\n", gyroz_sd);
            //
            //            printf("    roll=%f ��\r\n", roll); //�������̬����Ҫת��
            //            printf("  pitch=%f ��\r\n", pitch);
            //            printf("  yaw=%f ��\r\n\r\n", yaw);

            //            printf("  aacx=%d\r\n", aacx);//x����ٶ�
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



