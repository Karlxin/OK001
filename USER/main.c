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
#include "inv_mpu.h"//mpu��Ӳ�������Լ��������
#include "inv_mpu_dmp_motion_driver.h"//mpu��Ӳ�������Լ��������

#include <stdio.h>//������ı�׼�������

extern void TIM3_PWM_Init(u16 arr, u16 psc);
extern void TIM4_Cap_Init(u16 arr, u16 psc);
extern void TIM5_Int_Init(u16 arr, u16 psc);
extern void Moto_Throttle(int16_t desthrottle);
extern void MS5611_init(void);
extern void IIC_Init(void);

extern int16_t channel1_in, channel2_in, channel3_in, channel4_in;

extern void MS561101BA_RESET(void);

extern void ANO_DT_Send_MotoPWM(u16 m_1, u16 m_2, u16 m_3, u16 m_4, u16 m_5, u16 m_6, u16 m_7, u16 m_8);
extern void ANO_DT_Send_Senser(s16 a_x, s16 a_y, s16 a_z, s16 g_x, s16 g_y, s16 g_z, s16 m_x, s16 m_y, s16 m_z, s32 bar);
extern void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, s32 alt, u8 fly_model, u8 armed);
extern void ag2q2rpy(float gx, float gy, float gz, float ax, float ay, float az, float *pitch, float *roll, float *yaw);
extern float KalmanFilter(const float ResrcData, float ProcessNiose_Q, float MeasureNoise_R, float InitialPrediction);

extern u8 MPU_Init(void);
extern u8 mpu_dmp_init(void);

extern u8 ARMED;

u32 xitongshijian = 0; //ϵͳ����ʱ��,�Ժ���ƣ�����1193+Сʱ
u32 erhaomiao = 0;//������
u32 wuhaomiao = 0;//�����
u32 shihaomiao = 0; //ϵͳ����ʱ�䣬��ʮ�����,����119304+Сʱ
u32 ershihaomiao = 0; //ϵͳ����ʱ��,�Զ�ʮ����,����59652+Сʱ
u32 wushihaomiao = 0;
u32 yibaihaomiao = 0;
u32 miaozhong = 0; //ϵͳ����ʱ�䣬�����,����1193000+Сʱ

float roll, pitch, yaw;         //ŷ����,DMPӲ��õ��ĽǶ�,roll -180~180 pitch -90~90 yaw -180~180
short aacx, aacy, aacz;     //���ٶȴ�����ԭʼ����
short gyrox, gyroy, gyroz;  //������ԭʼ����
short temp;                 //�¶�
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
int32_t  TEMP;                   //��ѹ���¶�
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
uint16_t  Cal_C[7];         //���ڴ��PROM�е�6������1-6

uint32_t D1_Pres, D2_Temp;  // ����ѹ��ֵ,�����¶�ֵ

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

uint32_t Pressure, Pressure_old, qqp;               //����ѹ//��λ0.01mbar

int64_t T2, TEMP2;  //�¶�У��ֵ
int64_t OFF2, SENS2;

uint32_t Pres_BUFFER[20];     //������
uint32_t Temp_BUFFER[10];     //������

//-----------��ѹ�ƴ������õ����½�

float scaling;
float temp_jisuan;

float temp_Altitude = 0;
float Altitude_chushi;


//---Alt karlman top
extern void Kalman_filter_alt(void);
float Altitude_minus=0;
float Altitude_dt = 0.1; //the delta time
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

//---acc_Climb top
extern void acc_Climb_update(void);
float acc_Climb_dt = 0.01; //the delta time
u32 acc_Climb_temp_time = 0; //the record time
float acc_Climb_R = 23; //3sigma 23*3=69;
float acc_Climb_Q = 0.0004; //process Variance
float acc_Climb_K = 0; //kalman gain
float acc_Climb_X_hat = 15300; //init predict for accz
float acc_Climb_X_hat_minus = 0; //previous predict for accz
float acc_Climb_P = 140; //error variance
float acc_Climb = 0; //the climb rate

float acc_Climb_err=0;
float acc_Climb_out=0;
//---acc_Climb bottom

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

float Ahd=0;

//�����1ms�ڣ�����С���ӱ���Ŷ

int main(void)
{
    //------------------------------��ʼ���Ͻ�------------------------------

    u8 jiesuokeyi = 0;//����������Ա�־λ
    u16 baochijiesuo = 0;//���屣�ֽ���������
    u16 baochijiasuo = 0;//���屣�ּ���������
    int16_t temp1, temp2,  desthrottle, temp4; //������Ϊ�м����,��ң���ź�ת��ΪԤ�ڽǶ�
    u8 i;//forѭ���õ�

    SystemInit();//ϵͳ��ʼ��
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//���ȵȼ���Ϊ0��1��2��3���ӵȼ���Ϊ0,1(2:0)
    delay_init();//�ӳٳ�ʼ��

    TIM3_PWM_Init(19999, 71);  //50Hz

    TIM_SetCompare1(TIM3, 2000);        //����ռ�ձ�
    TIM_SetCompare2(TIM3, 2000);        //����ռ�ձ�
    TIM_SetCompare3(TIM3, 2000);        //����ռ�ձ�
    TIM_SetCompare4(TIM3, 2000);        //����ռ�ձ�

    delay_ms(5000);//�ӳ�5s

    TIM_SetCompare1(TIM3, 1000);        //����ռ�ձ�
    TIM_SetCompare2(TIM3, 1000);        //����ռ�ձ�
    TIM_SetCompare3(TIM3, 1000);        //����ռ�ձ�
    TIM_SetCompare4(TIM3, 1000);        //����ռ�ձ�

    delay_ms(3000);//�ӳ�3000ms

    MPU_Init();                 //��ʼ��MPU6050
    while(mpu_dmp_init())//���ٶȼƺ��������Ƿ��ʼ��
    {
        delay_ms(200);//�ӳ�200ms
        delay_ms(200);//�ӳ�200ms
    }
    delay_ms(1000);//�ӳ�1000ms
    IIC_Init();//���ɵ�·���߳�ʼ��
    delay_ms(100);//�ӳ����һ��
    MS561101BA_RESET();//��ѹ�����
    delay_ms(100);//�ӳ����һ��
    MS5611_init();//��ѹ�Ƴ�ʼ��
    delay_ms(1000);//�ӳ�һ��

    TIM4_Cap_Init(0xffff, 72 - 1); //PWM�����ʼ��,��1Mhz��Ƶ�ʼ���
    TIM5_Int_Init(9, 7199); //ϵͳ��ʱ��ʼ1ms����汾
    //TIM5_Int_Init(9, 3599); //ϵͳ��ʱ��ʼ0.5ms����汾

    delay_ms(300);//�ӳٸ����ٰɣ�Ҳ��֪���ɲ������������


    //��ʼ�������ǣ�ʵ�������������ǰ��ʼ�����ڵ�������������Ϊ��λ��¼����

    gyxt = xitongshijian; //��¼������ʱ��ϵͳ����ֵ
    while(xitongshijian * 0.001f < gyxt * 0.001f + 3) //�������뿪��һ������
    {
        if(!MPU_Get_Gyroscope(&gyrox, &gyroy, &gyroz))//��ȡ������ԭʼ����
        {
            temp_gyxc = ((float)gyrox  + temp_gyxc) * 0.5;
            temp_gyyc = ((float)gyroy  + temp_gyyc) * 0.5;
            temp_gyzc = ((float)gyroz  + temp_gyzc) * 0.5;
        }

        if(!MPU_Get_Accelerometer(&aacx, &aacy, &aacz)) //�õ����ٶȴ���������,��ʱ0.6ms
        {
            temp_axc = ((float)aacx + temp_axc) * 0.5;
            temp_ayc = ((float)aacy + temp_ayc) * 0.5;
            temp_azc = ((float)aacz + temp_azc) * 0.5;
            acc_Climb_update();
        }
    }
    gyrox_chushi = (short)temp_gyxc;
    gyroy_chushi = (short)temp_gyyc;
    gyroz_chushi = (short)temp_gyzc;

    aacx_chushi = (short)temp_axc;
    aacy_chushi = (short)temp_ayc;
    aacz_chushi = (short)acc_Climb_X_hat;

    for(i = 0; i < 20; i++)
    {
        delay_ms(100);

        MS561101BA_GetTemperature();//over 9.7ms
        temp_TEMP = ((float)TEMP + temp_TEMP) * 0.5;
    }
	
    TEMP_chushi = (int32_t)temp_TEMP;
    temp_jisuan    = (float)TEMP_chushi / 100.00 + 273.15f;
	
    for(i = 0; i < 20; i++) //
    {
        delay_ms(100);

        MS561101BA_getPressure();//over 9.7ms
        Kalman_filter_pressure();


    }

    Pressure_chushi = pressure_X_hat_minus;

    scaling = pressure_X_hat_minus / Pressure_chushi;
    MS561101BA_get_altitude();//0.02ms
    Altitude_chushi = MS5611_Altitude;

    //Uart1_Init(115200);//��ATKXCOMV2.0������ʱ��Ҫ�򿪵�ͨ���첽�շ����ڲ���������
    Uart1_Init(500000);//������4.06������ʱ��Ҫ�򿪵�����

    //Calculate_FilteringCoefficient(0.0050000, 10.0000000);//����accz�ĵ�ͨ�˲�������

    LED_Init();//��ʼ��������,��ʱ10s
    LED0 = 1; //������
    LED1 = 0; //�̵����ţ���ʾ����������

    erhaomiao = xitongshijian * 0.5f;
    wuhaomiao = xitongshijian * 0.2f;
    shihaomiao = xitongshijian * 0.1f;
    ershihaomiao = xitongshijian * 0.05f;
    wushihaomiao = xitongshijian * 0.02f;
    yibaihaomiao = xitongshijian * 0.01f;
    miaozhong = xitongshijian * 0.001f;

    //------------------------------��ʼ���½�------------------------------


    while(1)
    {
        //-----------------------------����������ˢ���Ͻ�----------------------------------------------

        if(jiesuokeyi) //������Խ���
        {
            if(channel3_in > 1100)//ͨ�������ܵ�����1100ռ�ձȵ�������Ʋ��ź�
            {
                desthrottle = channel3_in;//��Ҫ�����ŵ���ͨ�������յ����ź�ռ�ձ�

                if(channel1_in < 1507 - deadzone || channel1_in > 1507 + deadzone)//ͨ��һ�����źŲ��ڹ��������м�
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

            //printf("temp1=%d\r\n temp2=%d\r\n temp3=%d\r\n\r\n",(int)temp1, (int)temp2,(int)temp3);//������ͨ�������ֵ
            //printf("temp1=%f\r\n temp2=%f\r\n temp3=%f\r\n\r\n",temp1,temp2,temp3);//������ͨ�������ֵ
            //printf("desroll=%d\r\n despitch=%d\r\n desyaw=%d\r\n\r\n",desroll,despitch,desyaw);//������ͨ�������ֵ

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


        //----------------------------����λ�������Ͻ�-------------------------------------------------

        //ANO_DT_Send_Senser((s16)(aacx_s * 1000), (s16)(aacy_s * 1000), (s16)(aacz_s * 1000), (s16)(gyrox_sd * 10), (s16)(gyroy_sd * 10), (s16)(gyroz_sd * 10), (s16)0, (s16)0, (s16)0, (s32)0);
        //ANO_DT_Send_Status(roll, pitch, yaw, (s32)0, (u8)0, (u8)0);

        //ANO_DT_Send_Senser((s16)(aacx_s * 1000), (s16)(sliding_windows_filter(aacx_s * 1000)), (s16)(aacz_s * 1000), (s16)(gyrox_sd * 10), (s16)(gyroy_sd * 10), (s16)(gyroz_sd * 10), (s16)0, (s16)0, (s16)0, (s32)0);//�����˲�Ч��
        //printf("gyrox_chushi=%d\r\n gyroy_chushi=%d\r\n gyroz_chushi=%d\r\n\r\n",gyrox_chushi, gyroy_chushi,gyroz_chushi);//������ͨ�������ֵ

        //----------------------------����λ�������½�-------------------------------------------------




        //----------------------------���жϺ����ڲ���������ִ�г����Ͻ�-----------------------------------
        //�����뺯���Ͻ�
        /*
                if(xitongshijian * 0.5f > erhaomiao + 1)
                {
                    erhaomiao=xitongshijian*0.5f;//������һ��,����������ܾ�ȷ����
                }
        */
        //�����뺯���½�
        if(xitongshijian * 0.2f > wuhaomiao + 1)
        {
            wuhaomiao = xitongshijian * 0.2f; //ÿ�����������һ��

            if(!MPU_Get_Accelerometer(&aacx, &aacy, &aacz)) //�õ����ٶȴ���������,��ʱ0.6ms
            {
                //aacx_s = (float)aacx * 0.0005978;
                //aacy_s = (float)aacy * 0.0005978;
                //aacz_s = (float)aacz * 0.0005978;

                //Accz_filter();//���ٶȼ�Z�Ử�������˲�
                //ACC_IIR_Filter();//�Ӽ�IIR��ͨ�˲���,��5ms������10Hz�˲�
                acc_Climb_update();
                acc_Climb_dt = (float)(xitongshijian - acc_Climb_temp_time) * 0.001;
                acc_Climb_temp_time = xitongshijian;
                acc_Climb += (acc_Climb_X_hat_minus - aacz_chushi) * 0.0005978 * acc_Climb_dt;
				acc_Climb_out=acc_Climb-acc_Climb_err;
            }

            if(!MPU_Get_Gyroscope(&gyrox, &gyroy, &gyroz))  //�õ�����������,��ʱ0.6ms
            {

                Gyro_filter();//���������˲�,��ʱ0.02ms

                //gyrox_sr = (float)gyrox * 0.0010653;
                //gyroy_sr = (float)gyroy * 0.0010653;
                //gyroz_sr = (float)gyroz * 0.0010653;

                //gyrox_sd = (float)gyrox * 0.0610352;
                //gyroy_sd = (float)gyroy * 0.0610352;
                //gyroz_sd = (float)gyroz * 0.0610352;

            }


        }

        if(xitongshijian * 0.1f > shihaomiao + 1)
        {
            shihaomiao = xitongshijian * 0.1f; //ÿ��ʮ��������һ��
            mpu_dmp_get_data(&pitch, &roll, &yaw);//�˾仰���ľ��˵�52ms,ȥ��50ms�ӳٺ�ֻ��Ҫ2.1ms,����mpuӲ����̬
            roll_err = roll - desroll; //�õ�roll�Ƕ����
            pitch_err = pitch - despitch; //�õ�pitch�Ƕ����
            yaw_err = yaw - desyaw; //�õ�yaw�Ƕ����
            //ag2q2rpy(gyrox_sr+0.0553938, gyroy_sr-0.0170442, gyroz_sr-0.0159790, aacx-960, aacy-350, aacz+1085, &pitch, &roll, &yaw);//�����ʱ0.25ms;
            //__nop();//���������ܺ�ʱ1.4ms,ÿ���Ӹ���101��
            //ANO_DT_Send_Senser(aacx,aacy,aacz,gyrox,gyroy,gyroz);
            //ANO_DT_Send_Senser((int8_t)roll*100,(int8_t)pitch*100,(int8_t)yaw,gyrox,gyroy,gyroz);
            //ANO_DT_Send_Status(roll, pitch, yaw, (s32)0, (u8)0, (u8)0);//��ʱ0.37ms
            //ANO_DT_Send_Senser(aacx, aacy, aacz, gyrox, gyroy, gyroz);
            //__nop();

            //cN2rpy();//ʾ���鿴���������,����ʵ�Ƕ�Ϊ��ֵ,����Ҫ�Ƕ�Ϊ��ֵ
            //ANO_DT_Send_Senser(aacx, aacy, aacz, gyrox-gyrox_chushi, gyroy-gyroy_chushi, gyroz-gyroz_chushi,(s16)rjz,(s16)pjz,(s16)yjz,(s32)0);
            //ANO_DT_Send_Senser((s16)gyrox_filter[6]-gyrox_chushi,(s16)gyroy_filter[6]-gyroy_chushi ,(s16)gyroz_filter[6]-gyroz_chushi , gyrox-gyrox_chushi, gyroy-gyroy_chushi, gyroz-gyroz_chushi,(s32)rjz,(s32)pjz,(s32)yjz,0);
            //ANO_DT_Send_Status(roll, pitch, yaw, (s32)0, (u8)0, (u8)0);

            //ANO_DT_Send_Senser(gyrox-gyrox_chushi, gyroy-gyroy_chushi, gyroz-gyroz_chushi, gyrox_out, gyroy_out, gyroz_out,gyrox_chushi,gyroy_chushi,gyroz_chushi,(s32)0);
        }
        if(xitongshijian * 0.05f > ershihaomiao + 1)
        {
            ershihaomiao = xitongshijian * 0.05f; //ÿ����ʮ��������һ��
            //0.09*65536/4=1475(shortΪint16_t,65536��������2g)
            //desthrottle -= (int16_t)(0.03 * (float)(accz_out - aacz_chushi)); //��aacz���ڳ�ʼʱ��˵���ɻ�����,����Ӧ�ü�С,���̫�ֲ��ˣ��˵�����
            if(channel3_in > 1100)//ֻ�����Ŵ���1100ʱ������������ź��Զ�������
            {
                cyberNation();//���µ��,����50Hz���·�,over 0.06ms
				Altitude_hold_update();//���ߵ�����
                Moto_Throttle(desthrottle);//ֻ��������,���������������õײ�ֱ�ӿ��Ƶ���ĺ���.���Ķ�����һ
            }

            //MS561101BA_GetTemperature();//��ȡ�¶�,����20.6ms,ȥ��10ms�ӳٺ���Ҫ10ms,�Լ���10ms��Ϊ8ms��,ֻ��Ҫ8ms
            //MS561101BA_getPressure();   //��ȡ����ѹ,����20.6ms,ȥ���ӳٺ�,ȥ��10ms�ӳٺ���Ҫ10ms,�Լ���10ms��Ϊ8ms��,ֻ��Ҫ8ms

            //printf("����ڸ�����");//��������ͨ����׽��ͬʱ����Ƿ�����

            /*if(!MPU_Get_Accelerometer(&aacx, &aacy, &aacz))   //�õ����ٶȴ���������,��ʱ0.58ms
            {
                //aacx_s = (float)aacx * 0.0005978;
                //aacy_s = (float)aacy * 0.0005978;
                //aacz_s = (float)aacz * 0.0005978;
            }

            if(!MPU_Get_Gyroscope(&gyrox, &gyroy, &gyroz))  //�õ�����������,��ʱ0.58ms
            {

                //gyrox_sr = (float)gyrox * 0.0010653;
                //gyroy_sr = (float)gyroy * 0.0010653;
                //gyroz_sr = (float)gyroz * 0.0010653;

                //gyrox_sd = (float)gyrox * 0.0610352;
                //gyroy_sd = (float)gyroy * 0.0610352;
                //gyroz_sd = (float)gyroz * 0.0610352;
            }

            //ag2q2rpy(gyrox_sr+0.0553938, gyroy_sr-0.0170442, gyroz_sr-0.0159790, aacx-960, aacy-350, aacz+1085, &pitch, &roll, &yaw);//�����ʱ0.25ms;
            //__nop();//���������ܺ�ʱ1.4ms,ÿ���Ӹ���101��
            //ANO_DT_Send_Senser(aacx,aacy,aacz,gyrox,gyroy,gyroz);
            //ANO_DT_Send_Senser((int8_t)roll*100,(int8_t)pitch*100,(int8_t)yaw,gyrox,gyroy,gyroz);
            //ANO_DT_Send_Status(roll, pitch, yaw, (s32)0, (u8)0, (u8)0);//��ʱ0.37ms
            //ANO_DT_Send_Senser(aacx, aacy, aacz, gyrox, gyroy, gyroz);
            //__nop();*/

        }



        //----------------��ʮ�����Ͻ�

        if(xitongshijian * 0.02f > wushihaomiao + 1)
        {
            wushihaomiao = xitongshijian * 0.02f; //��ʮ����
			ANO_DT_Send_Status(acc_Climb*100, acc_Climb_out*100, Climb_X_hat_minus*100, (s32)MS5611_Altitude*100, (u8)0, (u8)0); //over 0.4ms
			
            //ANO_DT_Send_Status(roll, pitch, yaw, (s32)MS5611_Altitude, (u8)0, (u8)0);//over 0.4ms
            ANO_DT_Send_Senser(aacx, aacy, (s16)acc_Climb_X_hat_minus * 0.05978, gyrox_out, gyroy_out, gyroz_out, (s16)0, (s16)0, (s16)0, (s32)MS5611_Altitude*100); //over 0.5ms
            //ANO_DT_Send_MotoPWM((u16) cNd1, (u16) cNd2, (u16) cNd3, (u16) cNd4, (u16) 0, (u16) 0, (u16) 0, (u16) 0); //over 0.5ms
            //ANO_DT_Send_RCData((u16)channel3_in, (u16) channel4_in, (u16) channel1_in, (u16) channel2_in, (u16) 0, (u16) 0, (u16) 0, (u16) 0, (u16) 0, (u16) 0); //0.5ms

            //printf("  MS5611_Altitude =%fm\r\n", MS5611_Altitude);
            //ANO_DT_Send_Senser(aacx, accz_out/10, aacz/10, gyrox_out, gyroy_out, gyroz_out,(s16)desthrottle,(s16)0,(s16)0,(s32)0);//over 0.5ms

        }

        //---------------��ʮ�����½�

        //---------------һ�ٺ����Ͻ�
        if(xitongshijian * 0.01f > yibaihaomiao + 1)
        {
            yibaihaomiao = xitongshijian * 0.01f; //һ�ٺ�����һ��
            //MS561101BA_GetTemperature();//over 9.1ms
            MS561101BA_getPressure();//over 9.1ms
            Kalman_filter_pressure();
            scaling = pressure_X_hat_minus / Pressure_chushi;
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
			Altitude_minus=MS5611_Altitude;
        }
        //---------------һ�ٺ����½�


        //----------------------һ�����������Ͻ�

        if(xitongshijian * 0.001f > miaozhong + 1)
        {
            miaozhong = xitongshijian * 0.001f; //ÿ��һ��������һ��
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

        //---------------------------һ�����������½�
        //----------------------------���жϺ����ڲ���������ִ�г����½�-----------------------------------

    }
}



