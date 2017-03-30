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

u32 xitongshijian = 0; //ϵͳ����ʱ��,�Ժ���ƣ�����1193+Сʱ
u32 erhaomiao = 0;//������
u32 wuhaomiao = 0;//�����
u32 shihaomiao = 0; //ϵͳ����ʱ�䣬��ʮ�����,����119304+Сʱ
u32 ershihaomiao = 0; //ϵͳ����ʱ��,�Զ�ʮ����,����59652+Сʱ
u32 wushihaomiao = 0;
u32 miaozhong = 0; //ϵͳ����ʱ�䣬�����,����1193000+Сʱ

float roll, pitch, yaw;         //ŷ����,DMPӲ��õ��ĽǶ�,roll -180~180 pitch -90~90 yaw -180~180
short aacx, aacy, aacz;     //���ٶȴ�����ԭʼ����
short gyrox, gyroy, gyroz;  //������ԭʼ����
short temp;                 //�¶�
short gyrox_chushi, gyroy_chushi, gyroz_chushi; //�������ʼ���ڵ���ʱ���һЩ����,������ʼֵҪ��ȥ��
float temp_gyxc = 0, temp_gyyc = 0, temp_gyzc = 0; //����������������ݣ�������������ǳ�ʼֵ,����Ԥ��̬�����ֵƽ��Ȩ��ƽ��
float gyrox_filter[7]= {0},gyroy_filter[7]= {0},gyroz_filter[7]= {0}; //������������ǽ��ٶȵ���ֵ�˲����ݴ�����,����λ���������������ֵ�˲��ķ���ֵ���������˲����ֵ
short gyro_jishu=0;//�˲�����,һ�㵽3��0,����Զ���ܵ�3

extern void filter_threeValue(void);//������ֵ�˲��ĺ���,����ֻ�˲������ǽ��ٶ�

float aacx_s, aacy_s, aacz_s, gyrox_sr, gyroy_sr, gyroz_sr; //��ԭʼ����ת��Ϊ��׼����,�Ի��ȼ�
float gyrox_sd, gyroy_sd, gyroz_sd; //��ԭʼ����ת��Ϊ��׼���ݣ��Զȼ�
float gyrox_sr_kf, gyroy_sr_kf, gyroz_sr_kf;//�������˲����ֵ

int16_t deadzone = 20; //ң������

int16_t cNd1 = 0, cNd2 = 0, cNd3 = 0, cNd4 = 0; //cyberNation ������
float rjz=0,pjz=0,yjz=0;//��cNd1�����ݷֱ�ת��Ϊroll,pitch,yaw����ľ��������Ա�ʾ���۲�

extern void cN2rpy(void);

float x_last = 0; //for karlman
float p_last = 0;

u32 gyxt = 0; //������ʼ�������ǵ�ʱ���¼,gyroxitong

u32 shihaomiao2 = 0; //��ŷǼ�ʱ���жϴ�������������ʮ����ֵ
u32 ershihaomiao2 = 0; //��ŷǼ�ʱ���жϴ������������Ķ�ʮ����ֵ
u32 wushihaomiao2 = 0;//��ŷǼ�ʱ���жϴ���������������ʮ����ֵ
u32 miaozhong2 = 0; //������ŷǼ�ʱ���жϴ�������������ϵͳ����ֵ

extern int32_t  TEMP;                   //��ѹ���¶�
extern float MS561101BA_get_altitude(void);//��ø߶ȣ���ʵ�Ǽ�����߶�
extern uint32_t Pressure;               //����ѹ//��λ0.01mbar
extern int32_t  TEMP;                   //��ѹ���¶�

extern void MS561101BA_getPressure(void);
extern void MS561101BA_GetTemperature(void);

u8 roll_in_flag=0;//��X������Чң���ź�������Ϊ1������Ϊ0
u8 pitch_in_flag=0;
u8 yaw_in_flag=0;

int main(void)
{
    //------------------------------��ʼ���Ͻ�------------------------------

    u8 jiesuokeyi = 0;//����������Ա�־λ
    u16 baochijiesuo = 0;//���屣�ֽ���������
    u16 baochijiasuo = 0;//���屣�ּ���������
    int16_t desroll, despitch, desyaw, desthrottle;//������Ҫ�ĺ����������ƫ��������
    float temp1, temp2, temp3; //������Ϊ�м����,�Լ�������ֵ��������ô��

    SystemInit();//ϵͳ��ʼ��
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//���ȵȼ���Ϊ0��1��2��3���ӵȼ���Ϊ0,1(2:0)
    delay_init();//�ӳٳ�ʼ��

    //Uart1_Init(115200);//��ATKXCOMV2.0������ʱ��Ҫ�򿪵�ͨ���첽�׷����ڲ���������
    //Uart1_Init(500000);//������4.06������ʱ��Ҫ�򿪵�����

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

    gyxt = xitongshijian; //��¼������ʱ������ֵ
    while(xitongshijian * 0.001f < gyxt * 0.001f + 3) //�������뿪��һ������������
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

    LED_Init();//��ʼ��������
    LED0 = 1; //������
    LED1 = 0; //�̵����ţ���ʾ����������

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
                    desroll = 1507 - channel1_in;//��Ҫ�ĺ������ͨ��һ�м��
                    roll_in_flag=1;//ͨ��һ����Ч�ź�����
                }
                else//ͨ��һ�����ź��ڹ��������м�
                {
                    desroll = 0;//��Ҫ�ĺ��Ϊ��
                    roll_in_flag=0;//ͨ��һû��Ч�ź�����
                }
                if(channel2_in < 1508 - deadzone || channel2_in > 1508 + deadzone)//ͨ���������źŲ��ڹ��������м�
                {
                    despitch = 1508 - channel2_in;//��Ҫ�ĸ�������ͨ������λ��
                    pitch_in_flag=1;//ͨ��������Ч�ź�����
                }
                else//ͨ���������ź��ڹ��������м�
                {
                    despitch = 0;//��Ҫ�ĸ���Ϊ��
                    pitch_in_flag=0;//ͨ����û����Ч�ź�����
                }
                if(channel4_in < 1507 - deadzone || channel4_in > 1507 + deadzone)//ͨ���Ľ����źŲ��ڹ��������м�
                {
                    desyaw = 1507 - channel4_in;//��Ҫ��ƫ��Ϊͨ������λ��
                    yaw_in_flag=1;//ͨ��������Ч�ź�����
                }
                else//ͨ���Ľ����ź��ڹ��������м�
                {
                    desyaw = 0;//��Ҫ��ƫ��������
                    yaw_in_flag=0;//ͨ����û����Ч�ź�����
                }
            }
            else
            {
                //desthrottle = 1000;//��Ҫ�����ŵ�����С����,��Ϊ(1000,2000)�ǽ����ź�ֵ��Χ
                desthrottle = 0;//����1000̫�ֲ��ˣ��������������ţ��ڷ�ɢ��ʱ��ȶ��Ȳ���
                desroll = despitch = desyaw = 0;//��Ҫ�ĺ������ƫ����Ϊ��
            }

            temp1 = (float)desroll * 0.200; //��Ϊ��ǰ�����֮һ
            temp2 = (float)despitch * 0.200;
            temp3 = (float)desyaw * 0.200;

            //������仰Ϊ����ʮ��Ҫ������
            temp3 = 0;

            /*Moto_RPY(desroll, despitch, desthrottle, desyaw);//����Ҫ�ĽǶ�������Ƕȵ�����ֵ����*/
            Moto_RPY((int)temp1, (int)temp2, desthrottle, (int)temp3);//���������Ǿ仰ң��ֱ�ӿ��Ƶ���ʵ��̫�����Ըĸ�

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
            Moto_PwmRflash(0, 0, 0, 0);//ȫ��������С��

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
            else
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

        if(xitongshijian * 0.5f > erhaomiao + 1)
        {
            erhaomiao++;//ÿ������������һ��
        }

        if(xitongshijian * 0.2f > wuhaomiao + 1)
        {
            wuhaomiao++;//ÿ�����������һ��
        }

        if(xitongshijian * 0.1f > shihaomiao + 1)
        {
            shihaomiao++;//ÿ��ʮ��������һ��
            cyberNation();//���µ��
            mpu_dmp_get_data(&pitch, &roll, &yaw);//�˾仰���ľ��˵�52ms,ȥ��50ms�ӳٺ�ֻ��Ҫ2.1ms,����mpuӲ����̬

            if(!MPU_Get_Accelerometer(&aacx, &aacy, &aacz)) //�õ����ٶȴ���������,��ʱ0.6ms
            {
                //aacx_s = (float)aacx * 0.0005978;
                //aacy_s = (float)aacy * 0.0005978;
                //aacz_s = (float)aacz * 0.0005978;
            }

            if(!MPU_Get_Gyroscope(&gyrox, &gyroy, &gyroz))  //�õ�����������,��ʱ0.6ms
            {

                //gyrox_sr = (float)gyrox * 0.0010653;
                //gyroy_sr = (float)gyroy * 0.0010653;
                //gyroz_sr = (float)gyroz * 0.0010653;

                //gyrox_sd = (float)gyrox * 0.0610352;
                //gyroy_sd = (float)gyroy * 0.0610352;
                //gyroz_sd = (float)gyroz * 0.0610352;
            }

            filter_threeValue();//��ֵ�˲�

            //ag2q2rpy(gyrox_sr+0.0553938, gyroy_sr-0.0170442, gyroz_sr-0.0159790, aacx-960, aacy-350, aacz+1085, &pitch, &roll, &yaw);//�����ʱ0.25ms;
            //__nop();//���������ܺ�ʱ1.4ms,ÿ���Ӹ���101��
            //ANO_DT_Send_Senser(aacx,aacy,aacz,gyrox,gyroy,gyroz);
            //ANO_DT_Send_Senser((int8_t)roll*100,(int8_t)pitch*100,(int8_t)yaw,gyrox,gyroy,gyroz);
            //ANO_DT_Send_Status(roll, pitch, yaw, (s32)0, (u8)0, (u8)0);//��ʱ0.37ms
            //ANO_DT_Send_Senser(aacx, aacy, aacz, gyrox, gyroy, gyroz);
            //__nop();

            //cN2rpy();//ʾ���鿴������,һ����Ƕ�Ϊ��ֵ
            //ANO_DT_Send_Senser(aacx, aacy, aacz, gyrox-gyrox_chushi, gyroy-gyroy_chushi, gyroz-gyroz_chushi,(s32)rjz,(s32)pjz,(s32)yjz,0);
            //ANO_DT_Send_Senser((s16)gyrox_filter[6]-gyrox_chushi,(s16)gyroy_filter[6]-gyroy_chushi ,(s16)gyroz_filter[6]-gyroz_chushi , gyrox-gyrox_chushi, gyroy-gyroy_chushi, gyroz-gyroz_chushi,(s32)rjz,(s32)pjz,(s32)yjz,0);
            //ANO_DT_Send_Status(roll, pitch, yaw, (s32)0, (u8)0, (u8)0);


        }
        if(xitongshijian * 0.05f > ershihaomiao + 1)
        {
            ershihaomiao++;//ÿ����ʮ��������һ��
            //MS561101BA_GetTemperature();//��ȡ�¶�,����20.6ms,ȥ��10ms�ӳٺ���Ҫ10ms,�Լ���10ms��Ϊ8ms��,ֻ��Ҫ8ms
            //MS561101BA_getPressure();   //��ȡ����ѹ,����20.6ms,ȥ���ӳٺ�,ȥ��10ms�ӳٺ���Ҫ10ms,�Լ���10ms��Ϊ8ms��,ֻ��Ҫ8ms

            //printf("����ڸ�����");//��������ͨ����׽��ͬʱ����Ƿ�����

            //����17ע������϶�
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
            //����17ע������¶�
        }

        if(xitongshijian * 0.02f > wushihaomiao + 1)
        {
            wushihaomiao++;//��ʮ����
        }

        if(xitongshijian * 0.001f > miaozhong + 1)
        {
            miaozhong++;//ÿ��һ��������һ��
            //temp=MPU_Get_Temperature();

            //printf("  mpu temp=%.2f\r\n", temp*0.01f);//mpu�¶�
            //printf("  Temp : %.2f ��\r\n", TEMP * 0.01f);            //�������ԭʼ���ݣ���ѹ�ƶ������¶�
            //printf("  height : %.2f m\r\n", (float)MS561101BA_get_altitude());
            //printf("  Pressure : %.2f mbar\r\n\r\n\r\n", Pressure * 0.01f); //�������ԭʼ����




            //printf("  aacx=%f\r\n", aacx_s);//x����ٶ�,printf���������ATKCOM���ڲ�׽����PC����ʾ
            //printf("  aacy=%f\r\n", aacy_s);
            //printf("  aacz=%f\r\n\r\n", aacz_s);

            //printf("  gyrox=%f\r\n", gyrox_sd);
            //printf("  gyroy=%f\r\n", gyroy_sd);
            //printf("  gyroz=%f\r\n\r\n", gyroz_sd);






//          printf("    roll=%f ��\r\n", roll); //�������̬����Ҫת��
//            printf("  pitch=%f ��\r\n", pitch);
//            printf("  yaw=%f ��\r\n\r\n", yaw);
//
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

        //----------------------------���жϺ����ڲ���������ִ�г����½�-----------------------------------

    }
}



