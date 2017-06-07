/*--------------------------------version information top-------------------------------------------
Started at 2016
Created by Karlxin(410824290@qq.com)
Github:https://github.com/Karlxin/OK001.git
OpenKarlCopter
























--------------------------------version information bottom-------------------------------------------*/
#include "timer.h"
#include <stdio.h>

extern u32 xitongshijian;
extern u32 erhaomiao;
extern u32 wuhaomiao;
extern u32 shihaomiao;
extern u32 ershihaomiao;
extern u32 wushihaomiao;
extern u32 miaozhong;

extern float roll, pitch, yaw; 		//ŷ����
extern short aacx, aacy, aacz;		//���ٶȴ�����ԭʼ����
extern short gyrox, gyroy, gyroz;	//������ԭʼ����
extern short temp;					//mpu�¶�

extern float aacx_s, aacy_s, aacz_s, gyrox_sr, gyroy_sr, gyroz_sr; //��ԭʼ����ת��Ϊ��׼����,�Ի��ȼ�
extern float gyrox_sd, gyroy_sd, gyroz_sd; //��Ԭ������ת��Ϊ��׼���ݣ��Զȼ�
extern float gyrox_sr_kf, gyroy_sr_kf, gyroz_sr_kf; //�������˲����ֵ

extern int32_t  TEMP;					//��ѹ���¶�
extern float MS561101BA_get_altitude(float scaling);//��ø߶ȣ���ʵ�Ǽ�����߶�
extern uint32_t Pressure;				//����ѹ//��λ0.01mbar
extern int32_t  TEMP;					//��ѹ���¶�

extern void MS561101BA_getPressure(void);
extern void MS561101BA_GetTemperature(void);


extern u8 MPU_Get_Gyroscope(short *gx, short *gy, short *gz);
extern u8 MPU_Get_Accelerometer(short *ax, short *ay, short *az);
extern short MPU_Get_Temperature(void);
extern void ag2q2rpy(float gx, float gy, float gz, float ax, float ay, float az, float *pitch, float *roll, float *yaw);

extern void ANO_DT_Send_Senser(s16 a_x, s16 a_y, s16 a_z, s16 g_x, s16 g_y, s16 g_z);
extern void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, s32 alt, u8 fly_model, u8 armed);

extern u8 mpu_dmp_get_data(float *pitch, float *roll, float *yaw);//�����ʱ����һ��ʱ60ms,ʵ�ʺ��񲢲���.

extern void cyberNation(void);//�����Զ�������

extern float KalmanFilter(const float ResrcData, float ProcessNiose_Q, float MeasureNoise_R, float InitialPrediction);

u8 TIM4CH1_CAPTURE_STA = 0;	//ͨ��1���벶���־������λ�������־����6λ�������־
u16 TIM4CH1_CAPTURE_UPVAL;
u16 TIM4CH1_CAPTURE_DOWNVAL;

u8 TIM4CH2_CAPTURE_STA = 0;	//ͨ��2���벶���־������λ�������־����6λ�������־
u16 TIM4CH2_CAPTURE_UPVAL;
u16 TIM4CH2_CAPTURE_DOWNVAL;

u8 TIM4CH3_CAPTURE_STA = 0;	//ͨ��3���벶���־������λ�������־����6λ�������־
u16 TIM4CH3_CAPTURE_UPVAL;
u16 TIM4CH3_CAPTURE_DOWNVAL;

u8 TIM4CH4_CAPTURE_STA = 0;	//ͨ��1���벶���־������λ�������־����6λ�������־
u16 TIM4CH4_CAPTURE_UPVAL;
u16 TIM4CH4_CAPTURE_DOWNVAL;

u32 tempup1 = 0;	//�����ܸߵ�ƽ��ʱ��
u32 tempup2 = 0;	//�����ܸߵ�ƽ��ʱ��
u32 tempup3 = 0;	//�����ܸߵ�ƽ��ʱ��
u32 tempup4 = 0;	//�����ܸߵ�ƽ��ʱ��
u32 tim4_T1;
u32 tim4_T2;
u32 tim4_T3;
u32 tim4_T4;

extern u32 yibaihaomiao2;

extern u32 channel1_in, channel2_in, channel3_in, channel4_in; 				//�յ���ң��ռ�ձ�(1000~2000)

//Timer 5 init top
/*******************************************************************************
	* @Name				TIM5_Int_Init
	* @Description		to initialize timer 5
	* @Input			arr,psc
	* @Use				None
	* @Output			None
	* @Return			None
*******************************************************************************/
void TIM5_Int_Init(
u16 arr,
u16 psc)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE); //ʱ��ʹ��

    TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
    TIM_TimeBaseStructure.TIM_Prescaler = psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ  10Khz�ļ���Ƶ��
    TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ

    TIM_ITConfig( TIM5, TIM_IT_Update , ENABLE );

    NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  //��ռ���ȼ�
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //�����ȼ�
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
    NVIC_Init(&NVIC_InitStructure);  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���

    TIM_Cmd(TIM5, ENABLE);  //ʹ��TIMx����
}
//Timer 5 init bottom

extern float scaling;
extern float MS5611_Pressure;
extern float Pressure_chushi;
extern float baro_climb_rate;
extern float Altitude_out;
extern float Altitude_minus;
extern void Kalman_filter_alt(void);
extern float pressure_X_hat_minus;
extern float pressure_X_hat;
extern float Altitude_X_hat;
extern float Altitude_X_hat_minus;
extern void Derivative_Filter_baro_climb_rate(void);
extern float Altitude_samples[7];
extern u8 Altitude_sample_index;
extern float Altitude_samples_time_stamps[7];
extern void Kalman_filter_baro_climb(void);
extern u8 Altitude_samples_full;

/*******************************************************************************
	* @Name				TIM5_IRQHandler
	* @Description		to handle interrupt by timer 5
	* @Input			None
	* @Use				None
	* @Output			xitongshijian
	* @Return			None
*******************************************************************************/
void TIM5_IRQHandler(void)   //TIM5�ж�,ÿ��һ����͸�һ��
{
    if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET) //���ָ����TIM�жϷ������:TIM �ж�Դ
    {
        TIM_ClearITPendingBit(TIM5, TIM_IT_Update  );  //���TIMx���жϴ�����λ:TIM �ж�Դ
        xitongshijian++;//ÿ�������һ��
    }
}



/*******************************************************************************
	* @Name				TIM4_Cap_Init
	* @Description		init timer 4 PWM capture
	* @Input			arr,psc
	* @Use				None
	* @Output			None
	* @Return			None
*******************************************************************************/
void TIM4_Cap_Init(
u16 arr,
u16 psc)
{
	TIM_ICInitTypeDef TIM4_ICInitStructure;
	
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);	//ʹ��TIM4ʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);  //ʹ��GPIOBʱ��

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9; //PB6,7,8,9 ���֮ǰ����
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //PB6,7,8,9 ����
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOB, GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9);//PB6,7,8,9  ����

    //��ʼ����ʱ��4 TIM4
    TIM_TimeBaseStructure.TIM_Period = arr; //�趨�������Զ���װֵ
    TIM_TimeBaseStructure.TIM_Prescaler = psc; 	//Ԥ��Ƶ��
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ

    //��ʼ��TIM4���벶����� ͨ��1
    TIM4_ICInitStructure.TIM_Channel = TIM_Channel_1; //CC1S=01 	ѡ������� IC1ӳ�䵽TI1��
    TIM4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//�����ز���
    TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
    TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	  //���������Ƶ,����Ƶ
    TIM4_ICInitStructure.TIM_ICFilter = 0x00;	  //IC1F=0000 ���������˲��� ���˲�
    TIM_ICInit(TIM4, &TIM4_ICInitStructure);

    //��ʼ��TIM4���벶����� ͨ��2
    TIM4_ICInitStructure.TIM_Channel = TIM_Channel_2; //CC1S=01 	ѡ������� IC1ӳ�䵽TI1��
    TIM4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//�����ز���
    TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
    TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	  //���������Ƶ,����Ƶ
    TIM4_ICInitStructure.TIM_ICFilter = 0x00;	  //IC1F=0000 ���������˲��� ���˲�
    TIM_ICInit(TIM4, &TIM4_ICInitStructure);

    //��ʼ��TIM4���벶����� ͨ��3
    TIM4_ICInitStructure.TIM_Channel = TIM_Channel_3; //CC1S=01 	ѡ������� IC1ӳ�䵽TI1��
    TIM4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//�����ز���
    TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
    TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	  //���������Ƶ,����Ƶ
    TIM4_ICInitStructure.TIM_ICFilter = 0x00;	  //IC1F=0000 ���������˲��� ���˲�
    TIM_ICInit(TIM4, &TIM4_ICInitStructure);

    //��ʼ��TIM4���벶����� ͨ��4
    TIM4_ICInitStructure.TIM_Channel = TIM_Channel_4; //CC1S=01 	ѡ������� IC1ӳ�䵽TI1��
    TIM4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//�����ز���
    TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
    TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	  //���������Ƶ,����Ƶ
    TIM4_ICInitStructure.TIM_ICFilter = 0x00;	  //IC1F=0000 ���������˲��� ���˲�
    TIM_ICInit(TIM4, &TIM4_ICInitStructure);

    //�жϷ����ʼ��
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;  //TIM4�ж�
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //��ռ���ȼ�1��
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //�����ȼ�0��
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
    NVIC_Init(&NVIC_InitStructure);   //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���

    TIM_ITConfig(TIM4, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4,
                 ENABLE);   //����������жϣ�����CC1IE,CC2IE,CC3IE,CC4IE�����ж�

    TIM_Cmd(TIM4, ENABLE); 		//ʹ�ܶ�ʱ��4

}

/*******************************************************************************
	* @Name				TIM4_IRQHandler
	* @Description		to handle interrupt by timer 4
	* @Input			None
	* @Use				None
	* @Output			channel1_in,channel2_in,channel3_in,channel4_in
	* @Return			None
*******************************************************************************/
void TIM4_IRQHandler(void)
{
    if ((TIM4CH1_CAPTURE_STA & 0X80) == 0) 		//��δ�ɹ�����
    {
        if (TIM_GetITStatus(TIM4, TIM_IT_CC1) != RESET) 		//����1���������¼�
        {
            TIM_ClearITPendingBit(TIM4, TIM_IT_CC1); 		//����жϱ�־λ
            if (TIM4CH1_CAPTURE_STA & 0X40)		//����һ���½���
            {
                TIM4CH1_CAPTURE_DOWNVAL = TIM_GetCapture1(TIM4);//��¼�´�ʱ�Ķ�ʱ������ֵ
                if (TIM4CH1_CAPTURE_DOWNVAL < TIM4CH1_CAPTURE_UPVAL)
                {
                    tim4_T1 = 65535;
                }
                else
                    tim4_T1 = 0;
                tempup1 = TIM4CH1_CAPTURE_DOWNVAL - TIM4CH1_CAPTURE_UPVAL
                          + tim4_T1;		//�õ��ܵĸߵ�ƽ��ʱ��
                channel1_in = tempup1<2000?tempup1:channel1_in;		//�ܵĸߵ�ƽ��ʱ��
                TIM4CH1_CAPTURE_STA = 0;		//�����־λ����
                TIM_OC1PolarityConfig(TIM4, TIM_ICPolarity_Rising); //����Ϊ�����ز���
            }
            else //��������ʱ�䵫�����½��أ���һ�β��������أ���¼��ʱ�Ķ�ʱ������ֵ
            {
                TIM4CH1_CAPTURE_UPVAL = TIM_GetCapture1(TIM4);		//��ȡ����������
                TIM4CH1_CAPTURE_STA |= 0X40;		//����Ѳ���������
                TIM_OC1PolarityConfig(TIM4, TIM_ICPolarity_Falling);//����Ϊ�½��ز���
            }
        }
    }

    if ((TIM4CH2_CAPTURE_STA & 0X80) == 0)		//��δ�ɹ�����
    {
        if (TIM_GetITStatus(TIM4, TIM_IT_CC2) != RESET)		//����2���������¼�
        {
            TIM_ClearITPendingBit(TIM4, TIM_IT_CC2);		//����жϱ�־λ
            if (TIM4CH2_CAPTURE_STA & 0X40)		//����һ���½���
            {
                TIM4CH2_CAPTURE_DOWNVAL = TIM_GetCapture2(TIM4);//��¼�´�ʱ�Ķ�ʱ������ֵ
                if (TIM4CH2_CAPTURE_DOWNVAL < TIM4CH2_CAPTURE_UPVAL)
                {
                    tim4_T2 = 65535;
                }
                else
                    tim4_T2 = 0;
                tempup2 = TIM4CH2_CAPTURE_DOWNVAL - TIM4CH2_CAPTURE_UPVAL
                          + tim4_T2;		//�õ��ܵĸߵ�ƽ��ʱ��
                channel2_in = tempup2<2000?tempup2:channel2_in;		//�ܵĸߵ�ƽ��ʱ��
                TIM4CH2_CAPTURE_STA = 0;		//�����־λ����
                TIM_OC2PolarityConfig(TIM4, TIM_ICPolarity_Rising); //����Ϊ�����ز���
            }
            else //��������ʱ�䵫�����½��أ���һ�β��������أ���¼��ʱ�Ķ�ʱ������ֵ
            {
                TIM4CH2_CAPTURE_UPVAL = TIM_GetCapture2(TIM4);		//��ȡ����������
                TIM4CH2_CAPTURE_STA |= 0X40;		//����Ѳ���������
                TIM_OC2PolarityConfig(TIM4, TIM_ICPolarity_Falling);//����Ϊ�½��ز���
            }
        }
    }

    if ((TIM4CH3_CAPTURE_STA & 0X80) == 0)		//��δ�ɹ�����
    {
        if (TIM_GetITStatus(TIM4, TIM_IT_CC3) != RESET)		//����3���������¼�
        {
            TIM_ClearITPendingBit(TIM4, TIM_IT_CC3);		//����жϱ�־λ
            if (TIM4CH3_CAPTURE_STA & 0X40)		//����һ���½���
            {
                TIM4CH3_CAPTURE_DOWNVAL = TIM_GetCapture3(TIM4);//��¼�´�ʱ�Ķ�ʱ������ֵ
                if (TIM4CH3_CAPTURE_DOWNVAL < TIM4CH3_CAPTURE_UPVAL)
                {
                    tim4_T3 = 65535;
                }
                else
                    tim4_T3 = 0;
                tempup3 = TIM4CH3_CAPTURE_DOWNVAL - TIM4CH3_CAPTURE_UPVAL
                          + tim4_T3;		//�õ��ܵĸߵ�ƽ��ʱ��
                channel3_in = tempup3<2000?tempup3:channel3_in;		//�ܵĸߵ�ƽ��ʱ��
                TIM4CH3_CAPTURE_STA = 0;		//�����־λ����
                TIM_OC3PolarityConfig(TIM4, TIM_ICPolarity_Rising); //����Ϊ�����ز���
            }
            else //��������ʱ�䵫�����½��أ���һ�β��������أ���¼��ʱ�Ķ�ʱ������ֵ
            {
                TIM4CH3_CAPTURE_UPVAL = TIM_GetCapture3(TIM4);		//��ȡ����������
                TIM4CH3_CAPTURE_STA |= 0X40;		//����Ѳ���������
                TIM_OC3PolarityConfig(TIM4, TIM_ICPolarity_Falling);//����Ϊ�½��ز���
            }
        }
    }

    if ((TIM4CH4_CAPTURE_STA & 0X80) == 0)		//��δ�ɹ�����
    {
        if (TIM_GetITStatus(TIM4, TIM_IT_CC4) != RESET)		//����4���������¼�
        {
            TIM_ClearITPendingBit(TIM4, TIM_IT_CC4);		//����жϱ�־λ
            if (TIM4CH4_CAPTURE_STA & 0X40)		//����һ���½���
            {
                TIM4CH4_CAPTURE_DOWNVAL = TIM_GetCapture4(TIM4);//��¼�´�ʱ�Ķ�ʱ������ֵ
                if (TIM4CH4_CAPTURE_DOWNVAL < TIM4CH4_CAPTURE_UPVAL)
                {
                    tim4_T4 = 65535;
                }
                else
                    tim4_T4 = 0;
                tempup4 = TIM4CH4_CAPTURE_DOWNVAL - TIM4CH4_CAPTURE_UPVAL
                          + tim4_T4;		//�õ��ܵĸߵ�ƽ��ʱ��
                channel4_in = tempup4<2000?tempup4:channel4_in;		//�ܵĸߵ�ƽ��ʱ��
                TIM4CH4_CAPTURE_STA = 0;		//�����־λ����
                TIM_OC4PolarityConfig(TIM4, TIM_ICPolarity_Rising); //����Ϊ�����ز���
            }
            else //��������ʱ�䵫�����½��أ���һ�β��������أ���¼��ʱ�Ķ�ʱ������ֵ
            {
                TIM4CH4_CAPTURE_UPVAL = TIM_GetCapture4(TIM4);		//��ȡ����������
                TIM4CH4_CAPTURE_STA |= 0X40;		//����Ѳ���������
                TIM_OC4PolarityConfig(TIM4, TIM_ICPolarity_Falling);//����Ϊ�½��ز���
            }
        }
    }
}
