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

extern float roll, pitch, yaw; 		//欧拉角
extern short aacx, aacy, aacz;		//加速度传感器原始数据
extern short gyrox, gyroy, gyroz;	//陀螺仪原始数据
extern short temp;					//mpu温度

extern float aacx_s, aacy_s, aacz_s, gyrox_sr, gyroy_sr, gyroz_sr; //将原始数据转换为标准数据,以弧度计
extern float gyrox_sd, gyroy_sd, gyroz_sd; //将袁术数据转换为表准数据，以度计
extern float gyrox_sr_kf, gyroy_sr_kf, gyroz_sr_kf; //卡尔曼滤波后的值

extern int32_t  TEMP;					//气压计温度
extern float MS561101BA_get_altitude(float scaling);//获得高度，其实是计算出高度
extern uint32_t Pressure;				//大气压//单位0.01mbar
extern int32_t  TEMP;					//气压计温度

extern void MS561101BA_getPressure(void);
extern void MS561101BA_GetTemperature(void);


extern u8 MPU_Get_Gyroscope(short *gx, short *gy, short *gz);
extern u8 MPU_Get_Accelerometer(short *ax, short *ay, short *az);
extern short MPU_Get_Temperature(void);
extern void ag2q2rpy(float gx, float gy, float gz, float ax, float ay, float az, float *pitch, float *roll, float *yaw);

extern void ANO_DT_Send_Senser(s16 a_x, s16 a_y, s16 a_z, s16 g_x, s16 g_y, s16 g_z);
extern void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, s32 alt, u8 fly_model, u8 armed);

extern u8 mpu_dmp_get_data(float *pitch, float *roll, float *yaw);//仿真的时候这家伙耗时60ms,实际好像并不是.

extern void cyberNation(void);//更新自动控制量

extern float KalmanFilter(const float ResrcData, float ProcessNiose_Q, float MeasureNoise_R, float InitialPrediction);

u8 TIM4CH1_CAPTURE_STA = 0;	//通道1输入捕获标志，高两位做捕获标志，低6位做溢出标志
u16 TIM4CH1_CAPTURE_UPVAL;
u16 TIM4CH1_CAPTURE_DOWNVAL;

u8 TIM4CH2_CAPTURE_STA = 0;	//通道2输入捕获标志，高两位做捕获标志，低6位做溢出标志
u16 TIM4CH2_CAPTURE_UPVAL;
u16 TIM4CH2_CAPTURE_DOWNVAL;

u8 TIM4CH3_CAPTURE_STA = 0;	//通道3输入捕获标志，高两位做捕获标志，低6位做溢出标志
u16 TIM4CH3_CAPTURE_UPVAL;
u16 TIM4CH3_CAPTURE_DOWNVAL;

u8 TIM4CH4_CAPTURE_STA = 0;	//通道1输入捕获标志，高两位做捕获标志，低6位做溢出标志
u16 TIM4CH4_CAPTURE_UPVAL;
u16 TIM4CH4_CAPTURE_DOWNVAL;

u32 tempup1 = 0;	//捕获总高电平的时间
u32 tempup2 = 0;	//捕获总高电平的时间
u32 tempup3 = 0;	//捕获总高电平的时间
u32 tempup4 = 0;	//捕获总高电平的时间
u32 tim4_T1;
u32 tim4_T2;
u32 tim4_T3;
u32 tim4_T4;

extern u32 yibaihaomiao2;

extern u32 channel1_in, channel2_in, channel3_in, channel4_in; 				//收到的遥控占空比(1000~2000)

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

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE); //时钟使能

    TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值
    TIM_TimeBaseStructure.TIM_Prescaler = psc; //设置用来作为TIMx时钟频率除数的预分频值  10Khz的计数频率
    TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位

    TIM_ITConfig( TIM5, TIM_IT_Update , ENABLE );

    NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  //先占优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //从优先级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
    NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器

    TIM_Cmd(TIM5, ENABLE);  //使能TIMx外设
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
void TIM5_IRQHandler(void)   //TIM5中断,每过一毫秒就搞一次
{
    if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET) //检查指定的TIM中断发生与否:TIM 中断源
    {
        TIM_ClearITPendingBit(TIM5, TIM_IT_Update  );  //清除TIMx的中断待处理位:TIM 中断源
        xitongshijian++;//每毫秒过来一次
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

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);	//使能TIM4时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);  //使能GPIOB时钟

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9; //PB6,7,8,9 清除之前设置
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //PB6,7,8,9 输入
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOB, GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9);//PB6,7,8,9  下拉

    //初始化定时器4 TIM4
    TIM_TimeBaseStructure.TIM_Period = arr; //设定计数器自动重装值
    TIM_TimeBaseStructure.TIM_Prescaler = psc; 	//预分频器
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位

    //初始化TIM4输入捕获参数 通道1
    TIM4_ICInitStructure.TIM_Channel = TIM_Channel_1; //CC1S=01 	选择输入端 IC1映射到TI1上
    TIM4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿捕获
    TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
    TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	  //配置输入分频,不分频
    TIM4_ICInitStructure.TIM_ICFilter = 0x00;	  //IC1F=0000 配置输入滤波器 不滤波
    TIM_ICInit(TIM4, &TIM4_ICInitStructure);

    //初始化TIM4输入捕获参数 通道2
    TIM4_ICInitStructure.TIM_Channel = TIM_Channel_2; //CC1S=01 	选择输入端 IC1映射到TI1上
    TIM4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿捕获
    TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
    TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	  //配置输入分频,不分频
    TIM4_ICInitStructure.TIM_ICFilter = 0x00;	  //IC1F=0000 配置输入滤波器 不滤波
    TIM_ICInit(TIM4, &TIM4_ICInitStructure);

    //初始化TIM4输入捕获参数 通道3
    TIM4_ICInitStructure.TIM_Channel = TIM_Channel_3; //CC1S=01 	选择输入端 IC1映射到TI1上
    TIM4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿捕获
    TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
    TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	  //配置输入分频,不分频
    TIM4_ICInitStructure.TIM_ICFilter = 0x00;	  //IC1F=0000 配置输入滤波器 不滤波
    TIM_ICInit(TIM4, &TIM4_ICInitStructure);

    //初始化TIM4输入捕获参数 通道4
    TIM4_ICInitStructure.TIM_Channel = TIM_Channel_4; //CC1S=01 	选择输入端 IC1映射到TI1上
    TIM4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿捕获
    TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
    TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	  //配置输入分频,不分频
    TIM4_ICInitStructure.TIM_ICFilter = 0x00;	  //IC1F=0000 配置输入滤波器 不滤波
    TIM_ICInit(TIM4, &TIM4_ICInitStructure);

    //中断分组初始化
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;  //TIM4中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //先占优先级1级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //从优先级0级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
    NVIC_Init(&NVIC_InitStructure);   //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器

    TIM_ITConfig(TIM4, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4,
                 ENABLE);   //不允许更新中断，允许CC1IE,CC2IE,CC3IE,CC4IE捕获中断

    TIM_Cmd(TIM4, ENABLE); 		//使能定时器4

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
    if ((TIM4CH1_CAPTURE_STA & 0X80) == 0) 		//还未成功捕获
    {
        if (TIM_GetITStatus(TIM4, TIM_IT_CC1) != RESET) 		//捕获1发生捕获事件
        {
            TIM_ClearITPendingBit(TIM4, TIM_IT_CC1); 		//清除中断标志位
            if (TIM4CH1_CAPTURE_STA & 0X40)		//捕获到一个下降沿
            {
                TIM4CH1_CAPTURE_DOWNVAL = TIM_GetCapture1(TIM4);//记录下此时的定时器计数值
                if (TIM4CH1_CAPTURE_DOWNVAL < TIM4CH1_CAPTURE_UPVAL)
                {
                    tim4_T1 = 65535;
                }
                else
                    tim4_T1 = 0;
                tempup1 = TIM4CH1_CAPTURE_DOWNVAL - TIM4CH1_CAPTURE_UPVAL
                          + tim4_T1;		//得到总的高电平的时间
                channel1_in = tempup1<2000?tempup1:channel1_in;		//总的高电平的时间
                TIM4CH1_CAPTURE_STA = 0;		//捕获标志位清零
                TIM_OC1PolarityConfig(TIM4, TIM_ICPolarity_Rising); //设置为上升沿捕获
            }
            else //发生捕获时间但不是下降沿，第一次捕获到上升沿，记录此时的定时器计数值
            {
                TIM4CH1_CAPTURE_UPVAL = TIM_GetCapture1(TIM4);		//获取上升沿数据
                TIM4CH1_CAPTURE_STA |= 0X40;		//标记已捕获到上升沿
                TIM_OC1PolarityConfig(TIM4, TIM_ICPolarity_Falling);//设置为下降沿捕获
            }
        }
    }

    if ((TIM4CH2_CAPTURE_STA & 0X80) == 0)		//还未成功捕获
    {
        if (TIM_GetITStatus(TIM4, TIM_IT_CC2) != RESET)		//捕获2发生捕获事件
        {
            TIM_ClearITPendingBit(TIM4, TIM_IT_CC2);		//清除中断标志位
            if (TIM4CH2_CAPTURE_STA & 0X40)		//捕获到一个下降沿
            {
                TIM4CH2_CAPTURE_DOWNVAL = TIM_GetCapture2(TIM4);//记录下此时的定时器计数值
                if (TIM4CH2_CAPTURE_DOWNVAL < TIM4CH2_CAPTURE_UPVAL)
                {
                    tim4_T2 = 65535;
                }
                else
                    tim4_T2 = 0;
                tempup2 = TIM4CH2_CAPTURE_DOWNVAL - TIM4CH2_CAPTURE_UPVAL
                          + tim4_T2;		//得到总的高电平的时间
                channel2_in = tempup2<2000?tempup2:channel2_in;		//总的高电平的时间
                TIM4CH2_CAPTURE_STA = 0;		//捕获标志位清零
                TIM_OC2PolarityConfig(TIM4, TIM_ICPolarity_Rising); //设置为上升沿捕获
            }
            else //发生捕获时间但不是下降沿，第一次捕获到上升沿，记录此时的定时器计数值
            {
                TIM4CH2_CAPTURE_UPVAL = TIM_GetCapture2(TIM4);		//获取上升沿数据
                TIM4CH2_CAPTURE_STA |= 0X40;		//标记已捕获到上升沿
                TIM_OC2PolarityConfig(TIM4, TIM_ICPolarity_Falling);//设置为下降沿捕获
            }
        }
    }

    if ((TIM4CH3_CAPTURE_STA & 0X80) == 0)		//还未成功捕获
    {
        if (TIM_GetITStatus(TIM4, TIM_IT_CC3) != RESET)		//捕获3发生捕获事件
        {
            TIM_ClearITPendingBit(TIM4, TIM_IT_CC3);		//清除中断标志位
            if (TIM4CH3_CAPTURE_STA & 0X40)		//捕获到一个下降沿
            {
                TIM4CH3_CAPTURE_DOWNVAL = TIM_GetCapture3(TIM4);//记录下此时的定时器计数值
                if (TIM4CH3_CAPTURE_DOWNVAL < TIM4CH3_CAPTURE_UPVAL)
                {
                    tim4_T3 = 65535;
                }
                else
                    tim4_T3 = 0;
                tempup3 = TIM4CH3_CAPTURE_DOWNVAL - TIM4CH3_CAPTURE_UPVAL
                          + tim4_T3;		//得到总的高电平的时间
                channel3_in = tempup3<2000?tempup3:channel3_in;		//总的高电平的时间
                TIM4CH3_CAPTURE_STA = 0;		//捕获标志位清零
                TIM_OC3PolarityConfig(TIM4, TIM_ICPolarity_Rising); //设置为上升沿捕获
            }
            else //发生捕获时间但不是下降沿，第一次捕获到上升沿，记录此时的定时器计数值
            {
                TIM4CH3_CAPTURE_UPVAL = TIM_GetCapture3(TIM4);		//获取上升沿数据
                TIM4CH3_CAPTURE_STA |= 0X40;		//标记已捕获到上升沿
                TIM_OC3PolarityConfig(TIM4, TIM_ICPolarity_Falling);//设置为下降沿捕获
            }
        }
    }

    if ((TIM4CH4_CAPTURE_STA & 0X80) == 0)		//还未成功捕获
    {
        if (TIM_GetITStatus(TIM4, TIM_IT_CC4) != RESET)		//捕获4发生捕获事件
        {
            TIM_ClearITPendingBit(TIM4, TIM_IT_CC4);		//清除中断标志位
            if (TIM4CH4_CAPTURE_STA & 0X40)		//捕获到一个下降沿
            {
                TIM4CH4_CAPTURE_DOWNVAL = TIM_GetCapture4(TIM4);//记录下此时的定时器计数值
                if (TIM4CH4_CAPTURE_DOWNVAL < TIM4CH4_CAPTURE_UPVAL)
                {
                    tim4_T4 = 65535;
                }
                else
                    tim4_T4 = 0;
                tempup4 = TIM4CH4_CAPTURE_DOWNVAL - TIM4CH4_CAPTURE_UPVAL
                          + tim4_T4;		//得到总的高电平的时间
                channel4_in = tempup4<2000?tempup4:channel4_in;		//总的高电平的时间
                TIM4CH4_CAPTURE_STA = 0;		//捕获标志位清零
                TIM_OC4PolarityConfig(TIM4, TIM_ICPolarity_Rising); //设置为上升沿捕获
            }
            else //发生捕获时间但不是下降沿，第一次捕获到上升沿，记录此时的定时器计数值
            {
                TIM4CH4_CAPTURE_UPVAL = TIM_GetCapture4(TIM4);		//获取上升沿数据
                TIM4CH4_CAPTURE_STA |= 0X40;		//标记已捕获到上升沿
                TIM_OC4PolarityConfig(TIM4, TIM_ICPolarity_Falling);//设置为下降沿捕获
            }
        }
    }
}
