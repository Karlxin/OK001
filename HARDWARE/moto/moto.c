#include "moto.h"
#include "stm32f10x.h"
#include "delay.h"
#include "math.h"

int16_t MOTO1_PWM = 0;
int16_t MOTO2_PWM = 0;
int16_t MOTO3_PWM = 0;
int16_t MOTO4_PWM = 0;

extern int16_t cNd1, cNd2, cNd3, cNd4;
extern float roll, pitch, yaw; 		//roll -180~180 pitch -90~90 yaw -180~180
extern short gyrox, gyroy, gyroz;	//-32768~32767
extern short gyrox_chushi,gyroy_chushi,gyroz_chushi;

extern float ACC_IIR_FACTOR;

//roll:-180~180 pitch:-90:90 yaw:-180:180
//omegaroll:omegapitch:omegayaw:-32768:32767

//对于x,y轴的角度最多允许控制300油门,对于z轴的角度最多允许控制0油门
//static float kp_theta_x_c=300,kp_theta_y_c=300,kp_theta_z_c=0;

//对于x,y的角速度最多允许控制150油门,对于z轴的角速度最多允许控制25油门
//static float kp_omega_x_c=150,kp_omega_y_c=150,kp_omega_z_c=25;

//coefficient Proportion for Angle,coefficient Proportion for palstance,aka angular velocity
/*static float kp_theta_x = 1.6666667, kp_theta_y = 3.3333333, kp_theta_z = 0;
static float kp_omega_x = 0.0045778, kp_omega_y = 0.0045778, kp_omega_z = 0.0007629;*/

//PD controller top
//Test PD controller
//角度预估在15度以内,故让角度控制的油门为15*4=60
//角速度暂时没有估计,暂时估计在10000以内
//32767*0.03=900
//10000*0.03=300
//角速度转换为角度再输入进来，±2000，也就是相当于输出角度=原始数据乘以0.0610370,那么将这个数放到KP_OMEGA也行
float KP_THETA_X = 3.5, KP_THETA_Y = 3.5, KP_THETA_Z = 3.5;//常量
float KP_OMEGA_X = 0.6*0.0610370, KP_OMEGA_Y = 0.5*0.0610370, KP_OMEGA_Z = 0.7*0.0610370;//常量
float kp_theta_x = 3.5, kp_theta_y = 3.5, kp_theta_z = 3.5;//变量
float kp_omega_x = 0.6*0.0610370, kp_omega_y = 0.5*0.0610370, kp_omega_z = 0.7*0.0610370;//变量
//6*130=780;
//0.5*1000=500;
//consideration of mass of 1837g for MATLAB theory 
float KP_VEL_Z=5*0.5443658;
float KP_ACC_Z=0.5*0.5443658;
float kp_vel_Z=5*0.5443658;
float kp_acc_Z=0.5*0.5443658;
//PD controller bottom

extern float roll_err, pitch_err, yaw_err;//误差值,roll_err=roll-desroll,其中desroll为遥控信号线性映射的角度值
extern float desroll, despitch, desyaw;//定义想要的横滚，俯仰，偏航，油门

extern short gyrox_out,gyroy_out,gyroz_out;
extern short aacx, aacy, aacz;     //加速度传感器原始数据
extern short accz_out;

extern float Ahd;
extern u16 channel3_in;

extern float Scd;

//just constrain from right
int16_t Constrain_up(int16_t throttle,int16_t max)
{
	if(throttle>max)
	{
		return max;
	}
	return throttle;
}

//constrain both side
int16_t Constrain(int16_t throttle,int16_t max,int16_t min)
{
	if((min<throttle)&&(throttle<max))
	{
		return throttle;
	}
	else
	{
		if(max<=throttle)
		{
			return max;
		}
	}
	return min;
}


//directive controller motor ,core 1
void Moto_PwmRflash(int16_t MOTO1_PWM, int16_t MOTO2_PWM, int16_t MOTO3_PWM, int16_t MOTO4_PWM)
{

    if(MOTO1_PWM > Moto_PwmMax)	MOTO1_PWM = Moto_PwmMax;//最大油门保持
    if(MOTO2_PWM > Moto_PwmMax)	MOTO2_PWM = Moto_PwmMax;
    if(MOTO3_PWM > Moto_PwmMax)	MOTO3_PWM = Moto_PwmMax;
    if(MOTO4_PWM > Moto_PwmMax)	MOTO4_PWM = Moto_PwmMax;
    if(MOTO1_PWM < 1000)	MOTO1_PWM = 1000;//最小油门保持
    if(MOTO2_PWM < 1000)	MOTO2_PWM = 1000;
    if(MOTO3_PWM < 1000)	MOTO3_PWM = 1000;
    if(MOTO4_PWM < 1000)	MOTO4_PWM = 1000;

    TIM3->CCR1 = MOTO1_PWM;
    TIM3->CCR2 = MOTO2_PWM;
    TIM3->CCR3 = MOTO3_PWM;
    TIM3->CCR4 = MOTO4_PWM;
}

//core 2
void Moto_Throttle(int16_t desthrottle)
{
    int16_t d1, d2, d3, d4;
    d1 = Constrain_up(desthrottle,1780)+Constrain(cNd1,300,-300)+Constrain((int16_t)Ahd,30,-30)+Constrain((int16_t)Scd,30,-30); //              CW3     1CCW	   / \				 
    d2 = Constrain_up(desthrottle,1780)+Constrain(cNd2,300,-300)+Constrain((int16_t)Ahd,30,-30)+Constrain((int16_t)Scd,30,-30); //  俯视图          * *           / | \ X轴      	  Y轴
    d3 = Constrain_up(desthrottle,1780)+Constrain(cNd3,300,-300)+Constrain((int16_t)Ahd,30,-30)+Constrain((int16_t)Scd,30,-30); //                   *              |                <=======
    d4 = Constrain_up(desthrottle,1780)+Constrain(cNd4,300,-300)+Constrain((int16_t)Ahd,30,-30)+Constrain((int16_t)Scd,30,-30); //      	    CCW2    4CW         |

    Moto_PwmRflash(d1, d2, d3, d4);//core 1 called in place 3
}

float sfabs(float a)//单精度绝对值函数
{
	if(a>0.0000000)
	{
		return a;
	}
		
	return -a;
}


//cybernation offset.core 3
void cyberNation(void)
{
	cNd1 = +roll_err * kp_theta_x + pitch_err * kp_theta_y + yaw_err * kp_theta_z + gyrox_out * kp_omega_x + gyroy_out * kp_omega_y + gyroz_out * kp_omega_z;
    cNd2 = -roll_err * kp_theta_x - pitch_err * kp_theta_y + yaw_err * kp_theta_z - gyrox_out * kp_omega_x - gyroy_out * kp_omega_y + gyroz_out * kp_omega_z;
    cNd3 = -roll_err * kp_theta_x + pitch_err * kp_theta_y - yaw_err * kp_theta_z - gyrox_out * kp_omega_x + gyroy_out * kp_omega_y - gyroz_out * kp_omega_z;
    cNd4 = +roll_err * kp_theta_x - pitch_err * kp_theta_y - yaw_err * kp_theta_z + gyrox_out * kp_omega_x - gyroy_out * kp_omega_y - gyroz_out * kp_omega_z;
}

#define Filter_Num 6//sliding window with 6 values
void Gyro_filter(void)
{
	static short Filter_x[Filter_Num],Filter_y[Filter_Num],Filter_z[Filter_Num];
	static uint8_t Filter_count;
	int32_t Filter_sum_x=0,Filter_sum_y=0,Filter_sum_z=0;
	uint8_t i;
	
	Filter_x[Filter_count]=gyrox-gyrox_chushi;
	Filter_y[Filter_count]=gyroy-gyroy_chushi;
	Filter_z[Filter_count]=gyroz-gyroz_chushi;
	
	for(i=0;i<Filter_Num;i++)
	{
		Filter_sum_x+=Filter_x[i];
		Filter_sum_y+=Filter_y[i];
		Filter_sum_z+=Filter_z[i];
	}
	
	gyrox_out=Filter_sum_x/Filter_Num;
	gyroy_out=Filter_sum_y/Filter_Num;
	gyroz_out=Filter_sum_z/Filter_Num;
	
	Filter_count++;
	
	if(Filter_count==Filter_Num)
	{
		Filter_count=0;
	}
}

#define Filter_Num2 6//sliding window with 6 values
void Accz_filter(void)
{
	static short Filter_accz[Filter_Num2];
	static uint8_t Filter_count2;
	int32_t Filter_sum_accz=0;
	uint8_t i;
	
	Filter_accz[Filter_count2]=aacz;
	
	for(i=0;i<Filter_Num;i++)
	{
		Filter_sum_accz+=Filter_accz[i];
	}
	
	accz_out=Filter_sum_accz/Filter_Num;
	
	Filter_count2++;
	
	if(Filter_count2==Filter_Num2)
	{
		Filter_count2=0;
	}
}

/******************************************************************************
函数原型：	void Calculate_FilteringCoefficient(float Time, float Cut_Off)
功    能：	iir低通滤波参数计算
*******************************************************************************/ 
void Calculate_FilteringCoefficient(float Time, float Cut_Off)
{
	ACC_IIR_FACTOR = Time /( Time + 1/(2.0f*3.1415927*Cut_Off) );
}

/******************************************************************************
函数原型：	void ACC_IIR_Filter(struct _acc *Acc_in,struct _acc *Acc_out)
功    能：	iir低通滤波
*******************************************************************************/ 
void ACC_IIR_Filter(void)
{
	accz_out = accz_out + ACC_IIR_FACTOR*(aacz - accz_out); 
}

extern float accz_R;
extern float accz_Q;
extern float accz_K;
extern float accz_X_hat;
extern float accz_X_hat_minus;
extern float accz_P;

//this function use kalman filter to filt accz
void Kalman_filter_accz(void)
{
	//time update
	accz_X_hat_minus=accz_X_hat;
    accz_P=accz_P+accz_Q;
	
	//predict update
	accz_K=accz_P/(accz_P+accz_R);
	accz_X_hat=accz_X_hat_minus+accz_K*(aacz-accz_X_hat_minus);
	accz_P=(1-accz_K)*accz_P;
}




extern float accy_R;
extern float accy_Q;
extern float accy_K;
extern float accy_X_hat;
extern float accy_X_hat_minus;
extern float accy_P;

//this function use kalman filter to filt accy
void Kalman_filter_accy(void)
{
	//time update
	accy_X_hat_minus=accy_X_hat;
    accy_P=accy_P+accy_Q;
	
	//predict update
	accy_K=accy_P/(accy_P+accy_R);
	accy_X_hat=accy_X_hat_minus+accy_K*(aacy-accy_X_hat_minus);
	accy_P=(1-accy_K)*accy_P;
}

extern float accx_R;
extern float accx_Q;
extern float accx_K;
extern float accx_X_hat;
extern float accx_X_hat_minus;
extern float accx_P;

//this function use kalman filter to filt accx
void Kalman_filter_accx(void)
{
	//time update
	accx_X_hat_minus=accx_X_hat;
    accx_P=accx_P+accx_Q;
	
	//predict update
	accx_K=accx_P/(accx_P+accx_R);
	accx_X_hat=accx_X_hat_minus+accx_K*(aacx-accx_X_hat_minus);
	accx_P=(1-accx_K)*accx_P;
}


extern float acc_Climb_out;
extern short aacz_chushi;

void Altitude_hold_update(void)
{
	if(1500<channel3_in<1550)
	{
	Ahd=-kp_vel_Z*acc_Climb_out-kp_acc_Z*(accz_X_hat_minus - (float)aacz_chushi);
	Ahd=0;//暂时不要这个补偿
	}
	Ahd=0;//防止意外的发生嘛
}


//degree to radian by multiplier 0.0174533
void Sink_compensation(void)
{
	Scd=(channel3_in-1000)/(cosf(desroll* 0.0174533)*cosf(despitch* 0.0174533))-(channel3_in-1000);//cosine between EarthFrame_Z with BodyFrame_Z
	Scd=0;//先测试以前的还能工作吗
}


