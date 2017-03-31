#include "moto.h"
#include "stm32f10x.h"
#include "delay.h"
//////////////////////////////////////////////////////////////////////////////////
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK Mini STM32开发板
//PWM  驱动代码
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2010/12/03
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 正点原子 2009-2019
//All rights reserved
//////////////////////////////////////////////////////////////////////////////////
int16_t MOTO1_PWM = 0;
int16_t MOTO2_PWM = 0;
int16_t MOTO3_PWM = 0;
int16_t MOTO4_PWM = 0;

extern int16_t cNd1, cNd2, cNd3, cNd4;
extern float roll, pitch, yaw; 		//欧拉角,DMP硬解得到的角度,roll -180~180 pitch -90~90 yaw -180~180
extern short gyrox, gyroy, gyroz;	//陀螺仪原始数据-32768~32767
extern short gyrox_chushi,gyroy_chushi,gyroz_chushi;//陀螺仪最开始放在地面时候的一些数据

//roll:-180~180 pitch:-90:90 yaw:-180:180
//omegaroll:omegapitch:omegayaw:-32768:32767

//对于x,y轴的角度最多允许控制300油门,对于z轴的角度最多允许控制0油门
//static float kp_theta_x_c=300,kp_theta_y_c=300,kp_theta_z_c=0;

//对于x,y的角速度最多允许控制150油门,对于z轴的角速度最多允许控制25油门
//static float kp_omega_x_c=150,kp_omega_y_c=150,kp_omega_z_c=25;

//coefficient Proportion for Angle,coefficient Proportion for palstance,aka angular velocity
/*static float kp_theta_x = 1.6666667, kp_theta_y = 3.3333333, kp_theta_z = 0;
static float kp_omega_x = 0.0045778, kp_omega_y = 0.0045778, kp_omega_z = 0.0007629;*/

//PD控制器调试上端
//各种调试，PD控制器
//角度预估在15度以内,故让角度控制的油门为15*4=60
//角速度暂时没有估计,暂时估计在10000以内
//65536/2*0.004=131.072
//10000*0.04=400
float KP_THETA_X = 4.0, KP_THETA_Y = 4.0, KP_THETA_Z = 0;//常量
float KP_OMEGA_X = 0.04, KP_OMEGA_Y = 0.04, KP_OMEGA_Z = 0.02;//常量
float kp_theta_x = 4.0, kp_theta_y = 4.0, kp_theta_z = 0;//变量
float kp_omega_x = 0.04, kp_omega_y = 0.04, kp_omega_z = 0.02;//变量
//PD控制器调试下端

extern float rjz,pjz,yjz;//将cNd1等数据分别转换为roll,pitch,yaw方向的纠正量，以便示波观察

extern float gyrox_filter[7],gyroy_filter[7],gyroz_filter[7];//用来存放陀螺仪角速度的三值滤波器暂存数据,第四位用来存放陀螺仪三值滤波的返回值，即经过滤波后的值
extern short gyro_jishu;//滤波计数,一般到3回0,即永远不能到3

extern u8 roll_in_flag;//绕X轴有有效遥控信号输入则为1，否则为0
extern u8 pitch_in_flag;
extern u8 yaw_in_flag;

extern float roll_err, pitch_err, yaw_err;//误差值,roll_err=roll-desroll,其中desroll为遥控信号线性映射的角度值
extern float desroll, despitch, desyaw;//定义想要的横滚，俯仰，偏航，油门

extern short gyrox_out,gyroy_out,gyroz_out;

//此函数直接控制电机.核心一
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

//此函数输入遥控油门pwm信号,内部会结合自动控制量并调用最底层油门函数.核心二
void Moto_Throttle(int16_t desthrottle)
{
    int16_t d1, d2, d3, d4;
    d1 = desthrottle+cNd1; //            CW3     1CCW	   / \				 
    d2 = desthrottle+cNd2; //  俯视图        * *          / | \ X轴      	  Y轴
    d3 = desthrottle+cNd3; //                 *             |                <=======
    d4 = desthrottle+cNd4; //      	     CCW2    4CW        |

    Moto_PwmRflash(d1, d2, d3, d4);//此函数是最终改变油门的函数,核心一调用三
}

//没有这个限制函数简直要爆炸哦,暂时限制到300吧
int16_t xianzhi(int16_t a)
{
	if(a>300)
	{
		return 300;
	}
	
	if(a<-300)
	{
		return -300;
	}
	
	return a;
}

float sfabs(float a)//单精度绝对值函数
{
	if(a>0.0000000)
	{
		return a;
	}
		
	return -a;
}

float chuli(float a)//当a小于1时，返回1，当a大于等于1时，返回1/a
{
	if(a<1)
	{
		return 1;
	}
	return 1/a;
}

//此函数将角度和角速度值处理后转换为自动控制量,此些量会传递给核心二使用.核心三
void cyberNation(void)
{
	kp_omega_x=KP_OMEGA_X*chuli(sfabs(roll_err));//误差越多，放弃越多的角速度锁,1/15=0.06666666666666
	kp_omega_y=KP_OMEGA_Y*chuli(sfabs(pitch_err));
	
	cNd1 = +roll_err * kp_theta_x + pitch_err * kp_theta_y + yaw_err * kp_theta_z + gyrox_out * kp_omega_x + gyroy_out * kp_omega_y + gyroz_out * kp_omega_z;
    cNd2 = -roll_err * kp_theta_x - pitch_err * kp_theta_y + yaw_err * kp_theta_z - gyrox_out * kp_omega_x - gyroy_out * kp_omega_y + gyroz_out * kp_omega_z;
    cNd3 = -roll_err * kp_theta_x + pitch_err * kp_theta_y - yaw_err * kp_theta_z - gyrox_out * kp_omega_x + gyroy_out * kp_omega_y - gyroz_out * kp_omega_z;
    cNd4 = +roll_err * kp_theta_x - pitch_err * kp_theta_y - yaw_err * kp_theta_z + gyrox_out * kp_omega_x - gyroy_out * kp_omega_y - gyroz_out * kp_omega_z;
	
	cNd1=xianzhi(cNd1);
	cNd2=xianzhi(cNd2);
	cNd3=xianzhi(cNd3);
	cNd4=xianzhi(cNd4);
}

//此函数将cNd1~cNd4电机单个纠正量转化为roll,pitch,yaw方向的纠正量
void cN2rpy(void)
{
	rjz=-cNd1+cNd2+cNd3-cNd4;
	pjz=-cNd1+cNd2-cNd3+cNd4;
	yjz=-cNd1-cNd2+cNd3+cNd4;
}

#define Filter_Num 6//六值平均
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


