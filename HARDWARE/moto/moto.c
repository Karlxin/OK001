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
//各种调试，PD控制器，x控制0，y控制0，z控制300,尝试3指明的方向为，角速度只直接控制z
//角度预估在30度以内，直接让x和y控制300的油门,预估rollpitch在30度内，yaw角速度在正负10000
float kp_theta_x = 0, kp_theta_y = 0, kp_theta_z = 0;
float kp_omega_x = 0.08, kp_omega_y = 0.08, kp_omega_z = 0.05;
//PD控制器调试下端

extern float rjz,pjz,yjz;//将cNd1等数据分别转换为roll,pitch,yaw方向的纠正量，以便示波观察

extern float gyrox_filter[7],gyroy_filter[7],gyroz_filter[7];//用来存放陀螺仪角速度的三值滤波器暂存数据,第四位用来存放陀螺仪三值滤波的返回值，即经过滤波后的值
extern short gyro_jishu;//滤波计数,一般到3回0,即永远不能到3

extern u8 roll_in_flag;//绕X轴有有效遥控信号输入则为1，否则为0
extern u8 pitch_in_flag;
extern u8 yaw_in_flag;

//此函数直接控制电机
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

//此函数将想要的角度映射成油门输入给油门函数
void Moto_RPY(int16_t desroll, int16_t despitch, int16_t desthrottle, int16_t desyaw)
{
    int16_t d1, d2, d3, d4;
    d1 = desthrottle + desroll - despitch - desyaw+cNd1; //            CW3     1CCW	     / \				 
    d2 = desthrottle - desroll + despitch - desyaw+cNd2; //  俯视图        * *          / | \ X轴      	  Y轴
    d3 = desthrottle - desroll - despitch + desyaw+cNd3; //                 *             |                <=======
    d4 = desthrottle + desroll + despitch + desyaw+cNd4; //      	   CCW2    4CW        |

    Moto_PwmRflash(d1, d2, d3, d4);//此函数是最终改变油门的函数
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
}

//此函数将角度和角速度值处理后转换为自动控制量!!!哈哈哈终于到这了！！！我好兴奋啊！！！
void cyberNation(void)
{
	/*
    cNd1 = +roll * kp_theta_x + pitch * kp_theta_y + yaw * kp_theta_z + (gyrox-gyrox_chushi) * kp_omega_x + (gyroy-gyroy_chushi) * kp_omega_y + (gyroz-gyroz_chushi) * kp_omega_z;
    cNd2 = -roll * kp_theta_x - pitch * kp_theta_y + yaw * kp_theta_z - (gyrox-gyrox_chushi) * kp_omega_x - (gyroy-gyroy_chushi) * kp_omega_y + (gyroz-gyroz_chushi) * kp_omega_z;
    cNd3 = -roll * kp_theta_x + pitch * kp_theta_y - yaw * kp_theta_z - (gyrox-gyrox_chushi) * kp_omega_x + (gyroy-gyroy_chushi) * kp_omega_y - (gyroz-gyroz_chushi) * kp_omega_z;
    cNd4 = +roll * kp_theta_x - pitch * kp_theta_y - yaw * kp_theta_z + (gyrox-gyrox_chushi) * kp_omega_x - (gyroy-gyroy_chushi) * kp_omega_y - (gyroz-gyroz_chushi) * kp_omega_z;
	*/
	
	kp_omega_x = 0.08;//X轴角速度P值的原始值
	if(roll_in_flag)//如果X轴有遥控输入
	{
		kp_omega_x=0;//自动控制放弃对X轴的控制
	}
	
	kp_omega_y = 0.08;
	if(pitch_in_flag)
	{
		kp_omega_y=0;
	}
	kp_omega_z = 0.05;
	
	if(yaw_in_flag)
	{
		kp_omega_z=0;
	}
	
	cNd1 = +roll * kp_theta_x + pitch * kp_theta_y + yaw * kp_theta_z + (gyrox_filter[6]-gyrox_chushi) * kp_omega_x + (gyroy_filter[6]-gyroy_chushi) * kp_omega_y + (gyroz_filter[6]-gyroz_chushi) * kp_omega_z;
    cNd2 = -roll * kp_theta_x - pitch * kp_theta_y + yaw * kp_theta_z - (gyrox_filter[6]-gyrox_chushi) * kp_omega_x - (gyroy_filter[6]-gyroy_chushi) * kp_omega_y + (gyroz_filter[6]-gyroz_chushi) * kp_omega_z;
    cNd3 = -roll * kp_theta_x + pitch * kp_theta_y - yaw * kp_theta_z - (gyrox_filter[6]-gyrox_chushi) * kp_omega_x + (gyroy_filter[6]-gyroy_chushi) * kp_omega_y - (gyroz_filter[6]-gyroz_chushi) * kp_omega_z;
    cNd4 = +roll * kp_theta_x - pitch * kp_theta_y - yaw * kp_theta_z + (gyrox_filter[6]-gyrox_chushi) * kp_omega_x - (gyroy_filter[6]-gyroy_chushi) * kp_omega_y - (gyroz_filter[6]-gyroz_chushi) * kp_omega_z;
	
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

void filter_threeValue(void)
{
	int i;
	gyrox_filter[gyro_jishu]=gyrox;
	gyroy_filter[gyro_jishu]=gyroy;
	gyroz_filter[gyro_jishu]=gyroz;
	
	
	if(gyro_jishu==5)
	{
		gyro_jishu=0;
	}
	else
	{
		gyro_jishu++;
	}
	
	gyrox_filter[6]=0;
	gyroy_filter[6]=0;
	gyroz_filter[6]=0;
	
	for(i=0;i<6;i++)
	{
	gyrox_filter[6]+=gyrox_filter[i];
	gyroy_filter[6]+=gyroy_filter[i];
	gyroz_filter[6]+=gyroz_filter[i];
	}
	
	gyrox_filter[6]*=0.1666667;
	gyroy_filter[6]*=0.1666667;
	gyroz_filter[6]*=0.1666667;
}


