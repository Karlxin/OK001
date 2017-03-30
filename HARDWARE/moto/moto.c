#include "moto.h"
#include "stm32f10x.h"
#include "delay.h"
//////////////////////////////////////////////////////////////////////////////////
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK Mini STM32������
//PWM  ��������
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2010/12/03
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ����ԭ�� 2009-2019
//All rights reserved
//////////////////////////////////////////////////////////////////////////////////
int16_t MOTO1_PWM = 0;
int16_t MOTO2_PWM = 0;
int16_t MOTO3_PWM = 0;
int16_t MOTO4_PWM = 0;

extern int16_t cNd1, cNd2, cNd3, cNd4;
extern float roll, pitch, yaw; 		//ŷ����,DMPӲ��õ��ĽǶ�,roll -180~180 pitch -90~90 yaw -180~180
extern short gyrox, gyroy, gyroz;	//������ԭʼ����-32768~32767
extern short gyrox_chushi,gyroy_chushi,gyroz_chushi;//�������ʼ���ڵ���ʱ���һЩ����

//roll:-180~180 pitch:-90:90 yaw:-180:180
//omegaroll:omegapitch:omegayaw:-32768:32767

//����x,y��ĽǶ�����������300����,����z��ĽǶ�����������0����
//static float kp_theta_x_c=300,kp_theta_y_c=300,kp_theta_z_c=0;

//����x,y�Ľ��ٶ�����������150����,����z��Ľ��ٶ�����������25����
//static float kp_omega_x_c=150,kp_omega_y_c=150,kp_omega_z_c=25;

//coefficient Proportion for Angle,coefficient Proportion for palstance,aka angular velocity
/*static float kp_theta_x = 1.6666667, kp_theta_y = 3.3333333, kp_theta_z = 0;
static float kp_omega_x = 0.0045778, kp_omega_y = 0.0045778, kp_omega_z = 0.0007629;*/

//PD�����������϶�
//���ֵ��ԣ�PD��������x����0��y����0��z����300,����3ָ���ķ���Ϊ�����ٶ�ֱֻ�ӿ���z
//�Ƕ�Ԥ����30�����ڣ�ֱ����x��y����300������,Ԥ��rollpitch��30���ڣ�yaw���ٶ�������10000
float kp_theta_x = 0, kp_theta_y = 0, kp_theta_z = 0;
float kp_omega_x = 0.08, kp_omega_y = 0.08, kp_omega_z = 0.05;
//PD�����������¶�

extern float rjz,pjz,yjz;//��cNd1�����ݷֱ�ת��Ϊroll,pitch,yaw����ľ��������Ա�ʾ���۲�

extern float gyrox_filter[7],gyroy_filter[7],gyroz_filter[7];//������������ǽ��ٶȵ���ֵ�˲����ݴ�����,����λ���������������ֵ�˲��ķ���ֵ���������˲����ֵ
extern short gyro_jishu;//�˲�����,һ�㵽3��0,����Զ���ܵ�3

extern u8 roll_in_flag;//��X������Чң���ź�������Ϊ1������Ϊ0
extern u8 pitch_in_flag;
extern u8 yaw_in_flag;

//�˺���ֱ�ӿ��Ƶ��
void Moto_PwmRflash(int16_t MOTO1_PWM, int16_t MOTO2_PWM, int16_t MOTO3_PWM, int16_t MOTO4_PWM)
{

    if(MOTO1_PWM > Moto_PwmMax)	MOTO1_PWM = Moto_PwmMax;//������ű���
    if(MOTO2_PWM > Moto_PwmMax)	MOTO2_PWM = Moto_PwmMax;
    if(MOTO3_PWM > Moto_PwmMax)	MOTO3_PWM = Moto_PwmMax;
    if(MOTO4_PWM > Moto_PwmMax)	MOTO4_PWM = Moto_PwmMax;
    if(MOTO1_PWM < 1000)	MOTO1_PWM = 1000;//��С���ű���
    if(MOTO2_PWM < 1000)	MOTO2_PWM = 1000;
    if(MOTO3_PWM < 1000)	MOTO3_PWM = 1000;
    if(MOTO4_PWM < 1000)	MOTO4_PWM = 1000;

    TIM3->CCR1 = MOTO1_PWM;
    TIM3->CCR2 = MOTO2_PWM;
    TIM3->CCR3 = MOTO3_PWM;
    TIM3->CCR4 = MOTO4_PWM;
}

//�˺�������Ҫ�ĽǶ�ӳ���������������ź���
void Moto_RPY(int16_t desroll, int16_t despitch, int16_t desthrottle, int16_t desyaw)
{
    int16_t d1, d2, d3, d4;
    d1 = desthrottle + desroll - despitch - desyaw+cNd1; //            CW3     1CCW	     / \				 
    d2 = desthrottle - desroll + despitch - desyaw+cNd2; //  ����ͼ        * *          / | \ X��      	  Y��
    d3 = desthrottle - desroll - despitch + desyaw+cNd3; //                 *             |                <=======
    d4 = desthrottle + desroll + despitch + desyaw+cNd4; //      	   CCW2    4CW        |

    Moto_PwmRflash(d1, d2, d3, d4);//�˺��������ոı����ŵĺ���
}

//û��������ƺ�����ֱҪ��ըŶ,��ʱ���Ƶ�300��
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

//�˺������ǶȺͽ��ٶ�ֵ�����ת��Ϊ�Զ�������!!!���������ڵ����ˣ������Һ��˷ܰ�������
void cyberNation(void)
{
	/*
    cNd1 = +roll * kp_theta_x + pitch * kp_theta_y + yaw * kp_theta_z + (gyrox-gyrox_chushi) * kp_omega_x + (gyroy-gyroy_chushi) * kp_omega_y + (gyroz-gyroz_chushi) * kp_omega_z;
    cNd2 = -roll * kp_theta_x - pitch * kp_theta_y + yaw * kp_theta_z - (gyrox-gyrox_chushi) * kp_omega_x - (gyroy-gyroy_chushi) * kp_omega_y + (gyroz-gyroz_chushi) * kp_omega_z;
    cNd3 = -roll * kp_theta_x + pitch * kp_theta_y - yaw * kp_theta_z - (gyrox-gyrox_chushi) * kp_omega_x + (gyroy-gyroy_chushi) * kp_omega_y - (gyroz-gyroz_chushi) * kp_omega_z;
    cNd4 = +roll * kp_theta_x - pitch * kp_theta_y - yaw * kp_theta_z + (gyrox-gyrox_chushi) * kp_omega_x - (gyroy-gyroy_chushi) * kp_omega_y - (gyroz-gyroz_chushi) * kp_omega_z;
	*/
	
	kp_omega_x = 0.08;//X����ٶ�Pֵ��ԭʼֵ
	if(roll_in_flag)//���X����ң������
	{
		kp_omega_x=0;//�Զ����Ʒ�����X��Ŀ���
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

//�˺�����cNd1~cNd4�������������ת��Ϊroll,pitch,yaw����ľ�����
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


