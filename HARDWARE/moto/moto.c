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
//���ֵ��ԣ�PD������
//�Ƕ�Ԥ����15������,���ýǶȿ��Ƶ�����Ϊ15*4=60
//���ٶ���ʱû�й���,��ʱ������10000����
//65536/2*0.004=131.072
//10000*0.04=400
float KP_THETA_X = 4.0, KP_THETA_Y = 4.0, KP_THETA_Z = 0;//����
float KP_OMEGA_X = 0.04, KP_OMEGA_Y = 0.04, KP_OMEGA_Z = 0.02;//����
float kp_theta_x = 4.0, kp_theta_y = 4.0, kp_theta_z = 0;//����
float kp_omega_x = 0.04, kp_omega_y = 0.04, kp_omega_z = 0.02;//����
//PD�����������¶�

extern float rjz,pjz,yjz;//��cNd1�����ݷֱ�ת��Ϊroll,pitch,yaw����ľ��������Ա�ʾ���۲�

extern float gyrox_filter[7],gyroy_filter[7],gyroz_filter[7];//������������ǽ��ٶȵ���ֵ�˲����ݴ�����,����λ���������������ֵ�˲��ķ���ֵ���������˲����ֵ
extern short gyro_jishu;//�˲�����,һ�㵽3��0,����Զ���ܵ�3

extern u8 roll_in_flag;//��X������Чң���ź�������Ϊ1������Ϊ0
extern u8 pitch_in_flag;
extern u8 yaw_in_flag;

extern float roll_err, pitch_err, yaw_err;//���ֵ,roll_err=roll-desroll,����desrollΪң���ź�����ӳ��ĽǶ�ֵ
extern float desroll, despitch, desyaw;//������Ҫ�ĺ����������ƫ��������

extern short gyrox_out,gyroy_out,gyroz_out;

//�˺���ֱ�ӿ��Ƶ��.����һ
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

//�˺�������ң������pwm�ź�,�ڲ������Զ���������������ײ����ź���.���Ķ�
void Moto_Throttle(int16_t desthrottle)
{
    int16_t d1, d2, d3, d4;
    d1 = desthrottle+cNd1; //            CW3     1CCW	   / \				 
    d2 = desthrottle+cNd2; //  ����ͼ        * *          / | \ X��      	  Y��
    d3 = desthrottle+cNd3; //                 *             |                <=======
    d4 = desthrottle+cNd4; //      	     CCW2    4CW        |

    Moto_PwmRflash(d1, d2, d3, d4);//�˺��������ոı����ŵĺ���,����һ������
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
	
	return a;
}

float sfabs(float a)//�����Ⱦ���ֵ����
{
	if(a>0.0000000)
	{
		return a;
	}
		
	return -a;
}

float chuli(float a)//��aС��1ʱ������1����a���ڵ���1ʱ������1/a
{
	if(a<1)
	{
		return 1;
	}
	return 1/a;
}

//�˺������ǶȺͽ��ٶ�ֵ�����ת��Ϊ�Զ�������,��Щ���ᴫ�ݸ����Ķ�ʹ��.������
void cyberNation(void)
{
	kp_omega_x=KP_OMEGA_X*chuli(sfabs(roll_err));//���Խ�࣬����Խ��Ľ��ٶ���,1/15=0.06666666666666
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

//�˺�����cNd1~cNd4�������������ת��Ϊroll,pitch,yaw����ľ�����
void cN2rpy(void)
{
	rjz=-cNd1+cNd2+cNd3-cNd4;
	pjz=-cNd1+cNd2-cNd3+cNd4;
	yjz=-cNd1-cNd2+cNd3+cNd4;
}

#define Filter_Num 6//��ֵƽ��
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


