#include "moto.h"
#include "stm32f10x.h"
#include "delay.h"
#include "math.h"

int16_t MOTO1_PWM = 0;
int16_t MOTO2_PWM = 0;
int16_t MOTO3_PWM = 0;
int16_t MOTO4_PWM = 0;

extern int16_t cNd1_theta, cNd2_theta, cNd3_theta, cNd4_theta;
extern int16_t cNd1_omega, cNd2_omega, cNd3_omega, cNd4_omega;
extern int16_t cNd1_alpha, cNd2_alpha, cNd3_alpha, cNd4_alpha;

extern float roll, pitch, yaw; 		//roll -180~180 pitch -90~90 yaw -180~180
extern short gyrox, gyroy, gyroz;	//-32768~32767
extern short gyrox_chushi, gyroy_chushi, gyroz_chushi;

extern float ACC_IIR_FACTOR;

extern int16_t d1, d2, d3, d4;

extern float alphax, alphay, alphaz;

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
//angle by degree
//palstance in 32768，±2000，也就是相当于输出角度=原始数据乘以0.0610370,那么将这个数放到KP_OMEGA也行
float KP_THETA_X = 3, KP_THETA_Y = 3, KP_THETA_Z = 3;//常量
float kp_theta_x = 3, kp_theta_y = 3, kp_theta_z = 3;//变量
float KP_OMEGA_X = 0.6 * 0.0610370, KP_OMEGA_Y = 0.5 * 0.0610370, KP_OMEGA_Z = 0.7 * 0.0610370; //常量
float kp_omega_x = 0.6 * 0.0610370, kp_omega_y = 0.5 * 0.0610370, kp_omega_z = 0.7 * 0.0610370; //变量
float KP_ALPHA_X = 0.03, KP_ALPHA_Y = 0.03, KP_ALPHA_Z = 0;
float kp_alpha_x = 0.03, kp_alpha_y = 0.03, kp_alpha_z = 0;
//6*130=780;
//0.5*1000=500;
//consideration of mass of 1837g for MATLAB theory
//velocity and accelerate in centimeter
//velocity between 0~50,10Hz,5
//accelerate between 0~100,10Hz,10
float KP_VEL_Z = 0.5;
float kp_vel_z = 0.5;
float KP_ACC_Z = 0.5 * 0.05978;
float kp_acc_z = 0.5 * 0.05978;
//PD controller bottom

extern float roll_err, pitch_err, yaw_err;
extern float desroll, despitch, desyaw;

extern short gyrox_out, gyroy_out, gyroz_out;
extern short aacx, aacy, aacz;     
extern short accz_out;

extern float Ahd;
extern u16 channel3_in;

extern float Scd;

extern float gyro_R[3];
extern float gyro_Q[3];
extern float gyro_K[3];
extern float gyro_X_hat[3];
extern float gyro_X_hat_minus[3];
extern float gyro_P[3];
extern short gyro[3];
extern short gyro_chushi[3];

extern float alphax_out;
extern float alphay_out;
extern float alphaz_out;

extern short aacz_chushi;

extern float MS5611_Altitude;
extern float Altitude_out;

//just constrain from right
int16_t Constrain_up(int16_t throttle, int16_t max)
{
    if(throttle > max)
    {
        return max;
    }
    return throttle;
}

//constrain both side
int16_t Constrain(int16_t throttle, int16_t max, int16_t min)
{
    if((min < throttle) && (throttle < max))
    {
        return throttle;
    }
    else
    {
        if(max <= throttle)
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

    d1 = Constrain_up(desthrottle, 1780) + Constrain(cNd1_theta, 50, -50) + Constrain(cNd1_omega, 300, -300) + Constrain(cNd1_alpha, 30, -30) + Constrain((int16_t)Scd, 15, -15)+ Constrain((int16_t)Ahd, 5, -5) ; //              CW3     1CCW	   / \				 
    d2 = Constrain_up(desthrottle, 1780) + Constrain(cNd2_theta, 50, -50) + Constrain(cNd2_omega, 300, -300) + Constrain(cNd2_alpha, 30, -30) + Constrain((int16_t)Scd, 15, -15)+ Constrain((int16_t)Ahd, 5, -5) ; //  俯视图          * *           / | \ X轴      	  Y轴
    d3 = Constrain_up(desthrottle, 1780) + Constrain(cNd3_theta, 50, -50) + Constrain(cNd3_omega, 300, -300) + Constrain(cNd3_alpha, 30, -30) + Constrain((int16_t)Scd, 15, -15)+ Constrain((int16_t)Ahd, 5, -5) ; //                   *              |                <=======
    d4 = Constrain_up(desthrottle, 1780) + Constrain(cNd4_theta, 50, -50) + Constrain(cNd4_omega, 300, -300) + Constrain(cNd4_alpha, 30, -30) + Constrain((int16_t)Scd, 15, -15)+ Constrain((int16_t)Ahd, 5, -5) ; //      	    CCW2    4CW         |

    Moto_PwmRflash(d1, d2, d3, d4);//core 1 called in place 3
}

//cybernation offset.core 3
void cyberNation_theta(void)
{
    cNd1_theta = +roll_err * kp_theta_x + pitch_err * kp_theta_y + yaw_err * kp_theta_z ;
    cNd2_theta = -roll_err * kp_theta_x - pitch_err * kp_theta_y + yaw_err * kp_theta_z;
    cNd3_theta = -roll_err * kp_theta_x + pitch_err * kp_theta_y - yaw_err * kp_theta_z;
    cNd4_theta = +roll_err * kp_theta_x - pitch_err * kp_theta_y - yaw_err * kp_theta_z;
}

void cyberNation_omega(void)
{
    cNd1_omega = +gyrox_out * kp_omega_x + gyroy_out * kp_omega_y + gyroz_out * kp_omega_z;
    cNd2_omega = -gyrox_out * kp_omega_x - gyroy_out * kp_omega_y + gyroz_out * kp_omega_z;
    cNd3_omega = -gyrox_out * kp_omega_x + gyroy_out * kp_omega_y - gyroz_out * kp_omega_z;
    cNd4_omega = +gyrox_out * kp_omega_x - gyroy_out * kp_omega_y - gyroz_out * kp_omega_z;
}

void cyberNation_alpha(void)
{
    cNd1_alpha = +(-cNd1_omega+cNd2_omega+cNd3_omega-cNd4_omega-cNd1_theta+cNd2_theta+cNd3_theta-cNd4_theta) * kp_alpha_x + (-cNd1_omega+cNd2_omega-cNd3_omega+cNd4_omega-cNd1_theta+cNd2_theta-cNd3_theta+cNd4_theta) * kp_alpha_y;
    cNd2_alpha = -(-cNd1_omega+cNd2_omega+cNd3_omega-cNd4_omega-cNd1_theta+cNd2_theta+cNd3_theta-cNd4_theta) * kp_alpha_x - (-cNd1_omega+cNd2_omega-cNd3_omega+cNd4_omega-cNd1_theta+cNd2_theta-cNd3_theta+cNd4_theta) * kp_alpha_y;
    cNd3_alpha = -(-cNd1_omega+cNd2_omega+cNd3_omega-cNd4_omega-cNd1_theta+cNd2_theta+cNd3_theta-cNd4_theta) * kp_alpha_x + (-cNd1_omega+cNd2_omega-cNd3_omega+cNd4_omega-cNd1_theta+cNd2_theta-cNd3_theta+cNd4_theta) * kp_alpha_y;
    cNd4_alpha = +(-cNd1_omega+cNd2_omega+cNd3_omega-cNd4_omega-cNd1_theta+cNd2_theta+cNd3_theta-cNd4_theta) * kp_alpha_x - (-cNd1_omega+cNd2_omega-cNd3_omega+cNd4_omega-cNd1_theta+cNd2_theta-cNd3_theta+cNd4_theta) * kp_alpha_y;
}

#define Filter_Num 6//sliding window with 6 values
void Gyro_filter(void)
{
    static float Filter_x[Filter_Num], Filter_y[Filter_Num], Filter_z[Filter_Num];
    static uint8_t Filter_count;
    float Filter_sum_x = 0, Filter_sum_y = 0, Filter_sum_z = 0;
    uint8_t i;

    Filter_x[Filter_count] = gyro[0] - gyro_chushi[0];
    Filter_y[Filter_count] = gyro[1] - gyro_chushi[1];
    Filter_z[Filter_count] = gyro[2] - gyro_chushi[2];

    for(i = 0; i < Filter_Num; i++)
    {
        Filter_sum_x += Filter_x[i];
        Filter_sum_y += Filter_y[i];
        Filter_sum_z += Filter_z[i];
    }

    gyrox_out = Filter_sum_x / Filter_Num;
    gyroy_out = Filter_sum_y / Filter_Num;
    gyroz_out = Filter_sum_z / Filter_Num;

    Filter_count++;

    if(Filter_count == Filter_Num)
    {
        Filter_count = 0;
    }
}

#define Filter_Num2 6//sliding window with <input> values
void Accz_filter(void)
{
    static float Filter_accz[Filter_Num2];
    static uint8_t Filter_count2;
    float Filter_sum_accz = 0;
    uint8_t i;

    Filter_accz[Filter_count2] = aacz-aacz_chushi;

    for(i = 0; i < Filter_Num2; i++)
    {
        Filter_sum_accz += Filter_accz[i];
    }

    accz_out = Filter_sum_accz / Filter_Num2;

    Filter_count2++;

    if(Filter_count2 == Filter_Num2)
    {
        Filter_count2 = 0;
    }
}

#define Filter_Num3 1//sliding window with 6 values
void Altitude_filter(void)
{
    static float Filter_Altitude[Filter_Num3];//if use integer,we will lose the value
    static uint8_t Filter_count3;
    float Filter_sum_Altitude=0;
    uint8_t i;

    Filter_Altitude[Filter_count3] = MS5611_Altitude;

    for(i = 0; i < Filter_Num3; i++)
    {
        Filter_sum_Altitude += Filter_Altitude[i];
    }

    Altitude_out = Filter_sum_Altitude / Filter_Num3;

    Filter_count3++;

    if(Filter_count3 == Filter_Num3)
    {
        Filter_count3 = 0;
    }
}



/******************************************************************************
函数原型：	void Calculate_FilteringCoefficient(float Time, float Cut_Off)
功    能：	iir低通滤波参数计算
*******************************************************************************/
void Calculate_FilteringCoefficient(float Time, float Cut_Off)
{
    ACC_IIR_FACTOR = Time / ( Time + 1 / (2.0f * 3.1415927 * Cut_Off) );
}

/******************************************************************************
函数原型：	void ACC_IIR_Filter(struct _acc *Acc_in,struct _acc *Acc_out)
功    能：	iir低通滤波
*******************************************************************************/
void ACC_IIR_Filter(void)
{
    accz_out = accz_out + ACC_IIR_FACTOR * (aacz - accz_out);
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
    accz_X_hat_minus = accz_X_hat;
    accz_P = accz_P + accz_Q;

    //predict update
    accz_K = accz_P / (accz_P + accz_R);
    accz_X_hat = accz_X_hat_minus + accz_K * (aacz- accz_X_hat_minus);
    accz_P = (1 - accz_K) * accz_P;
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
    accy_X_hat_minus = accy_X_hat;
    accy_P = accy_P + accy_Q;

    //predict update
    accy_K = accy_P / (accy_P + accy_R);
    accy_X_hat = accy_X_hat_minus + accy_K * (aacy - accy_X_hat_minus);
    accy_P = (1 - accy_K) * accy_P;
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
    accx_X_hat_minus = accx_X_hat;
    accx_P = accx_P + accx_Q;

    //predict update
    accx_K = accx_P / (accx_P + accx_R);
    accx_X_hat = accx_X_hat_minus + accx_K * (aacx - accx_X_hat_minus);
    accx_P = (1 - accx_K) * accx_P;
}


extern float acc_Climb_out;
extern short aacz_chushi;

void Altitude_hold_update(void)
{
    if(desroll<0.5&&despitch<0.5)
    {
        Ahd = -kp_vel_z * acc_Climb_out - kp_acc_z *accz_out;
    }
    Ahd = 0; 
}

//degree to radian by multiplier 0.0174533
void Sink_compensation(void)
{
    Scd = (channel3_in - 1000) / (cosf(desroll * 0.0174533) * cosf(despitch * 0.0174533)) - (channel3_in - 1000); //cosine between EarthFrame_Z with BodyFrame_Z
}

void Kalman_filter_gyro(void)
{
    u8 i = 0;
    for(i = 0; i < 3; i++)
    {
        //time update
        gyro_X_hat_minus[i] = gyro_X_hat[i];
        gyro_P[i] = gyro_P[i] + gyro_Q[i];

        //predict update
        gyro_K[i] = gyro_P[i] / (gyro_P[i] + gyro_R[i]);
        gyro_X_hat[i] = gyro_X_hat_minus[i] + gyro_K[i] * (gyro[i] - gyro_chushi[i] - gyro_X_hat_minus[i]);
        gyro_P[i] = (1 - gyro_K[i]) * gyro_P[i];
    }
}


