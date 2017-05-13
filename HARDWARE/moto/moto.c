/*--------------------------------version information top-------------------------------------------
Started at 2016
Created by Karlxin(410824290@qq.com)
Github:https://github.com/Karlxin/OK001.git
OpenKarlCopter
























--------------------------------version information bottom-------------------------------------------*/

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

extern float alpha[3];

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
float KP_THETA_X = 2.6, KP_THETA_Y = 2.6, KP_THETA_Z = 2.6;//常量
float kp_theta_x = 2.6, kp_theta_y = 2.6, kp_theta_z = 2.6;//变量
float KP_OMEGA_X = 0.52 * 0.0610370, KP_OMEGA_Y = 0.52 * 0.0610370, KP_OMEGA_Z = 0.6 * 0.0610370; //常量
float kp_omega_x = 0.52 * 0.0610370, kp_omega_y = 0.52 * 0.0610370, kp_omega_z = 0.6 * 0.0610370; //变量
float KP_ALPHA_X = 0.03* 0.0610370, KP_ALPHA_Y = 0.03* 0.0610370, KP_ALPHA_Z = 0;
float kp_alpha_x = 0.03* 0.0610370, kp_alpha_y = 0.03* 0.0610370, kp_alpha_z = 0;
//6*130=780;
//0.5*1000=500;
//consideration of mass of 1837g for MATLAB theory
//velocity and accelerate in centimeter
//velocity between 0~50,10Hz,5
//accelerate between 0~100,10Hz,10
float KP_VEL_Z = 0.36;
float kp_vel_z = 0.36;
float KP_ACC_Z = 0.5 * 0.05978;
float kp_acc_z = 0.5 * 0.05978;
//PD controller bottom

extern float roll_err, pitch_err, yaw_err;
extern float desroll, despitch, desyaw;

extern short gyro_out[3];
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

extern float alpha_out[3];

extern short aacz_chushi;

extern float MS5611_Altitude;
extern float Altitude_out;

extern short gyro_cybernation[3];

extern int16_t cNd1_memory,cNd2_memory,cNd3_memory,cNd4_memory;
extern int16_t cNd1_memory_temp,cNd2_memory_temp,cNd3_memory_temp,cNd4_memory_temp;

/*******************************************************************************
	* @Name				Constrain_up
	* @Description		constrain value from right
	* @Input			throttle,max
	* @Use    			None
	* @Output			None
	* @Return			throttle;max
*******************************************************************************/
int16_t Constrain_up(
int16_t throttle,
int16_t max)
{
    if(throttle > max)
    {
        return max;
    }
    return throttle;
}


/*******************************************************************************
	* @Name				Constrain
	* @Description		constrain both side
	* @Input			throttle,max,min
	* @Use    			None
	* @Output			None
	* @Return			throttle;max;min
*******************************************************************************/
int16_t Constrain(
int16_t throttle,
int16_t max,
int16_t min)
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

/*******************************************************************************
	* @Name				Moto_PwmRflash
	* @Description		the last function to control motor
	* @Input			MOTO1_PWM,MOTO2_PWM,MOTO3_PWM,MOTO4_PWM
	* @Use    			Moto_PwmMax
	* @Output			None
	* @Return			None
*******************************************************************************/
void Moto_PwmRflash(
int16_t MOTO1_PWM,
int16_t MOTO2_PWM,
int16_t MOTO3_PWM,
int16_t MOTO4_PWM)
{
	//directive controller motor ,core 1
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

/*******************************************************************************
	* @Name				Moto_Throttle
	* @Description		Control motor
	* @Input			desthrottle
	* @Use    			(arguments too much)
	* @Output			d1,d2,d3,d4
	* @Return			None
*******************************************************************************/
void Moto_Throttle(int16_t desthrottle)
{
	//core 2
    d1 = Constrain_up(desthrottle, 1500) + Constrain(cNd1_alpha, 15, -15) + Constrain(cNd1_omega, 130, -130) + Constrain(cNd1_theta, 60, -60)   + Constrain((int16_t)Scd, 23, -23) + Constrain((int16_t)Ahd, 10, -10) + Constrain((int16_t)cNd1_memory, 30, -30) ;
    d2 = Constrain_up(desthrottle, 1500) + Constrain(cNd2_alpha, 15, -15) + Constrain(cNd2_omega, 130, -130) + Constrain(cNd2_theta, 60, -60)   + Constrain((int16_t)Scd, 23, -23) + Constrain((int16_t)Ahd, 10, -10) + Constrain((int16_t)cNd2_memory, 30, -30) ;
    d3 = Constrain_up(desthrottle, 1500) + Constrain(cNd3_alpha, 15, -15) + Constrain(cNd3_omega, 130, -130) + Constrain(cNd3_theta, 60, -60)   + Constrain((int16_t)Scd, 23, -23) + Constrain((int16_t)Ahd, 10, -10) + Constrain((int16_t)cNd3_memory, 30, -30) ;
    d4 = Constrain_up(desthrottle, 1500) + Constrain(cNd4_alpha, 15, -15) + Constrain(cNd4_omega, 130, -130) + Constrain(cNd4_theta, 60, -60)   + Constrain((int16_t)Scd, 23, -23) + Constrain((int16_t)Ahd, 10, -10) + Constrain((int16_t)cNd4_memory, 30, -30) ;

    Moto_PwmRflash(d1, d2, d3, d4);//core 1 called in place 3
	
	 //              CW 3    1CCW	     / \				 
	 //  planform        * *            / | \ X axis     Y axis
	 //                   *               |             <=======
	 //      	     CCW2    4 CW         |
	
}


/*******************************************************************************
	* @Name				cyberNation_theta
	* @Description		cyberNation for angle
	* @Input			None
	* @Use    			(arguments too much)
	* @Output			cNd1_theta,cNd2_theta,cNd3_theta,cNd4_theta
	* @Return			None
*******************************************************************************/
void cyberNation_theta(void)
{
	//cybernation offset.core 3
    cNd1_theta = +roll_err * kp_theta_x + pitch_err * kp_theta_y + yaw_err * kp_theta_z ;
    cNd2_theta = -roll_err * kp_theta_x - pitch_err * kp_theta_y + yaw_err * kp_theta_z;
    cNd3_theta = -roll_err * kp_theta_x + pitch_err * kp_theta_y - yaw_err * kp_theta_z;
    cNd4_theta = +roll_err * kp_theta_x - pitch_err * kp_theta_y - yaw_err * kp_theta_z;
}

/*******************************************************************************
	* @Name				cyberNation_omega
	* @Description		cyberNation for angle velocity
	* @Input			None
	* @Use				gyro_cybernation[2:0],kp_omega_x,kp_omega_y,kp_omega_z
	* @Output			cNd1_omega,cNd2_omega,cNd3_omega,cNd4_omega
	* @Return			None
*******************************************************************************/
void cyberNation_omega(void)
{
    cNd1_omega = +gyro_cybernation[0] * kp_omega_x + gyro_cybernation[1] * kp_omega_y + gyro_cybernation[2] * kp_omega_z;
    cNd2_omega = -gyro_cybernation[0] * kp_omega_x - gyro_cybernation[1] * kp_omega_y + gyro_cybernation[2] * kp_omega_z;
    cNd3_omega = -gyro_cybernation[0] * kp_omega_x + gyro_cybernation[1] * kp_omega_y - gyro_cybernation[2] * kp_omega_z;
    cNd4_omega = +gyro_cybernation[0] * kp_omega_x - gyro_cybernation[1] * kp_omega_y - gyro_cybernation[2] * kp_omega_z;
}

/*******************************************************************************
	* @Name				cyberNation_alpha
	* @Description		cyberNation for angle acceleration
	* @Input			None
	* @Use				alpha_out[1:0],kp_alpha_x,kp_alpha_y
	* @Output			cNd1_alpha,cNd2_alpha,cNd3_alpha,cNd4_alpha
	* @Return			None
*******************************************************************************/
void cyberNation_alpha(void)
{
    cNd1_alpha = +alpha_out[0] * kp_alpha_x + alpha_out[1] * kp_alpha_y;
    cNd2_alpha = -alpha_out[0] * kp_alpha_x - alpha_out[1] * kp_alpha_y;
    cNd3_alpha = -alpha_out[0] * kp_alpha_x + alpha_out[1] * kp_alpha_y;
    cNd4_alpha = +alpha_out[0] * kp_alpha_x - alpha_out[1] * kp_alpha_y;
}

#define Filter_Num 6//sliding window with 6 values
/*******************************************************************************
	* @Name				Gyro_filter
	* @Description		gyro sliding window filter
	* @Input    		None
	* @Use				gyro[2:0],gyro_chushi[2:0]
	* @Output   		gyro_out[2:0]
	* @Return   		None
*******************************************************************************/
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

    gyro_out[0] = Filter_sum_x / Filter_Num;
    gyro_out[1] = Filter_sum_y / Filter_Num;
    gyro_out[2] = Filter_sum_z / Filter_Num;

    Filter_count++;

    if(Filter_count == Filter_Num)
    {
        Filter_count = 0;
    }
}

#define Filter_Num2 6//sliding window with <input> values
/*******************************************************************************
	* @Name				Accz_filter
	* @Description		acc sliding window filter
	* @Input    		None
	* @Use    			aacz,aacz_chushi
	* @Output   		accz_out
	* @Return   		None
*******************************************************************************/
void Accz_filter(void)
{
    static float Filter_accz[Filter_Num2];
    static uint8_t Filter_count2;
    float Filter_sum_accz = 0;
    uint8_t i;

    Filter_accz[Filter_count2] = aacz - aacz_chushi;

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

extern float angle_roll_out, angle_pitch_out, angle_yaw_out;


#define Filter_Num3 4//sliding window with 6 values
/*******************************************************************************
	* @Name				Angle_filter
	* @Description		Angle sliding window filter
	* @Input    		None
	* @Use    			roll,pitch,yaw
	* @Output   		angle_roll_out,angle_pitch_out,angle_yaw_out
	* @Return   		None
*******************************************************************************/
void Angle_filter(void)
{
    static float Filter_angle_roll[Filter_Num3], Filter_angle_pitch[Filter_Num3], Filter_angle_yaw[Filter_Num3];
    static uint8_t Filter_count3;
    float Filter_sum_angle_roll = 0, Filter_sum_angle_pitch = 0, Filter_sum_angle_yaw = 0;
    uint8_t i;

    Filter_angle_roll[Filter_count3] = roll;
    Filter_angle_pitch[Filter_count3] = pitch;
    Filter_angle_yaw[Filter_count3] = yaw;

    for(i = 0; i < Filter_Num3; i++)
    {
        Filter_sum_angle_roll += Filter_angle_roll[i];
        Filter_sum_angle_pitch += Filter_angle_pitch[i];
        Filter_sum_angle_yaw += Filter_angle_yaw[i];
    }

    angle_roll_out = Filter_sum_angle_roll / Filter_Num3;
    angle_pitch_out = Filter_sum_angle_pitch / Filter_Num3;
    angle_yaw_out = Filter_sum_angle_yaw / Filter_Num3;

    Filter_count3++;

    if(Filter_count3 == Filter_Num3)
    {
        Filter_count3 = 0;
    }
}

extern float accz_R;
extern float accz_Q;
extern float accz_K;
extern float accz_X_hat;
extern float accz_X_hat_minus;
extern float accz_P;

/*******************************************************************************
	* @Name				Kalman_filter_accz
	* @Description		kalman filter for z axis acceleration
	* @Input    		None
	* @Use    			accz_X_hat,accz_P,accy_Q,accz_R,aacz
	* @Output   		accz_X_hat_minus,accz_P,accz_K,accz_X_hat
	* @Return   		None
*******************************************************************************/
void Kalman_filter_accz(void)
{
	//in 32767
    //time update
    accz_X_hat_minus = accz_X_hat;
    accz_P = accz_P + accz_Q;

    //predict update
    accz_K = accz_P / (accz_P + accz_R);
    accz_X_hat = accz_X_hat_minus + accz_K * (aacz - accz_X_hat_minus);
    accz_P = (1 - accz_K) * accz_P;
}

extern float accy_R;
extern float accy_Q;
extern float accy_K;
extern float accy_X_hat;
extern float accy_X_hat_minus;
extern float accy_P;

/*******************************************************************************
	* @Name				Kalman_filter_accy
	* @Description		kalman filter for y axis acceleration
	* @Input    		None
	* @Use    			accy_X_hat,accy_P,accy_Q,accy_R,aacy
	* @Output   		accy_X_hat_minus,accy_P,accy_K,accy_X_hat
	* @Return   		None
*******************************************************************************/
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

/*******************************************************************************
	* @Name				Kalman_filter_accx
	* @Description		kalman filter for x axis acceleration
	* @Input    		None
	* @Use    			accx_X_hat,accx_P,accx_Q,accx_R,aacx
	* @Output   		accx_X_hat_minus,accx_P,accy_K,accx_X_hat
	* @Return   		None
*******************************************************************************/
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


extern short aacz_chushi;
extern u8 stopping_throttle_both_recorded;
extern u32 stopping_throttle_upper_bound_fine;
extern u32 stopping_throttle_lower_bound_fine;
extern float baro_climb_rate;

/*******************************************************************************
	* @Name				Altitude_hold_update
	* @Description		update altitude hold cybernation value
	* @Input    		None
	* @Use    			stopping_throttle_both_recorded,desroll,despitch,pitch,roll,stopping_throttle_lower_bound_fine,channel3_in,stopping_throttle_upper_bound_fine,kp_vel_z,baro_climb_rate
	* @Output   		Ahd
	* @Return   		None
*******************************************************************************/
void Altitude_hold_update(void)
{
    if(stopping_throttle_both_recorded && _fabsf(desroll) < 0.5 && _fabsf(despitch) < 0.5 && _fabsf(pitch) < 1 && _fabsf(roll) < 1 && stopping_throttle_lower_bound_fine < channel3_in && channel3_in < stopping_throttle_upper_bound_fine)
    {
        Ahd = -kp_vel_z * baro_climb_rate;//- kp_acc_z * (accz_X_hat_minus - aacz_chushi);
        //Ahd=0;//we do not use it temporarily
    }
    else
    {
        Ahd = 0;
    }
    Ahd = 0; //we temporarily disuse this
}

/*******************************************************************************
	* @Name				Sink_compensation
	* @Description		Sink compensation
	* @Input    		None
	* @Use    			channel3_in,desroll,despitch
	* @Output   		Scd
	* @Return   		None
*******************************************************************************/
void Sink_compensation(void)
{
	//degree to radian by multiplier 0.0174533
    Scd = (channel3_in - 1000) / (cosf(desroll * 0.0174533) * cosf(despitch * 0.0174533)) - (channel3_in - 1000); //cosine between EarthFrame_Z with BodyFrame_Z
}

/*******************************************************************************
	* @Name				Kalman_filter_gyro
	* @Description		kalman filter for three axes gyro
	* @Input    		None
	* @Use    			gyro_X_hat[2:0],gyro_P[2:0],gyro_Q[2:0],gyro_R[2:0],gyro_out[2:0]
	* @Output   		gyro_X_hat_minus[2:0],gyro_P[2:0],gyro_K[2:0],gyro_X_hat[2:0]
	* @Return   		None
*******************************************************************************/
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
        gyro_X_hat[i] = gyro_X_hat_minus[i] + gyro_K[i] * (gyro_out[i] - gyro_X_hat_minus[i]);
        gyro_P[i] = (1 - gyro_K[i]) * gyro_P[i];
    }
}

/*******************************************************************************
	* @Name				cyber_memory_update
	* @Description		record cybernation
	* @Input    		None
	* @Use    			cNd1_memory_temp,cNd2_memory_temp,cNd3_memory_temp,cNd4_memory_temp
	* @Output   		cNd1_memory,cNd2_memory,cNd3_memory,cNd4_memory,cNd1_memory_temp,cNd2_memory_temp,cNd3_memory_temp,cNd4_memory_temp
	* @Return   		None
*******************************************************************************/
void cyber_memory_update(void)
{
	cNd1_memory=cNd1_memory_temp;
	cNd2_memory=cNd2_memory_temp;
	cNd3_memory=cNd3_memory_temp;
	cNd4_memory=cNd4_memory_temp;
	
	cNd1_memory_temp=0;
	cNd2_memory_temp=0;
	cNd3_memory_temp=0;
	cNd4_memory_temp=0;
}
