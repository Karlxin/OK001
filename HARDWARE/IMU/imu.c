//=====================================================================================================
// IMU.c
// S.O.H. Madgwick
// 25th September 2010
//=====================================================================================================
// Description:
//
// Quaternion implementation of the 'DCM filter' [Mayhony et al].
//
// User must define 'halfT' as the (sample period / 2), and the filter gains 'Kp' and 'Ki'.
//
// Global variables 'q0', 'q1', 'q2', 'q3' are the quaternion elements representing the estimated
// orientation.  See my report for an overview of the use of quaternions in this application.
//
// User must call 'IMUupdate()' every sample period and parse calibrated gyroscope ('gx', 'gy', 'gz')
// and accelerometer ('ax', 'ay', 'ay') data.  Gyroscope units are radians/second, accelerometer
// units are irrelevant as the vector is normalised.
//
//=====================================================================================================

//----------------------------------------------------------------------------------------------------
// Header files

#include "imu.h"
#include <math.h>
#include <stdint.h>
//----------------------------------------------------------------------------------------------------
// Definitions

//#define Kp 2.0f // proportional gain governs rate of convergence to accelerometer/magnetometer
//#define Ki 0.005f // integral gain governs rate of convergence of gyroscope biases
//#define halfT 0.5f  // half the sample period

//试着修改一次,照这个重点参数
#define Kp 10.0f
#define Ki 0.008f
#define halfT 0.01f


//---------------------------------------------------------------------------------------------------
// Variable definitions

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;        // quaternion elements representing the estimated orientation
float exInt = 0, eyInt = 0, ezInt = 0;        // scaled integral error

//====================================================================================================
// Function
//====================================================================================================

float invSqrt(float number)
{
    volatile long i;
    volatile float x, y;
    volatile const float f = 1.5F;

    x = number * 0.5F;
    y = number;
    i = * (( long * ) &y);
    i = 0x5f375a86 - ( i >> 1 );
    y = * (( float * ) &i);
    y = y * ( f - ( x * y * y ) );
    return y;
}

//IMUupdate

//KarlKing buchongstart
void ag2q2rpy(float gx, float gy, float gz, float ax, float ay, float az, float *pitch, float *roll, float *yaw)
{
    float invnorm;
    float vx, vy, vz;
    float ex, ey, ez;

    // normalise the measurements
    invnorm = invSqrt(ax * ax + ay * ay + az * az);
    ax = ax * invnorm;
    ay = ay * invnorm;
    az = az * invnorm;
    //把加计的三维向量转成单位向量。


    // estimated direction of gravity
    vx = 2 * (q1 * q3 - q0 * q2);
    vy = 2 * (q0 * q1 + q2 * q3);
    vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    //这是把四元数换算成《方向余弦矩阵》中的第三列的三个元素。
    //根据余弦矩阵和欧拉角的定义，地理坐标系的重力向量，转到机体坐标系，正好是这三个元素。
    //所以这里的vx\y\z，其实就是当前的欧拉角（即四元数）的机体坐标参照系上，换算出来的重力单位向量。


    // error is sum of cross product between reference direction of field and direction measured by sensor
    ex = (ay * vz - az * vy);
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);

    //axyz是机体坐标参照系上，加速度计测出来的重力向量，也就是实际测出来的重力向量。
    //axyz是测量得到的重力向量，vxyz是陀螺积分后的姿态来推算出的重力向量，它们都是机体坐标参照系上的重力向量。
    //那它们之间的误差向量，就是陀螺积分后的姿态和加计测出来的姿态之间的误差。
    //向量间的误差，可以用向量叉积（也叫向量外积、叉乘）来表示，exyz就是两个重力向量的叉积。
    //这个叉积向量仍旧是位于机体坐标系上的，而陀螺积分误差也是在机体坐标系，而且叉积的大小与陀螺积分误差成正比，正好拿来纠正陀螺。（你可以自己拿东西想象一下）由于陀螺是对机体直接积分，所以对陀螺的纠正量会直接体现在对机体坐标系的纠正。



    // integral error scaled integral gain
    exInt = exInt + ex * Ki;
    eyInt = eyInt + ey * Ki;
    ezInt = ezInt + ez * Ki;

    // adjusted gyroscope measurements
    gx = gx + Kp * ex + exInt;
    gy = gy + Kp * ey + eyInt;
    gz = gz + Kp * ez + ezInt;

    //用叉积误差来做PI修正陀螺零偏




    // integrate quaternion rate and normalise
    q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * halfT;
    q1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * halfT;
    q2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * halfT;
    q3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * halfT;
    //四元数微分方程



    // normalise quaternion
    invnorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 = q0 * invnorm;
    q1 = q1 * invnorm;
    q2 = q2 * invnorm;
    q3 = q3 * invnorm;
    //四元数规范化

    *pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * 57.3;	// pitch
    *roll  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 57.3;	// roll
    *yaw   = atan2(2 * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 57.3;	//yaw
}
//KarlKing buchongend

//====================================================================================================
// END OF CODE
//====================================================================================================

extern float acc_climb_rate_out;
extern float Climb_X_hat_minus;
extern float acc_climb_err;
extern float baro_climb_rate;
extern float acc_climb_rate;

void complementation_filter(void)
{
	acc_climb_rate=baro_climb_rate; //offset the error
}










