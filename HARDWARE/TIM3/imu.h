#ifndef _IMU_H_
#define _IMU_H_
#include "stm32f10x.h"

typedef struct{
				float X;
				float Y;
				float Z;}S_FLOAT_XYZ;
extern S_FLOAT_XYZ Q_ANGLE;			//四元数计算出的角度
extern S_FLOAT_XYZ GYRO_I;				//陀螺仪积分
void Prepare_Data(void);
void Get_Attitude(void);
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az);
void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
#endif
