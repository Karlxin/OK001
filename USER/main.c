/*--------------------------------version information top-------------------------------------------
Started at 2016
Created by Karlxin(410824290@qq.com)
Github:https://github.com/Karlxin/OK001.git
OpenKarlCopter
Version:dev_005

features:To be continued...

Notice:Dev version created without large experiment.The comments for every single line of the code will
be added soon.Karl has a whole theory to explain the control law and guide for optimization of gains.



postscript:
I am very grateful to Andrew Ng for the dreams of Artificial Intelligence and the open but valuable
course of machine learning in Coursera.

To make your life better.Making others' life better also.

If you wanted experiments videos and theory,please send emails to Karl with 410824290@qq.com.

Your time is valuable.No time for us to waste.Do our best to build the artificial intelligence

I hope some day we will meet each other with our dreams achieved.

Good Luck!

EL PSY CONGROO
--------------------------------version information bottom-------------------------------------------*/

#include "led.h"//light emitting diode head file 
#include "delay.h"//delay head file
#include "sys.h"//system head file
#include "usart.h"//universal synchronous asynchronous receiver transmitter head file
#include "stm32f10x.h"//stm32f10 controller head file
#include "moto.h"//motor head file
#include "timer.h"//timer 5 and 4 head file,timer 5 for system time,timer 4 for capture of pulse width modulation wave 
#include "pwm.h"//timer 3 for output of pulse width modulation wave 
#include "imu.h"//inertial measurement unit
#include "mpu6050.h"//motion process unit with accelerometer and gyrometer
#include "inv_mpu.h"//mpu hardware config and reading config head file
#include "inv_mpu_dmp_motion_driver.h"//mpu hardware config and reading config head file
#include "math.h"//math head file
#include <stdio.h>//standard input output with buffer
#include "spi.h"//serial peripheral interface
#include "flash.h"//electrically erasable programmable read only memory 

extern void TIM3_PWM_Init(u16 arr, u16 psc);//initialize TIM3 for four channel PWM output
extern void TIM4_Cap_Init(u16 arr, u16 psc);//initialize TIM4 for four channel PWM input
extern void TIM5_Int_Init(u16 arr, u16 psc);//initialize TIM5 for system time record
extern void Moto_Throttle(int16_t desthrottle);//last but one function to call the motor output function
extern void MS5611_init(void);//initialize the barometer for altitude and climb rate
extern void IIC_Init(void);//barometer Inter Integrated Circuit communication protocol initialization
extern void MS561101BA_RESET(void);//reset MS5611

extern void ANO_DT_Send_MotoPWM(u16 m_1, u16 m_2, u16 m_3, u16 m_4, u16 m_5, u16 m_6, u16 m_7, u16 m_8);//send motor PWM to master
extern void ANO_DT_Send_Senser(s16 a_x, s16 a_y, s16 a_z, s16 g_x, s16 g_y, s16 g_z, s16 m_x, s16 m_y, s16 m_z, s32 bar);//send sensor raw to master
extern void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, s32 alt, u8 fly_model, u8 armed);//send status to master
extern void ANO_DT_Send_RCData(u16 thr, u16 yaw, u16 rol, u16 pit, u16 aux1, u16 aux2, u16 aux3, u16 aux4, u16 aux5, u16 aux6);//send Remote Controller data to master

extern u8 MPU_Init(void);//initialize acclerometer and gyrometer
extern u8 mpu_dmp_init(void);//initialize hardware digital motion process 

u32 xitongshijian = 0; //system timer,resolution of 0.1ms,119hour
u32 haomiao = 0; //millisecond resolution
u32 erhaomiao = 0;
u32 wuhaomiao = 0;
u32 shihaomiao = 0; //ten milliseconds resolution
u32 ershihaomiao = 0;
u32 wushihaomiao = 0;
u32 yibaihaomiao = 0;//hundred milliseconds resolution
u32 yibaihaomiao2 = 0;
u32 miaozhong = 0; //second resolution
u32 ermiaozhong = 0;
u32 wumiaozhong = 0;
u32 shimiaozhong = 0;
u32 ershimiaozhong = 0;
u32 wushimiaozhong = 0;
u32 yibaimiaozhong = 0;
u32 erbaimiaozhong = 0;
u32 wubaimiaozhong = 0;
u32 yiqianmiaozhong = 0;

u32 channel1_in, channel2_in, channel3_in, channel4_in;//between 1000~2000
u32 STOPPING_THROTTLE = 1400;//guess for hover throttle

float roll, pitch, yaw;//Tait-Bryan angles ¦Õ¦È¦×,DMP hardware resolving ,roll,pitch,yaw
short aacx, aacy, aacz;//accelerometer raw data
short gyrox, gyroy, gyroz;//gyrometer raw data
short gyro[3] = {0, 0, 0};//deposit for gyro_x,gyro_y,gyro_z
short temp;//temperature
short gyrox_chushi, gyroy_chushi, gyroz_chushi; //at the beginning we grab some data from gyrometer as the sensor zero offset
short gyro_chushi[3] = {0, 0, 0};//for convenience,deposit for gyrox_chushi, gyroy_chushi, gyroz_chushi
float temp_gyxc = 0, temp_gyyc = 0, temp_gyzc = 0; //the temporal for gyrometer offset,use average filter
short aacx_chushi, aacy_chushi, aacz_chushi;//accelerometer initial value
float temp_axc = 0, temp_ayc = 0, temp_azc = 0;//the temporal for accelerometer offset,use average filter

float angle_roll_out, angle_pitch_out, angle_yaw_out;//the angle of each axis after sliding window filter

int16_t deadzone = 20; //remote controller deadzone

int16_t cNd1_theta = 0, cNd2_theta = 0, cNd3_theta = 0, cNd4_theta = 0; //cyberNation of angle
int16_t cNd1_omega = 0, cNd2_omega = 0, cNd3_omega = 0, cNd4_omega = 0; //cyberNation of angle velocity
int16_t cNd1_alpha = 0, cNd2_alpha = 0, cNd3_alpha = 0, cNd4_alpha = 0; //cyberNation of angle acceleration

int16_t cNd1_memory=0,cNd2_memory=0,cNd3_memory=0,cNd4_memory=0;//cyberNation record
int16_t cNd1_memory_temp=0,cNd2_memory_temp=0,cNd3_memory_temp=0,cNd4_memory_temp=0;//cyberNation record temporal
u16 memory_time_stamp;//cyberNation record time stamp
u8 memory_flag=0;//cyberNation record flag

extern float MS561101BA_get_altitude(float scaling);//calculate the altitude

extern int16_t Constrain_up(int16_t throttle, int16_t max);//constrain value from right

float MS5611_Altitude;//using MS5611 pressure and temperature to calculate the Altitude.
float Altitude_out = 0;//Altitude calculated from MS5611 barometer pressure

extern void MS561101BA_getPressure(void);//read pressure from MS5611 barometer
extern void MS561101BA_GetTemperature(void);//read temperature from MS5611 barometer
float MS5611_Pressure;//raw pressure data read from MS5611
int32_t  TEMP;//MS5611 barometer temperature
float Pressure_chushi;//raw pressure initial data read from MS5611
int32_t TEMP_chushi;//MS5611 barometer temperature initial data
float temp_TEMP = 0;//the temporal for temperature

float roll_err, pitch_err, yaw_err;//error,roll_err=roll-desroll,desroll is the conversion of remote controller channel input
float desroll, despitch, desyaw;//expectation

short gyro_out[3];//after sliding windows filter
short accz_out;//after filter, and trans
short gyro_cybernation[3];//gyro data after kalman filter

extern void Gyro_filter(void);//gyro sliding window filter
extern void Accz_filter(void);//acc sliding window filter

extern void Altitude_hold_update(void);//update altitude hold cybernation value

//-----------barometer top
int64_t OFF_;//for MS5611_Pressure

/*
C1  Pressure Sensitivity |SENS_T1
C2  Pressure Offset |OFF_T1
C3  Temperature coefficient of pressure sensitivity |TCS
C4  Temperature coefficient of pressure offset |TCO
C5  Reference temperature |T_REF
C6  Temperature coefficient of the temperature |TEMPSENS
*/
uint16_t  Cal_C[7];//for depositing [C7:C1]
uint32_t D1_Pres, D2_Temp;//digital pressure value,digital temperature value

/*
dT difference between actual and reference temperature
TEMP actual temperature (-40:+85 with resolution 0.01 degree Celsius)
*/
int32_t dT, TEMP;
/*
OFF offset at actual temperature
SENS sensitivity at actual temperature
*/
int64_t OFF, SENS;

int32_t P;//temperature compensated pressure (10:1200mbar with resolution 0.01mbar)

uint32_t Pressure;//unit 0.01mbar,1bar=100,000Pa

int64_t T2, TEMP2;//for MS5611_Pressure
int64_t OFF2, SENS2;//for MS5611_Pressure

//-----------barometer bottom

float scaling;//for altitude
float temp_jisuan;//for altitude

//---baro Alt karlman top
extern void Kalman_filter_alt(void);//kalman filter for altitude
float Altitude_minus = 0;//last barometer altitude converted by pressure
float Altitude_dt = 0.1;//the delta time
u32 Altitude_temp_time = 0; //record the time
float Altitude_R = 20; //measurement variance pn 20cm
float Altitude_Q = 0.01; //process variance 1/2*5*0.1^2
float Altitude_K = 0; //kalman gain
float Altitude_X_hat = 0; //init predict
float Altitude_X_hat_minus = 0; //previous predict
float Altitude_P = 0; //error variance
//---baro Alt karlman bottom

//---baro Climb Karlman top
extern void Kalman_filter_baro_climb(void);//kalman filter for baro climb rate
float baro_climb_R = 1;////measurement variance pn 1cm
float baro_climb_Q = 1; //process Variance,3/10=0.3
float baro_climb_K = 0; //kalman gain
float baro_climb_X_hat = 0; //init predict for Climb rate
float baro_climb_X_hat_minus = 0; //previous predict for Climb rate
float baro_climb_P = 0; //error variance
float baro_climb_rate = 0;//baro climb rate calculated from derivative 7 values filter
//---baro Climb karlman bottom

//---Kalman_filter_accz top
extern void Kalman_filter_accz(void);//kalman filter for z axis acceleration
float accz_dt = 0.01; //the delta time
u32 accz_temp_time = 0; //the record time
float accz_R = 83; //positive negative 67,measurement variance
float accz_Q = 0.01347916; //process Variance
float accz_K = 0; //kalman gain
float accz_X_hat = 15300; //init predict for accz
float accz_X_hat_minus = 0; //previous predict for accz
float accz_P = 140; //error variance
//---Kalman_filter_accz bottom

//---Kalman_filter_accy top
extern void Kalman_filter_accy(void);//kalman filter for y axis acceleration
float accy_dt = 0.01; //the delta time
u32 accy_temp_time = 0; //the record time
float accy_R = 23; //3sigma 23*3=69;
float accy_Q = 0.0004; //process Variance
float accy_K = 0; //kalman gain
float accy_X_hat = 300; //init predict for accy
float accy_X_hat_minus = 0; //previous predict for accy
float accy_P = 140; //error variance
//---Kalman_filter_accy bottom

//---Kalman_filter_accx top
extern void Kalman_filter_accx(void);//kalman filter for x axis acceleration
float accx_dt = 0.01; //the delta time
u32 accx_temp_time = 0; //the record time
float accx_R = 23; //3sigma 23*3=69;
float accx_Q = 0.0004; //process Variance
float accx_K = 0; //kalman gain
float accx_X_hat = -170; //init predict for accx
float accx_X_hat_minus = 0; //previous predict for accx
float accx_P = 140; //error variance
//---Kalman_filter_accx bottom

//---pressure kalman top
extern void Kalman_filter_pressure(void);//kalman filter for baro pressure
float pressure_dt = 0.01; //the delta time
u32 pressure_temp_time = 0; //the record time
float pressure_R = 20; //3sigma 7*3=21
float pressure_Q = 0.0169; //process Variance
float pressure_K = 0; //kalman gain
float pressure_X_hat = 0; //init predict
float pressure_X_hat_minus = 0; //previous predict
float pressure_P = 0.1; //error variance
//---pressure kalman bottom

//---gyrometer kalman top
extern void Kalman_filter_gyro(void);//kalman filter for gyro
//float gyro_dt[3] = {0.01,0.01,0.01}; //the delta time
float gyro_R[3] = {1, 1, 1}; //3sigma measurement variance
float gyro_Q[3] = {20, 25, 15}; //process Variance
float gyro_K[3] = {0, 0, 0}; //kalman gain
float gyro_X_hat[3] = {0, 0, 0}; //init predict
float gyro_X_hat_minus[3] = {0, 0, 0}; //previous predict
float gyro_P[3] = {1, 1, 1}; //error variance,6sigma
//---gyrometer kalman bottom

float Ahd = 0; //altitude hold throttle offset

extern void Sink_compensation(void);//sink update
float Scd = 0; //sink throttle offset

u32 debug[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; //time pin,to observe the real frequency.

//deviation between 0.1ms£¬little boy do not be afraid

int16_t d1 = 0, d2 = 0, d3 = 0, d4 = 0;//motor output value

float alpha[3]={0,0,0};//angle acceleration after derivative 7 values filter
float alpha_out[3]={0,0,0};//angle acceleration 

extern void cyberNation_theta(void);//cyberNation for angle
extern void cyberNation_omega(void);//cyberNation for angle velocity
extern void cyberNation_alpha(void);//cyberNation for angle acceleration

float Altitude_samples[7] = {0, 0, 0, 0, 0, 0, 0};//for climb rate
float Altitude_samples_time_stamps[7] = {0, 0, 0, 0, 0, 0, 0};//for climb rate
u8 Altitude_sample_index = 0;//for climb rate
u8 Altitude_samples_full = 0;//for climb rate
extern void Derivative_Filter_baro_climb_rate(void);//for climb rate

float gyrox_samples[7] = {0, 0, 0, 0, 0, 0, 0};//for x axis angle 2 order derivative with t 
float gyroy_samples[7] = {0, 0, 0, 0, 0, 0, 0};
float gyroz_samples[7] = {0, 0, 0, 0, 0, 0, 0};
float gyro_samples_time_stamps[7] = {0, 0, 0, 0, 0, 0, 0};//for angle 2 order derivative with t 
u8 gyro_sample_index = 0;//for angle 2 order derivative with t
u8 gyro_samples_full = 0;//for angle 2 order derivative with t
extern void Derivative_Filter_alpha(void);//for angle acceleration

u32 stopping_throttle_upper_bound_coarse = 1600;//the hover upper bound coarse throttle
u32 stopping_throttle_lower_bound_coarse = 1400;
u32 stopping_throttle_upper_bound_fine = 1600;//the hover upper bound fine throttle
u32 stopping_throttle_lower_bound_fine = 1400;
u32 stopping_throttle_temp;//temporal
float baro_trigger = 8; //the trigger to record channel3_in i.e. throttle in
u8 stopping_throttle_upper_recorded = 0;//flag for recording done
u8 stopping_throttle_lower_recorded = 0;
u8 stopping_throttle_both_recorded = 0;
u32 hover_range_top = 10;//for record the hover throttle
u32 hover_range_bottom = 50;

short accz_integral_deadzone = 3; //to create a deadzone and deal with steady noise
extern void Angle_filter(void);//Angle sliding window filter

//spi flash data top
u8 datatemp[240];//to write a page in w25q64.
u32 FLASH_SIZE = 8 * 1024 * 1024; //FLASH size of 8M Byte
u16 gaxyz[6]; //gryo xyz acc xyz data temp

u8 datatemp2[252];//to write a page in w25q61.recording the cybernation quantities
u16 rpygxyzbc[7];//roll pitch yaw gyro_out x y z baro climb rate

u8 datatemp3[234];
u16 kgxyz_ogxyz_drpy_orpy_bcr[13];//kalman gyro xyz,ouput gyro xyz,desired roll pitch yaw,ouput roll pitch yaw,baro climb rate
//spi flash data bottom

extern void cyber_memory_update(void);//record cybernation

//main top
int main(void)
{
    //------------------------------initiation top------------------------------
    u8 jiesuokeyi = 0;//sign for ARMED
    u16 baochijiesuo = 0;//counter for holding ARMED channel input
    u16 baochijiasuo = 0;//counter for holding DISARMED channel input
    u8 flymode = 0;//fly mode
    int16_t temp1, temp2,  desthrottle, temp4; //to convert channel pulse width modulation wave to expectation angle
    u8 i;//for for loop
	
    u8 USART1_Open = 1;//Open Serial by 500000 baud rate by setting it.
    u8 USART2_Open = 0;//Open Serial by 115200 baud rate by setting it.
    u8 kalman_gyro_Open = 0; //Open kalman instead of sliding window for gyro.we set it to 1 to open.
    u8 Flash_read_Open = 0; //flag for W25Q64 flash read
    u8 Flash_write_Open = 0; //flag for W25Q64 flash write
    u32 i2 = 0; //for read flash
    u32 i3 = 0; //for write flash
    u32 i4 = 0; //for write flash
    u32 i5 = 0; //for write flash
    //0 to record raw grro acc xyz,1 to record the cybernation quantities
    //2 to record the 1 and expectation roll pitch yaw and kalman gyro
    u8 record_option = 1;

    SystemInit();//over 0.02628ms
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//priority,parent divided by 0£¬1£¬2£¬3£»subclass by 0,1(2:0),over 0.0004ms
    delay_init();//over 0.00114s

    TIM3_PWM_Init(19999, 71);  //50Hz,over 0.02154ms

    TIM_SetCompare1(TIM3, 2000);//set duty ratio,over 0.00047ms
    TIM_SetCompare2(TIM3, 2000);
    TIM_SetCompare3(TIM3, 2000);
    TIM_SetCompare4(TIM3, 2000);

    delay_ms(5000);//delay 5000 millisecond

    TIM_SetCompare1(TIM3, 1000);
    TIM_SetCompare2(TIM3, 1000);
    TIM_SetCompare3(TIM3, 1000);
    TIM_SetCompare4(TIM3, 1000);

    delay_ms(3000);

    MPU_Init();//over 0.103s
    while(mpu_dmp_init())//is mpu initialization done
    {
        delay_ms(200);
        delay_ms(200);
    }//over 1.91s
    delay_ms(1000);
    IIC_Init();//inter integrated circuit bus protocol,over 0.00546ms
    delay_ms(100);
    MS561101BA_RESET();//over 0.35881ms
    delay_ms(100);
    MS5611_init();//over 0.0324s
    delay_ms(300);


    for(i = 0; i < 100; i++)
    {
        if(!MPU_Get_Gyroscope(&gyrox, &gyroy, &gyroz))//get gyrometer raw data
        {
            temp_gyxc = ((float)gyrox  + temp_gyxc) * 0.5;//to get averaged initial data
            temp_gyyc = ((float)gyroy  + temp_gyyc) * 0.5;
            temp_gyzc = ((float)gyroz  + temp_gyzc) * 0.5;
        }

        if(!MPU_Get_Accelerometer(&aacx, &aacy, &aacz)) //over 0.6ms
        {
            temp_axc = ((float)aacx + temp_axc) * 0.5;
            temp_ayc = ((float)aacy + temp_ayc) * 0.5;
            temp_azc = ((float)aacz + temp_azc) * 0.5;
            Kalman_filter_accz();//call the Kalman filter
        }
    }
    gyro_chushi[0] = (short)temp_gyxc;//record the initial data,over 0.00211ms
    gyro_chushi[1] = (short)temp_gyyc;
    gyro_chushi[2] = (short)temp_gyzc;

    aacx_chushi = (short)temp_axc;
    aacy_chushi = (short)temp_ayc;
    aacz_chushi = (short)accz_X_hat;

    for(i = 0; i < 20; i++)//20*100=2000milliseconds
    {
        delay_ms(100);

        MS561101BA_GetTemperature();//over 9.7ms
        temp_TEMP = ((float)TEMP + temp_TEMP) * 0.5;
    }//over 2.195s

    TEMP_chushi = (int32_t)temp_TEMP;//over 0.00203ms
    temp_jisuan    = (float)TEMP_chushi / 100.00 + 273.15f;//for the altitude conversion

    if(USART1_Open)
    {
        Uart1_Init(500000);//anonymous 4.06
    }
    else if(USART2_Open)
    {
        Uart1_Init(115200);//ATKXCOMV2.0
    }

    delay_ms(10);

    TIM4_Cap_Init(0xffff, 72 - 1); //pulse width modulation capturer,1MHz,over 0.02819ms
    TIM5_Int_Init(9, 719); //system timer,resolution of 0.1ms

    delay_ms(10);

    SPI_Flash_Init();//we must first erase the whole chip ,i.e.,set all the bit

    delay_ms(10);


    while(SPI_Flash_ReadID() != W25Q64)//W25Q64 has not been detected
    {
        LED0 = !LED0; //DS0 toggle
        delay_ms(500);
    }

    LED_Init();//over 0.01044ms
    LED0 = 1; //Darkening red LED,showing DISARMED
    LED1 = 0; //Lightening green LED£¬showing DISARMED

    delay_ms(10);

    haomiao = xitongshijian * 0.1f;
    erhaomiao = xitongshijian * 0.05f;//noticing that we disorder the schedule before
    wuhaomiao = xitongshijian * 0.02f;
    shihaomiao = xitongshijian * 0.01f;
    ershihaomiao = xitongshijian * 0.005f;
    wushihaomiao = xitongshijian * 0.002f;
    yibaihaomiao = xitongshijian * 0.001f;
    yibaihaomiao2 = xitongshijian * 0.001f;
    miaozhong = xitongshijian * 0.0001f;
    ermiaozhong = xitongshijian * 0.00005f;
    wumiaozhong = xitongshijian * 0.00002f;
    shimiaozhong = xitongshijian * 0.00001f;
    ershimiaozhong = xitongshijian * 0.000005f;
    wushimiaozhong = xitongshijian * 0.000002f;
    yibaimiaozhong = xitongshijian * 0.000001f;
    erbaimiaozhong = xitongshijian * 0.0000005f;
    wubaimiaozhong = xitongshijian * 0.0000002f;
    yiqianmiaozhong = xitongshijian * 0.0000001f;
    //------------------------------initiation bottom-----------------------------

    //loop top
    while(1)//using 8s to get there
    {
        //ms top
        if(xitongshijian * 0.1f > haomiao + 1)
        {
            haomiao = xitongshijian * 0.1f;
            debug[0]++;

            //read MPU top
			
			/*
            if(!MPU_Get_Accelerometer(&aacx, &aacy, &aacz)) //get acc data,over 0.6ms
            {
                Accz_filter();//sliding window filter
                //Kalman_filter_accz();
            }
			*/

            if(!MPU_Get_Gyroscope(&gyro[0], &gyro[1], &gyro[2]))  //get gyro data,over 0.6ms
            {
                Gyro_filter();//sliding window filter,over 0.02ms
				gyro_cybernation[0] =gyro_out[0];
				gyro_cybernation[1] =gyro_out[1];
				gyro_cybernation[2] =gyro_out[2];
				
                if(kalman_gyro_Open)
                {
                    Kalman_filter_gyro();
					gyro_cybernation[0] = gyro_X_hat_minus[0];
					gyro_cybernation[1] = gyro_X_hat_minus[1];
					gyro_cybernation[2] = gyro_X_hat_minus[2];
                }

                //alpha top
				/*
                gyrox_samples[gyro_sample_index] = gyro_X_hat_minus[0];
                gyroy_samples[gyro_sample_index] = gyro_X_hat_minus[1];
                gyro_samples_time_stamps[gyro_sample_index] = xitongshijian * 0.0001f;
                gyro_sample_index++;
                if(gyro_sample_index == 7)
                {
                    gyro_sample_index = 0;
                    if(!gyro_samples_full)
                    {
                        gyro_samples_full = 1;
                    }
                }
                if(gyro_samples_full)
                {
                    Derivative_Filter_alpha();
                    alpha_out[0] = alpha[0];
                    alpha_out[1] = alpha[1];
                }
				*/
                //alpha bottom
				
            }
            //read MPU bottom

        }
        //ms bottom

        //two ms top
        if(xitongshijian * 0.05f > erhaomiao + 1)
        {
            erhaomiao = xitongshijian * 0.05f; //visiting by two milliseconds resolution
            debug[1]++;
			
			/*//memory top
			if(jiesuokeyi)
			{
				if(__fabs(desroll)<1&&__fabs(despitch)<1&&__fabs(angle_roll_out)<1&&__fabs(angle_pitch_out)<1)//we are in fascinating state
				{
					memory_time_stamp++;
					if(memory_time_stamp>500)//we are in the fascinating state more than one second.
					{
						cNd1_memory_temp=d1-Constrain_up(desthrottle, 1500);
						cNd2_memory_temp=d2-Constrain_up(desthrottle, 1500);
						cNd3_memory_temp=d3-Constrain_up(desthrottle, 1500);
						cNd4_memory_temp=d4-Constrain_up(desthrottle, 1500);
						memory_flag=1;//we are ready to update the new balance position
						LED0=1;//using darkening red led to show something
						memory_time_stamp=0;
					}
				}
				else
				{
					memory_time_stamp=0;
				}
			}
			//memory bottom*/
			
        }
        //two ms bottom

        //five ms top
        if(xitongshijian * 0.02f > wuhaomiao + 1)
        {
            wuhaomiao = xitongshijian * 0.02f; //visiting by five milliseconds resolution
            debug[2]++;

            mpu_dmp_get_data(&pitch, &roll, &yaw);//over amazing 52ms,move 50ms delay and we got 2.1ms,use dmp hardware
            Angle_filter();

            roll_err = angle_roll_out - desroll; //get roll_err
            pitch_err = angle_pitch_out - despitch;
            yaw_err = angle_yaw_out - desyaw;

            //------------------------------read flash 200Hz top--------------------------------
            if(Flash_read_Open && !jiesuokeyi) //disarmed and Open read flash,that is to read the data in 10X rate.
            {
                switch(record_option)
                {
                case 0:
                    if(i2 < 699000)
                    {
                        SPI_Flash_Read(datatemp, 0 + i2 * 12, 12);
                        gaxyz[0] = ((u16)datatemp[1] << 8) | ((u16)datatemp[0]);
                        gaxyz[1] = ((u16)datatemp[3] << 8) | ((u16)datatemp[2]);
                        gaxyz[2] = ((u16)datatemp[5] << 8) | ((u16)datatemp[4]);
                        gaxyz[3] = ((u16)datatemp[7] << 8) | ((u16)datatemp[6]);
                        gaxyz[4] = ((u16)datatemp[9] << 8) | ((u16)datatemp[8]);
                        gaxyz[5] = ((u16)datatemp[11] << 8) | ((u16)datatemp[10]);
                        ANO_DT_Send_Senser(gaxyz[0], gaxyz[1], gaxyz[2], gaxyz[3], gaxyz[4], gaxyz[5], (s16)0, (s16)0, (s16)0, (s32)0);
                        i2++;
                    }
                    break;
                case 1:
                    if(i2 < 699000)
                    {
                        SPI_Flash_Read(datatemp2, 0 + i2 * 14, 14);
                        rpygxyzbc[0] = ((u16)datatemp2[1] << 8) | ((u16)datatemp2[0]);
                        rpygxyzbc[1] = ((u16)datatemp2[3] << 8) | ((u16)datatemp2[2]);
                        rpygxyzbc[2] = ((u16)datatemp2[5] << 8) | ((u16)datatemp2[4]);
                        rpygxyzbc[3] = ((u16)datatemp2[7] << 8) | ((u16)datatemp2[6]);
                        rpygxyzbc[4] = ((u16)datatemp2[9] << 8) | ((u16)datatemp2[8]);
                        rpygxyzbc[5] = ((u16)datatemp2[11] << 8) | ((u16)datatemp2[10]);
                        rpygxyzbc[6] = ((u16)datatemp2[13] << 8) | ((u16)datatemp2[12]);

                        ANO_DT_Send_Status((float)rpygxyzbc[0], (float)rpygxyzbc[1], (float)rpygxyzbc[2], (s32)0, (u8)0, (u8)0); //over 0.4ms
                        ANO_DT_Send_Senser((s16)0, (s16)0, (s16)0, rpygxyzbc[3], rpygxyzbc[4], rpygxyzbc[5], rpygxyzbc[6], (s16)0, (s16)0, (s32)0);
                        i2++;
                    }
                    break;
                case 2:
                    if(i2 < 699000)
                    {
                        SPI_Flash_Read(datatemp3, 0 + i2 * 26, 26);
                        kgxyz_ogxyz_drpy_orpy_bcr[0] = ((u16)datatemp3[1] << 8) | ((u16)datatemp3[0]);
                        kgxyz_ogxyz_drpy_orpy_bcr[1] = ((u16)datatemp3[3] << 8) | ((u16)datatemp3[2]);
                        kgxyz_ogxyz_drpy_orpy_bcr[2] = ((u16)datatemp3[5] << 8) | ((u16)datatemp3[4]);
                        kgxyz_ogxyz_drpy_orpy_bcr[3] = ((u16)datatemp3[7] << 8) | ((u16)datatemp3[6]);
                        kgxyz_ogxyz_drpy_orpy_bcr[4] = ((u16)datatemp3[9] << 8) | ((u16)datatemp3[8]);
                        kgxyz_ogxyz_drpy_orpy_bcr[5] = ((u16)datatemp3[11] << 8) | ((u16)datatemp3[10]);
                        kgxyz_ogxyz_drpy_orpy_bcr[6] = ((u16)datatemp3[13] << 8) | ((u16)datatemp3[12]);
                        kgxyz_ogxyz_drpy_orpy_bcr[7] = ((u16)datatemp3[15] << 8) | ((u16)datatemp3[14]);
                        kgxyz_ogxyz_drpy_orpy_bcr[8] = ((u16)datatemp3[17] << 8) | ((u16)datatemp3[16]);
                        kgxyz_ogxyz_drpy_orpy_bcr[9] = ((u16)datatemp3[19] << 8) | ((u16)datatemp3[18]);
                        kgxyz_ogxyz_drpy_orpy_bcr[10] = ((u16)datatemp3[21] << 8) | ((u16)datatemp3[20]);
                        kgxyz_ogxyz_drpy_orpy_bcr[11] = ((u16)datatemp3[23] << 8) | ((u16)datatemp3[22]);
                        kgxyz_ogxyz_drpy_orpy_bcr[12] = ((u16)datatemp3[25] << 8) | ((u16)datatemp3[24]);

                        ANO_DT_Send_Senser(kgxyz_ogxyz_drpy_orpy_bcr[0], kgxyz_ogxyz_drpy_orpy_bcr[1], \
                                           kgxyz_ogxyz_drpy_orpy_bcr[2], kgxyz_ogxyz_drpy_orpy_bcr[3], kgxyz_ogxyz_drpy_orpy_bcr[4], \
                                           kgxyz_ogxyz_drpy_orpy_bcr[5], kgxyz_ogxyz_drpy_orpy_bcr[6], kgxyz_ogxyz_drpy_orpy_bcr[7], \
                                           kgxyz_ogxyz_drpy_orpy_bcr[8], (s32)0);
                        ANO_DT_Send_Status((float)kgxyz_ogxyz_drpy_orpy_bcr[9], (float)kgxyz_ogxyz_drpy_orpy_bcr[10], \
                                           (float)kgxyz_ogxyz_drpy_orpy_bcr[11], (s32)0, (u8)0, (u8)0); //over 0.4ms
                        //ANO_DT_Send_Status((float)kgxyz_ogxyz_drpy_orpy_bcr[12], (float)0, \
                        (float)0, (s32)0, (u8)0, (u8)0); //over 0.4ms
                        i2++;
                    }
                    break;
                case 3:
                    if(i2 < 699000)
                    {
                        SPI_Flash_Read(datatemp, 0 + i2 * 12, 12);
                        gaxyz[0] = ((u16)datatemp[1] << 8) | ((u16)datatemp[0]);
                        gaxyz[1] = ((u16)datatemp[3] << 8) | ((u16)datatemp[2]);
                        gaxyz[2] = ((u16)datatemp[5] << 8) | ((u16)datatemp[4]);
                        gaxyz[3] = ((u16)datatemp[7] << 8) | ((u16)datatemp[6]);
                        gaxyz[4] = ((u16)datatemp[9] << 8) | ((u16)datatemp[8]);
                        gaxyz[5] = ((u16)datatemp[11] << 8) | ((u16)datatemp[10]);
                        ANO_DT_Send_Senser(gaxyz[0], gaxyz[1], gaxyz[2], gaxyz[3], gaxyz[4], gaxyz[5], (s16)0, (s16)0, (s16)0, (s32)0);
                        i2++;
                    }
                    break;
                default:
                    break;
                }

            }
            //------------------------------read flash 200Hz bottom-----------------------------


        }
        //five ms bottom

        //ten ms top
        if(xitongshijian * 0.01f > shihaomiao + 1)
        {
            shihaomiao = xitongshijian * 0.01f; //visiting by ten milliseconds resolution
            debug[3]++;

            //-----------------------------Control and Throttle update top----------------------------------------------
            if(jiesuokeyi) //if ARMED,over 0.01204ms
            {
                if(channel3_in > 1100)//over 0.01519ms
                {
                    desthrottle = channel3_in;//over 0.01208ms

                    if(channel1_in < 1507 - deadzone || channel1_in > 1507 + deadzone)//channel1 is not in deadzone ,over 0.01558ms
                    {
                        temp1 =  channel1_in - 1507 ;
                        desroll = (float)temp1 * 0.0361446; //covert to ¡À15¡ã

                    }
                    else//channel1 is in deadzone
                    {
                        desroll = 0;
                    }

                    if(channel2_in < 1508 - deadzone || channel2_in > 1508 + deadzone)
                    {
                        temp2 = 1508 - channel2_in;
                        despitch = (float)temp2 * 0.0361446;

                    }
                    else//channel2 is in deadzone
                    {
                        despitch = 0;
                    }

                    if(channel4_in < 1507 - deadzone || channel4_in > 1507 + deadzone)
                    {
                        temp4 = 1507 - channel4_in;
                        desyaw = (float)temp4 * 0.0361446;
                        desyaw = 0; //we just do not control yaw temporarily

                    }
                    else//channel4 is in deadzone
                    {
                        desyaw = 0;
                    }
                }
                else//channel3<1100
                {
                    d1 = 0;
                    d2 = 0;
                    d3 = 0;
                    d4 = 0;
                    Moto_PwmRflash(0, 0, 0, 0);//core 1 called in place 1
                }

                if(channel3_in < 1100 && channel4_in < 1100)//throttle<1100,yaw<1100,i.e. stick of throttle to the very left and down
                {
                    if(baochijiasuo == 0)//counter for holding disarming =0
                    {
                        baochijiasuo = miaozhong;//set counter for holding disarming to system seconds
                    }
                    if((miaozhong - baochijiasuo) >= 8)//holding stick for disarming maintained over eight seconds
                    {
                        jiesuokeyi = 0;//reset flag of disarming
                        LED1 = 0;//lightening green led stand for disarming
                        LED0 = 1;//darkening red led stand for disarming
                        baochijiasuo = 0; //reset baochijiasuo counter
                    }
                }
                else
                {
                    baochijiasuo = 0;//reset counter of holding disarming
                }

                if(channel3_in < 1100 && channel4_in > 1900)//armed but still holding for arming,that is to erase flash
                {
                    if(baochijiesuo == 0)//counter of arming=0
                    {
                        baochijiesuo = miaozhong;
                    }

                    if((miaozhong - baochijiesuo) >= 13)//holding arming over thirteen seconds
                    {
                        jiesuokeyi = 0;//set flag of disarming
                        LED1 = 0; //green led light
                        LED0 = 0; //red led light
                        //both light means erasing the whole external FLASH
                        SPI_Flash_Erase_Chip();
                        LED1 =  0;
                        LED0 = 1;
                        baochijiesuo = 0; //reset baochijiesuo
                        //return to initial status
                    }
                }
                else
                {
                    baochijiesuo = 0;//reset baochijiesuo
                }
            }
            else//if disarmed
            {
                d1 = 0;
                d2 = 0;
                d3 = 0;
                d4 = 0;
                Moto_PwmRflash(0, 0, 0, 0);//disarm all the motor,over 0.02ms,core 1 called in 2 place

                if(channel3_in < 1100 && channel4_in > 1900)//throttle<1100,yaw>1900,i.e.,the stick of throttle to the very right and bottom
                {
                    if(baochijiesuo == 0)//counter of arming=0
                    {
                        baochijiesuo = miaozhong;//get the system time
                    }
                    if((miaozhong - baochijiesuo) >= 5)//holding arming over five seconds
                    {
                        jiesuokeyi = 1;//set flag of arming
                        LED1 = 1; //green led dark stand for arming
                        LED0 = 0; //red led light stand for arming.please be careful of rotating propeller
                        baochijiesuo = 0; //reset baochijiesuo counter
                    }
                }
                else//channel3_in>1100,channel4_in<1900,that is throttle stick above bottom,yaw stick is not on the right
                {
                    baochijiesuo = 0;//holding ARMED counter reset.
                }

                if(channel3_in < 1100 && channel4_in < 1100)//throttle<1100,yaw<1100,i.e. stick of throttle to the very left and down
                {
                    if(baochijiasuo == 0)//counter for holding disarming =0
                    {
                        baochijiasuo = miaozhong;//set counter for holding disarming to system seconds
                    }
                    if((miaozhong - baochijiasuo) >= 8)//holding stick for disarming maintained over eight seconds
                    {
                        if(Flash_read_Open)//when reading flash and we get there
                        {
                            LED1 = 0; //lighten green led stand for initial status
                        }
                        else//we are not reading flash
                        {
                            LED1 = 1; //darken green LED
                        }
                        Flash_read_Open = !Flash_read_Open; //open flash reader

                        baochijiasuo = 0; //reset baochijiasuo counter
                    }
                }
                else
                {
                    baochijiasuo = 0;//reset counter of holding disarming
                }
            }


            //-----------------------------Control and Throttle update bottom--------------------------------------------

        }
        //ten ms bottom

        //twenty ms top
        if(xitongshijian * 0.005f > ershihaomiao + 1)
        {
            ershihaomiao = xitongshijian * 0.005f; //visiting by twenty milliseconds resolution
            debug[4]++;

            if(channel3_in > 1100 && jiesuokeyi) //only when channel3 >1100 and jiesuokeyi will update motor controlling
            {
				if(!memory_flag)
				{
                //cyberNation_alpha();//noise is too big
                cyberNation_omega();
                cyberNation_theta();
                Sink_compensation();//sink offset superposition
                Altitude_hold_update();//this frequency may be enough.altitude holding superposition
				}
				else
				{
					cyber_memory_update();
					memory_flag=0;
				}
                Moto_Throttle(desthrottle);//core 2 called in 1 place
            }

            if(!stopping_throttle_both_recorded)
            {
                if(jiesuokeyi && (stopping_throttle_lower_bound_coarse < channel3_in) && (channel3_in < stopping_throttle_upper_bound_coarse))
                {
                    if(!stopping_throttle_upper_recorded)
                    {
                        if(baro_trigger < baro_climb_rate)
                        {
                            stopping_throttle_upper_bound_fine = channel3_in + hover_range_top; //record the upper throttle
                            stopping_throttle_upper_recorded = 1; //set flag
                        }
                    }
                    if(!stopping_throttle_lower_recorded)
                    {
                        if(stopping_throttle_upper_recorded)
                        {
                            stopping_throttle_lower_bound_fine = stopping_throttle_upper_bound_fine - hover_range_top - hover_range_bottom; //record the upper throttle
                            stopping_throttle_lower_recorded = 1; //set flag
                        }
                    }
                    if(stopping_throttle_upper_recorded && stopping_throttle_lower_recorded)
                    {
                        stopping_throttle_both_recorded = 1; //set all done flag
                        LED0 = 1; //Darkening red LED£¬showing range recorded.we want to test cNd_memory first
                    }
                }
            }

        }
        //twenty ms bottom


        //fifty ms top
        if(xitongshijian * 0.002f > wushihaomiao + 1)
        {
            wushihaomiao = xitongshijian * 0.002f; //visiting by fifty milliseconds resolution
            debug[5]++;

            //serial top
            if(USART1_Open && !Flash_write_Open && !Flash_read_Open)
            {
                ANO_DT_Send_Status((float)angle_roll_out, (float)angle_pitch_out, (float)angle_yaw_out, (s32)MS5611_Altitude, (u8)flymode, (u8)jiesuokeyi); //over 0.4ms
                ANO_DT_Send_MotoPWM((u16) d1, (u16) d2, (u16) d3, (u16) d4, (u16) stopping_throttle_upper_recorded, (u16) stopping_throttle_lower_recorded, (u16) stopping_throttle_both_recorded, (u16) 0); //over 0.5ms
                ANO_DT_Send_RCData((u16)channel3_in, (u16) channel4_in, (u16) channel1_in, (u16) channel2_in, (u16) stopping_throttle_upper_bound_fine, (u16) stopping_throttle_lower_bound_fine, (u16) 0, (u16) 0, (u16) 0, (u16) 0); //0.5ms
                ANO_DT_Send_Senser((s16)gyro_cybernation[0] , (s16)gyro_cybernation[1], (s16)gyro_cybernation[2], (s16)gyro_out[0], (s16)gyro_out[1], (s16)gyro_out[2], (s16)Altitude_X_hat_minus, (s16)baro_climb_rate, (s16)baro_climb_X_hat_minus, (s32)MS5611_Altitude);

            }
            else if(USART2_Open && !Flash_write_Open && !Flash_read_Open)
            {
                printf("  MS5611_Altitude =%fm\r\n", MS5611_Altitude);
            }
            //serial bottom

            //write flash top
            if(jiesuokeyi&&Flash_write_Open) //armed ok
            {
                switch(record_option)
                {
                case 0://record raw data
                    if(i3 < 699000) //8MB i.e. 8*1024*1024Byte contains 699050*12Byte
                    {
                        datatemp[0 + i5 * 12] = aacx & 0x00ff; //little endian
                        datatemp[1 + i5 * 12] = aacx >> 8;
                        datatemp[2 + i5 * 12] = aacy & 0x00ff;
                        datatemp[3 + i5 * 12] = aacy >> 8;
                        datatemp[4 + i5 * 12] = aacz & 0x00ff;
                        datatemp[5 + i5 * 12] = aacz >> 8;
                        datatemp[6 + i5 * 12] = gyro[0] & 0x00ff;
                        datatemp[7 + i5 * 12] = gyro[0] >> 8;
                        datatemp[8 + i5 * 12] = gyro[1] & 0x00ff;
                        datatemp[9 + i5 * 12] = gyro[1] >> 8;
                        datatemp[10 + i5 * 12] = gyro[2] & 0x00ff;
                        datatemp[11 + i5 * 12] = gyro[2] >> 8;

                        i3++;
                        i5++;
                        if(i5 == 20) //we have collected 20 group data
                        {
                            SPI_Flash_Write2(datatemp, 0 + i4 * 240, 240);//over 1ms
                            i4++;
                            i5 = 0;
                        }
                    }
                    break;
                case 1://record cybernation data
                    if(i3 < 699000) //8MB i.e. 8*1024*1024Byte contains 699050*12Byte
                    {
                        datatemp2[0 + i5 * 14] = (s16)angle_roll_out & 0x00ff; //little endian
                        datatemp2[1 + i5 * 14] = (s16)angle_roll_out >> 8;
                        datatemp2[2 + i5 * 14] = (s16)angle_pitch_out & 0x00ff;
                        datatemp2[3 + i5 * 14] = (s16)angle_pitch_out >> 8;
                        datatemp2[4 + i5 * 14] = (s16)angle_yaw_out & 0x00ff;
                        datatemp2[5 + i5 * 14] = (s16)angle_yaw_out >> 8;
                        datatemp2[6 + i5 * 14] = gyro_out[0] & 0x00ff;
                        datatemp2[7 + i5 * 14] = gyro_out[0] >> 8;
                        datatemp2[8 + i5 * 14] = gyro_out[1] & 0x00ff;
                        datatemp2[9 + i5 * 14] = gyro_out[1] >> 8;
                        datatemp2[10 + i5 * 14] = gyro_out[2] & 0x00ff;
                        datatemp2[11 + i5 * 14] = gyro_out[2] >> 8;
                        datatemp2[12 + i5 * 14] = (s16)baro_climb_rate & 0x00ff;
                        datatemp2[13 + i5 * 14] = (s16)baro_climb_rate >> 8;

                        i3++;
                        i5++;
                        if(i5 == 18) //we have collected 18 group data
                        {
                            SPI_Flash_Write2(datatemp2, 0 + i4 * 252, 252);//over 1ms
                            i4++;
                            i5 = 0;
                        }
                    }
                    break;
                case 2:
                    if(i3 < 699000) //8MB i.e. 8*1024*1024Byte contains 699050*12Byte
                    {
                        datatemp3[0 + i5 * 26] = (s16)gyro_X_hat_minus[0] & 0x00ff; //little endian
                        datatemp3[1 + i5 * 26] = (s16)gyro_X_hat_minus[0] >> 8;
                        datatemp3[2 + i5 * 26] = (s16)gyro_X_hat_minus[1] & 0x00ff;
                        datatemp3[3 + i5 * 26] = (s16)gyro_X_hat_minus[1] >> 8;
                        datatemp3[4 + i5 * 26] = (s16)gyro_X_hat_minus[2] & 0x00ff;
                        datatemp3[5 + i5 * 26] = (s16)gyro_X_hat_minus[2] >> 8;

                        datatemp3[6 + i5 * 26] = gyro_out[0] & 0x00ff;
                        datatemp3[7 + i5 * 26] = gyro_out[0] >> 8;
                        datatemp3[8 + i5 * 26] = gyro_out[1] & 0x00ff;
                        datatemp3[9 + i5 * 26] = gyro_out[1] >> 8;
                        datatemp3[10 + i5 * 26] = gyro_out[2] & 0x00ff;
                        datatemp3[11 + i5 * 26] = gyro_out[2] >> 8;

                        datatemp3[12 + i5 * 26] = (s16)desroll & 0x00ff;
                        datatemp3[13 + i5 * 26] = (s16)desroll >> 8;
                        datatemp3[14 + i5 * 26] = (s16)despitch & 0x00ff;
                        datatemp3[15 + i5 * 26] = (s16)despitch >> 8;
                        datatemp3[16 + i5 * 26] = (s16)desyaw & 0x00ff;
                        datatemp3[17 + i5 * 26] = (s16)desyaw >> 8;

                        datatemp3[18 + i5 * 26] = (s16) angle_roll_out & 0x00ff;
                        datatemp3[19 + i5 * 26] = (s16) angle_roll_out >> 8;
                        datatemp3[20 + i5 * 26] = (s16) angle_pitch_out & 0x00ff;
                        datatemp3[21 + i5 * 26] = (s16) angle_pitch_out >> 8;
                        datatemp3[22 + i5 * 26] = (s16) angle_yaw_out   & 0x00ff;
                        datatemp3[23 + i5 * 26] = (s16) angle_yaw_out   >> 8;

                        datatemp3[24 + i5 * 26] = (s16)baro_climb_rate & 0x00ff;
                        datatemp3[25 + i5 * 26] = (s16) baro_climb_rate  >> 8;

                        i3++;
                        i5++;

                        if(i5 == 9) //we have collected 9 group data
                        {
                            SPI_Flash_Write2(datatemp3, 0 + i4 * 234, 234);//over 1ms
                            i4++;
                            i5 = 0;
                        }
                    }
                    break;
                case 3:
                    if(i3 < 699000) //8MB i.e. 8*1024*1024Byte contains 699050*12Byte
                    {
                        datatemp[0 + i5 * 12] = (s16)alpha_out[0] & 0x00ff; //little endian
                        datatemp[1 + i5 * 12] = (s16)alpha_out[0] >> 8;
                        datatemp[2 + i5 * 12] = (s16)alpha_out[1] & 0x00ff;
                        datatemp[3 + i5 * 12] = (s16)alpha_out[1] >> 8;
                        datatemp[4 + i5 * 12] = (s16)alpha_out[2] & 0x00ff;
                        datatemp[5 + i5 * 12] = (s16)alpha_out[2] >> 8;
                        datatemp[6 + i5 * 12] = gyro[0] & 0x00ff;
                        datatemp[7 + i5 * 12] = gyro[0] >> 8;
                        datatemp[8 + i5 * 12] = gyro[1] & 0x00ff;
                        datatemp[9 + i5 * 12] = gyro[1] >> 8;
                        datatemp[10 + i5 * 12] = gyro[2] & 0x00ff;
                        datatemp[11 + i5 * 12] = gyro[2] >> 8;

                        i3++;
                        i5++;
                        if(i5 == 20) //we have collected 20 group data
                        {
                            SPI_Flash_Write2(datatemp, 0 + i4 * 240, 240);//over 1ms
                            i4++;
                            i5 = 0;
                        }
                    }
                    break;
                default:
                    break;
                }
            }

            //write flash bottom
        }
        //fifty ms bottom


        //hundred ms top
        if(xitongshijian * 0.001f > yibaihaomiao + 1)
        {
            yibaihaomiao = xitongshijian * 0.001f; //visit by hundred milliseconds resolution
            debug[6]++;

            MS561101BA_getPressure();//over 9.1ms
            if(yibaihaomiao < 100)
            {
                Pressure_chushi = 0.5 * (Pressure_chushi + MS5611_Pressure);
            }
            else
            {
                Altitude_out = MS561101BA_get_altitude(MS5611_Pressure / Pressure_chushi);
                Kalman_filter_alt();

                Altitude_samples[Altitude_sample_index] = Altitude_X_hat_minus;
                Altitude_samples_time_stamps[Altitude_sample_index] = xitongshijian * 0.0001f;
                Altitude_sample_index++;
                if(Altitude_sample_index == 7)
                {
                    Altitude_sample_index = 0;
                    if(!Altitude_samples_full)
                    {
                        Altitude_samples_full = 1;
                    }
                }
                if(Altitude_samples_full)
                {
                    Derivative_Filter_baro_climb_rate();
                    Kalman_filter_baro_climb();
                }
            }
        }
        //hundred ms bottom

        //seconds top
        if(xitongshijian * 0.0001f > miaozhong + 1)
        {
            miaozhong = xitongshijian * 0.0001f; //visit by seconds resolution
            debug[7]++;
            if(Flash_read_Open)
            {
                LED1 = !LED1; //we are in the status of reading flash,so let the green led flash stand for reading flash
            }
			//operation details:
			//1.hold arm for more than about 20 second to erase all FLASH(black box)
			//2.hold disarm for more than about 10 second to read FLASH in
        }
        //seconds bottom

        /*
        //five seconds top
        if(xitongshijian * 0.00002f > wumiaozhong + 1)
        {
            wumiaozhong++;
        }
        //five seconds bottom
        */
    }
    //loop bottom
}
//main bottom

//I am Hououin Kyouma,a mad scientist,and the destroyer of this world's ruling structure.
//Failure is out of the question.

//Okay.I believe in you.

//little butterfly do not be afraid,the turbulence can save the world.
//OORGNO CYSPLE


