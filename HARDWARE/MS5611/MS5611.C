#include "MS5611.h"
#include "math.h"

//气压计状态机
#define SCTemperature    0x01	  //开始温度转换
#define CTemperatureing  0x02 //正在转换温度
#define SCPressure  		 0x03	    //开始气压转换
#define SCPressureing    0x04	  //正在转换气压

double OFF_;
float Aux;

/*
C1 压力灵敏度 SENS|T1
C2  压力补偿  OFF|T1
C3	温度压力灵敏度系数 TCS
C4	温度系数的压力补偿 TCO
C5	参考温度 T|REF
C6 	温度系数的温度 TEMPSENS
*/
uint16_t  Cal_C[7];	        //用于存放PROM中的6组数据1-6

uint32_t D1_Pres, D2_Temp;	// 数字压力值,数字温度值

/*
dT 实际和参考温度之间的差异
TEMP 实际温度
*/
int32_t dT, TEMP;
/*
OFF 实际温度补偿
SENS 实际温度灵敏度
*/
int64_t OFF, SENS;

int32_t P;//单位0.01mbar

uint32_t Pressure, Pressure_old, qqp;				//大气压//单位0.01mbar

int32_t T2,TEMP2;	//温度校验值
int64_t OFF2,SENS2;

uint32_t Pres_BUFFER[20];     //数据组
uint32_t Temp_BUFFER[10];     //数据组


//1 axis sliding windows filter
#undef FILTER_WINDOWS
#define  FILTER_WINDOWS    8

#define PA_OFFSET_INIT_NUM 50
static float Alt_offset_Pa = 0; //存放着0米离起飞所在平面时对应的气压值，这个值存放上电时的气压值
double paOffsetNum = 0;
uint16_t  paInitCnt = 0;
uint8_t paOffsetInited = 0;
static float Alt_Offset_m = 0;

static float sliding_windows_filter(float in)
{
    static float buf[FILTER_WINDOWS];
    static int buf_pointer = 0;
    int weight = FILTER_WINDOWS;
    int count = FILTER_WINDOWS;
    int index = buf_pointer;
    int total_weight = 0;
    float integral = {0};

    //add the newest date to the buffer
    buf[index] = in;

    //filter
    while(count--)
    {
        integral += buf[index] * weight;

        total_weight += weight; //the newest data have the most significant weight
        weight--;

        index++;
        if(index >= FILTER_WINDOWS)
            index = 0;
    }
    buf_pointer ++;
    if(buf_pointer >= FILTER_WINDOWS)
        buf_pointer = 0;

    //update the newest date
    return (integral / total_weight);
}



/*******************************************************************************
  * @函数名称	MS561101BA_RESET
  * @函数说明   复位MS5611
  * @输入参数   无
  * @输出参数   无
  * @返回参数   无
*******************************************************************************/
void MS561101BA_RESET(void)
{
    IIC_Start();
    IIC_Send_Byte(0xEE);//CSB接地，主机地址：0XEE，否则 0X77
    IIC_Wait_Ack();
    IIC_Send_Byte(0x1E);//发送复位命令
    IIC_Wait_Ack();
    IIC_Stop();

}
/*******************************************************************************
  * @函数名称	MS5611_init
  * @函数说明   初始化5611
  * @输入参数  	无
  * @输出参数   无
  * @返回参数   无
*******************************************************************************/
u8 MS5611_init(void)
{
    u8  inth, intl;
    int i;
    for (i = 1; i <= 6; i++)
    {

        IIC_Start();
        IIC_Send_Byte(0xEE);
        IIC_Wait_Ack();
		
        IIC_Send_Byte(0xA0 + (i * 2));
        IIC_Wait_Ack();
		
        IIC_Stop();
        delay_us(5);
        IIC_Start();
        IIC_Send_Byte(0xEE + 0x01); //进入接收模式
        delay_us(1);
        IIC_Wait_Ack();
        inth = IIC_Read_Byte(1);  		//带ACK的读数据
        delay_us(1);
        intl = IIC_Read_Byte(0); 			//最后一个字节NACK
        IIC_Stop();
        Cal_C[i] = (((uint16_t)inth << 8) | intl);
    }
    return !Cal_C[0];
}


/**************************实现函数********************************************
*函数原型:unsigned long MS561101BA_getConversion(void)
*功　　能:    读取 MS561101B 的转换结果
*******************************************************************************/
unsigned long MS561101BA_getConversion(uint8_t command)
{

    unsigned long conversion = 0;
    u8 temp[3];

    IIC_Start();
    IIC_Send_Byte(0xEE); 		//写地址
    IIC_Wait_Ack();
    IIC_Send_Byte(command); //写转换命令
    IIC_Wait_Ack();
    IIC_Stop();

    delay_ms(8);
    IIC_Start();
    IIC_Send_Byte(0xEE); 		//写地址
    IIC_Wait_Ack();
    IIC_Send_Byte(0);				// start read sequence
    IIC_Wait_Ack();
    IIC_Stop();

    IIC_Start();
    IIC_Send_Byte(0xEE + 0x01); //进入接收模式
    IIC_Wait_Ack();
    temp[0] = IIC_Read_Byte(1);  //带ACK的读数据  bit 23-16
    temp[1] = IIC_Read_Byte(1);  //带ACK的读数据  bit 8-15
    temp[2] = IIC_Read_Byte(0);  //带NACK的读数据 bit 0-7
    IIC_Stop();

    conversion = (unsigned long)temp[0] * 65536 + (unsigned long)temp[1] * 256 + (unsigned long)temp[2];
    return conversion;

}


/**************************实现函数********************************************
*函数原型:void MS561101BA_GetTemperature(void)
*功　　能:    读取 温度转换结果
*******************************************************************************/

void MS561101BA_GetTemperature(void)
{

    D2_Temp = MS561101BA_getConversion(0x58);
    //delay_ms(10);
    dT = D2_Temp - (((uint32_t)Cal_C[5]) << 8);
    TEMP = 2000 + dT * ((uint32_t)Cal_C[6]) / 8388608;
}

///***********************************************
//  * @brief  读取气压
//  * @param  None
//  * @retval None
//************************************************/
volatile float MS5611_Pressure;

void MS561101BA_getPressure(void)
{
    D1_Pres = MS561101BA_getConversion(0x48);
    //delay_ms(10);

    OFF_ = (uint32_t)Cal_C[2] * 65536 + ((uint32_t)Cal_C[4] * dT) / 128;
    SENS = (uint32_t)Cal_C[1] * 32768 + ((uint32_t)Cal_C[3] * dT) / 256;

    if(TEMP < 2000)
    {
        //Aux = (2000-TEMP)*(2000-TEMP);
        T2 = (dT * dT) / 0x80000000;
        Aux = TEMP * TEMP;
        OFF2 = 2.5f * Aux;
        SENS2 = 1.25f * Aux;

        TEMP = TEMP - TEMP2;
        OFF_ = OFF_ - OFF2;
        SENS = SENS - SENS2;
    }

    Pressure = (D1_Pres * SENS / 2097152 - OFF_) / 32768;
    MS5611_Pressure = Pressure;
}

float MS561101BA_get_altitude(void)
{
    static float Altitude, AltPre;
    float dz, dt;
    uint32_t current = 0;
    static uint32_t tp = 0;

    if(Alt_offset_Pa == 0)
    {
        if(paInitCnt > PA_OFFSET_INIT_NUM)
        {
            Alt_offset_Pa = paOffsetNum / paInitCnt;
            paOffsetInited = 1;
        }
        else
            paOffsetNum += MS5611_Pressure;

        paInitCnt++;

        Altitude = 0; //高度为0

        return Altitude;
    }
    //计算相对上电的位置的高度,单位m
    Altitude = 4433000.0f * (1.f - pow((MS5611_Pressure / Alt_offset_Pa), 0.1903f)) * 0.01f;
    //Altitude = Altitude - Alt_Offset_m ;
	//Altitude = sliding_windows_filter(Altitude);

    return Altitude;
}

