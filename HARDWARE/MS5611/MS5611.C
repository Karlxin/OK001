#include "MS5611.h"
#include "math.h"

//��ѹ��״̬��
#define SCTemperature    0x01	  //��ʼ�¶�ת��
#define CTemperatureing  0x02 //����ת���¶�
#define SCPressure  		 0x03	    //��ʼ��ѹת��
#define SCPressureing    0x04	  //����ת����ѹ

double OFF_;
float Aux;

/*
C1 ѹ�������� SENS|T1
C2  ѹ������  OFF|T1
C3	�¶�ѹ��������ϵ�� TCS
C4	�¶�ϵ����ѹ������ TCO
C5	�ο��¶� T|REF
C6 	�¶�ϵ�����¶� TEMPSENS
*/
uint16_t  Cal_C[7];	        //���ڴ��PROM�е�6������1-6

uint32_t D1_Pres, D2_Temp;	// ����ѹ��ֵ,�����¶�ֵ

/*
dT ʵ�ʺͲο��¶�֮��Ĳ���
TEMP ʵ���¶�
*/
int32_t dT, TEMP;
/*
OFF ʵ���¶Ȳ���
SENS ʵ���¶�������
*/
int64_t OFF, SENS;

int32_t P;//��λ0.01mbar

uint32_t Pressure, Pressure_old, qqp;				//����ѹ//��λ0.01mbar

int32_t T2,TEMP2;	//�¶�У��ֵ
int64_t OFF2,SENS2;

uint32_t Pres_BUFFER[20];     //������
uint32_t Temp_BUFFER[10];     //������


//1 axis sliding windows filter
#undef FILTER_WINDOWS
#define  FILTER_WINDOWS    8

#define PA_OFFSET_INIT_NUM 50
static float Alt_offset_Pa = 0; //�����0�����������ƽ��ʱ��Ӧ����ѹֵ�����ֵ����ϵ�ʱ����ѹֵ
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
  * @��������	MS561101BA_RESET
  * @����˵��   ��λMS5611
  * @�������   ��
  * @�������   ��
  * @���ز���   ��
*******************************************************************************/
void MS561101BA_RESET(void)
{
    IIC_Start();
    IIC_Send_Byte(0xEE);//CSB�ӵأ�������ַ��0XEE������ 0X77
    IIC_Wait_Ack();
    IIC_Send_Byte(0x1E);//���͸�λ����
    IIC_Wait_Ack();
    IIC_Stop();

}
/*******************************************************************************
  * @��������	MS5611_init
  * @����˵��   ��ʼ��5611
  * @�������  	��
  * @�������   ��
  * @���ز���   ��
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
        IIC_Send_Byte(0xEE + 0x01); //�������ģʽ
        delay_us(1);
        IIC_Wait_Ack();
        inth = IIC_Read_Byte(1);  		//��ACK�Ķ�����
        delay_us(1);
        intl = IIC_Read_Byte(0); 			//���һ���ֽ�NACK
        IIC_Stop();
        Cal_C[i] = (((uint16_t)inth << 8) | intl);
    }
    return !Cal_C[0];
}


/**************************ʵ�ֺ���********************************************
*����ԭ��:unsigned long MS561101BA_getConversion(void)
*��������:    ��ȡ MS561101B ��ת�����
*******************************************************************************/
unsigned long MS561101BA_getConversion(uint8_t command)
{

    unsigned long conversion = 0;
    u8 temp[3];

    IIC_Start();
    IIC_Send_Byte(0xEE); 		//д��ַ
    IIC_Wait_Ack();
    IIC_Send_Byte(command); //дת������
    IIC_Wait_Ack();
    IIC_Stop();

    delay_ms(8);
    IIC_Start();
    IIC_Send_Byte(0xEE); 		//д��ַ
    IIC_Wait_Ack();
    IIC_Send_Byte(0);				// start read sequence
    IIC_Wait_Ack();
    IIC_Stop();

    IIC_Start();
    IIC_Send_Byte(0xEE + 0x01); //�������ģʽ
    IIC_Wait_Ack();
    temp[0] = IIC_Read_Byte(1);  //��ACK�Ķ�����  bit 23-16
    temp[1] = IIC_Read_Byte(1);  //��ACK�Ķ�����  bit 8-15
    temp[2] = IIC_Read_Byte(0);  //��NACK�Ķ����� bit 0-7
    IIC_Stop();

    conversion = (unsigned long)temp[0] * 65536 + (unsigned long)temp[1] * 256 + (unsigned long)temp[2];
    return conversion;

}


/**************************ʵ�ֺ���********************************************
*����ԭ��:void MS561101BA_GetTemperature(void)
*��������:    ��ȡ �¶�ת�����
*******************************************************************************/

void MS561101BA_GetTemperature(void)
{

    D2_Temp = MS561101BA_getConversion(0x58);
    //delay_ms(10);
    dT = D2_Temp - (((uint32_t)Cal_C[5]) << 8);
    TEMP = 2000 + dT * ((uint32_t)Cal_C[6]) / 8388608;
}

///***********************************************
//  * @brief  ��ȡ��ѹ
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

        Altitude = 0; //�߶�Ϊ0

        return Altitude;
    }
    //��������ϵ��λ�õĸ߶�,��λm
    Altitude = 4433000.0f * (1.f - pow((MS5611_Pressure / Alt_offset_Pa), 0.1903f)) * 0.01f;
    //Altitude = Altitude - Alt_Offset_m ;
	//Altitude = sliding_windows_filter(Altitude);

    return Altitude;
}
