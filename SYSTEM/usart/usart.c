#include "usart.h"
#include <stdio.h>
#include "led.h"
#include "imu.h"

#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))

u16 moto1, moto2, moto3, moto4;

u8 ARMED = 0;
char flag_send;

//���յ���RC����,1000~2000
float RC_Target_ROL = 0, RC_Target_PIT = 0, RC_Target_YAW = 0;



#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))

#pragma import(__use_no_semihosting)
//��׼����Ҫ��֧�ֺ���
struct __FILE
{
    int handle;

};

FILE __stdout;
//����_sys_exit()�Ա���ʹ�ð�����ģʽ
int _sys_exit(int x)
{
    x = x;
    return x;
}
//�ض���fputc����
int fputc(int ch, FILE *f)
{
    while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
    USART_SendData(USART1, (uint8_t)ch);
    return ch;
}

typedef union
{
    unsigned char byte[4];
    float num;
} t_floattobyte;
t_floattobyte floattobyte;
/*************************************************
Copyright:www.wellmakers.com
Date:2014-04-25
Description:���ڳ�ʼ��
**************************************************/
void Uart1_Init(u32 br_num)
{
    USART_InitTypeDef USART_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); //����USART1ʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    //����PA9��ΪUSART1��Tx
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA , &GPIO_InitStructure);
    //����PA10��ΪUSART1��Rx
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA , &GPIO_InitStructure);

    //����USART1
    //�жϱ�������
    USART_InitStructure.USART_BaudRate = br_num;       //�����ʿ���ͨ������վ����
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //8λ����
    USART_InitStructure.USART_StopBits = USART_StopBits_1;   //��֡��β����1��ֹͣλ
    USART_InitStructure.USART_Parity = USART_Parity_No;    //������żУ��
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //Ӳ��������ʧ��
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;  //���͡�����ʹ��
    //����USART1ʱ��
    //	USART_ClockInitStruct.USART_Clock = USART_Clock_Disable;  //ʱ�ӵ͵�ƽ�
    //	USART_ClockInitStruct.USART_CPOL = USART_CPOL_Low;  //SLCK������ʱ������ļ���->�͵�ƽ
    //	USART_ClockInitStruct.USART_CPHA = USART_CPHA_2Edge;  //ʱ�ӵڶ������ؽ������ݲ���
    //	USART_ClockInitStruct.USART_LastBit = USART_LastBit_Disable; //���һλ���ݵ�ʱ�����岻��SCLK���

    USART_Init(USART1, &USART_InitStructure);
    //	USART_ClockInit(USART1, &USART_ClockInitStruct);

    //ʹ��USART1�����ж�
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    //ʹ��USART1
    USART_Cmd(USART1, ENABLE);
}
u8 TxBuffer[0xff];
u8 TxCounter = 0;
u8 count = 0;
u8 Rx_Buf[2][32];	//����32�ֽڵĴ��ڽ��ջ���
u8 Rx_Act = 0;		//����ʹ�õ�buf��
u8 Rx_Adr = 0;		//���ڽ��յڼ��ֽ�
u8 Rx_Ok0 = 0;
u8 Rx_Ok1 = 0;
extern u8 ARMED ;
u8 WaitForStart ;
void Uart1_IRQ(void)
{
    u8 com_data ;
    if(USART1->SR & USART_IT_ORE)
    {
        USART1->SR;
    }
    //�����ж�
    if((USART1->SR & (1 << 7)) && (USART1->CR1 & USART_CR1_TXEIE)) //if(USART_GetITStatus(USART1,USART_IT_TXE)!=RESET)
    {
        USART1->DR = TxBuffer[TxCounter++]; //дDR����жϱ�־
        if(TxCounter == count)
        {
            USART1->CR1 &= ~USART_CR1_TXEIE;		//�ر�TXE�ж�
            //USART_ITConfig(USART1,USART_IT_TXE,DISABLE);
        }
    }
    //�����ж� (���ռĴ����ǿ�)
    if(USART1->SR & (1 << 5)) //if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        WaitForStart = 0;
        com_data = USART1->DR;
        if(Rx_Adr == 0)		//Ѱ��֡ͷ0X8A
        {
            if(com_data == 0x8A)	//�������������0X8A,��д�뻺��
            {
                Rx_Buf[Rx_Act][0] = com_data;
                Rx_Adr = 1;
            }
        }
        else		//���ڽ�������
        {
            Rx_Buf[Rx_Act][Rx_Adr] = com_data;
            Rx_Adr ++;
        }
        if(Rx_Adr == 32)		//���ݽ������
        {
            Rx_Adr = 0;
            if(Rx_Act)
            {
                Rx_Act = 0; 			//�л�����
                Rx_Ok1 = 1;
            }
            else
            {
                Rx_Act = 1;
                Rx_Ok0 = 1;
            }
        }
    }
}

/**************************ʵ�ֺ���********************************************
*******************************************************************************/
uint8_t Uart1_Put_Char(unsigned char DataToSend)
{
    TxBuffer[count++] = DataToSend;
    USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
    while((USART1->SR & 0X40) == 0); //�ȴ���һ�η������
    USART1->DR = DataToSend;
    return DataToSend;
}
uint8_t Uart1_Put_Int16(uint16_t DataToSend)
{
    uint8_t sum = 0, count = 0;
    TxBuffer[count++] = BYTE1(DataToSend);
    TxBuffer[count++] = BYTE0(DataToSend);
    USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
    sum += BYTE1(DataToSend);
    sum += BYTE0(DataToSend);
    while((USART1->SR & 0X40) == 0); //�ȴ���һ�η������
    USART1->DR = 1500;
    return sum;
}
uint8_t Uart1_Put_Float(float DataToSend)
{
    uint8_t sum = 0;
    floattobyte.num = DataToSend;
    TxBuffer[count++] = floattobyte.byte[3];
    TxBuffer[count++] = floattobyte.byte[2];
    TxBuffer[count++] = floattobyte.byte[1];
    TxBuffer[count++] = floattobyte.byte[0];
    USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
    sum += BYTE3(DataToSend);
    sum += BYTE2(DataToSend);
    sum += BYTE1(DataToSend);
    sum += BYTE0(DataToSend);
    return sum;
}
void Uart1_Put_String(unsigned char *Str)
{
    //�ж�Strָ��������Ƿ���Ч.
    while(*Str)
    {
        //�Ƿ��ǻس��ַ� �����,������Ӧ�Ļس� 0x0d 0x0a
        if(*Str == '\r')Uart1_Put_Char(0x0d);
        else if(*Str == '\n')Uart1_Put_Char(0x0a);
        else Uart1_Put_Char(*Str);
        //ָ��++ ָ����һ���ֽ�.
        Str++;
    }
}


void Uart_DataAnl(u8 buf_num)		//���ڻ������ݷ���
{
    if(Rx_Buf[buf_num][1] == 0x8A)		//�����յ�������λ����ң������
    {
        Uart1_Put_Char(0x30 + buf_num);
    }
}

void Uart_CheckEvent(void)
{
    int i;
    u8 sum;
    if(Rx_Ok0)
    {
        Rx_Ok0 = 0;
        sum = 0;
        for( i = 0; i < 31; i++)
            sum += Rx_Buf[0][i];
        if(sum == Rx_Buf[0][31])		//��У��ͨ��
        {
            Uart_DataAnl(0);
        }
    }
    if(Rx_Ok1)
    {
        Rx_Ok1 = 0;
        sum = 0;
        for( i = 0; i < 31; i++)
            sum += Rx_Buf[1][i];
        if(sum == Rx_Buf[1][31])		//��У��ͨ��
        {
            Uart_DataAnl(1);
        }
    }
}

void Uart1_Send_Buf(u8 *buf, u8 len)		//����buf,����len,�����ֽں�sum
{
    while(len)
    {
        Uart1_Put_Char(*buf);
        buf++;
        len--;
    }
}


void PC_Debug_Show(u8 num, u16 sta) //sta=0 Ϩ�� sta=1 ����  >1 ȡ��
{
    static uint8_t led_s[6] = {0, 0, 0, 0, 0, 0};
    uint8_t sum = 0;
    if(0 < num && num < 7)
    {
        sum += Uart1_Put_Char(0x88);
        sum += Uart1_Put_Char(0xAD);
        sum += Uart1_Put_Char(0x02);
        sum += Uart1_Put_Char(num);
        if(sta == 0)
            sum += Uart1_Put_Char(0x00);
        else if(sta == 1)
            sum += Uart1_Put_Char(0x01);
        else
        {
            if(led_s[num])	led_s[num] = 0;
            else 			led_s[num] = 1;
            sum += Uart1_Put_Char(led_s[num]);
        }
        Uart1_Put_Char(sum);
    }
    else if(6 < num && num < 13)
    {
        sum += Uart1_Put_Char(0x88);
        sum += Uart1_Put_Char(0xAD);
        sum += Uart1_Put_Char(0x03);
        sum += Uart1_Put_Char(num);
        sum += Uart1_Put_Int16(sta);
        Uart1_Put_Char(sum);
    }
}





