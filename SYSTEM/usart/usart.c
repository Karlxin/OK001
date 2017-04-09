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

//接收到的RC数据,1000~2000
float RC_Target_ROL = 0, RC_Target_PIT = 0, RC_Target_YAW = 0;



#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))

#pragma import(__use_no_semihosting)
//标准库需要的支持函数
struct __FILE
{
    int handle;

};

FILE __stdout;
//定义_sys_exit()以避免使用半主机模式
int _sys_exit(int x)
{
    x = x;
    return x;
}
//重定义fputc函数
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
Description:串口初始化
**************************************************/
void Uart1_Init(u32 br_num)
{
    USART_InitTypeDef USART_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); //开启USART1时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    //配置PA9作为USART1　Tx
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA , &GPIO_InitStructure);
    //配置PA10作为USART1　Rx
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA , &GPIO_InitStructure);

    //配置USART1
    //中断被屏蔽了
    USART_InitStructure.USART_BaudRate = br_num;       //波特率可以通过地面站配置
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //8位数据
    USART_InitStructure.USART_StopBits = USART_StopBits_1;   //在帧结尾传输1个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;    //禁用奇偶校验
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //硬件流控制失能
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;  //发送、接收使能
    //配置USART1时钟
    //	USART_ClockInitStruct.USART_Clock = USART_Clock_Disable;  //时钟低电平活动
    //	USART_ClockInitStruct.USART_CPOL = USART_CPOL_Low;  //SLCK引脚上时钟输出的极性->低电平
    //	USART_ClockInitStruct.USART_CPHA = USART_CPHA_2Edge;  //时钟第二个边沿进行数据捕获
    //	USART_ClockInitStruct.USART_LastBit = USART_LastBit_Disable; //最后一位数据的时钟脉冲不从SCLK输出

    USART_Init(USART1, &USART_InitStructure);
    //	USART_ClockInit(USART1, &USART_ClockInitStruct);

    //使能USART1接收中断
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    //使能USART1
    USART_Cmd(USART1, ENABLE);
}
u8 TxBuffer[0xff];
u8 TxCounter = 0;
u8 count = 0;
u8 Rx_Buf[2][32];	//两个32字节的串口接收缓存
u8 Rx_Act = 0;		//正在使用的buf号
u8 Rx_Adr = 0;		//正在接收第几字节
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
    //发送中断
    if((USART1->SR & (1 << 7)) && (USART1->CR1 & USART_CR1_TXEIE)) //if(USART_GetITStatus(USART1,USART_IT_TXE)!=RESET)
    {
        USART1->DR = TxBuffer[TxCounter++]; //写DR清除中断标志
        if(TxCounter == count)
        {
            USART1->CR1 &= ~USART_CR1_TXEIE;		//关闭TXE中断
            //USART_ITConfig(USART1,USART_IT_TXE,DISABLE);
        }
    }
    //接收中断 (接收寄存器非空)
    if(USART1->SR & (1 << 5)) //if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        WaitForStart = 0;
        com_data = USART1->DR;
        if(Rx_Adr == 0)		//寻找帧头0X8A
        {
            if(com_data == 0x8A)	//接收数据如果是0X8A,则写入缓存
            {
                Rx_Buf[Rx_Act][0] = com_data;
                Rx_Adr = 1;
            }
        }
        else		//正在接收数据
        {
            Rx_Buf[Rx_Act][Rx_Adr] = com_data;
            Rx_Adr ++;
        }
        if(Rx_Adr == 32)		//数据接收完毕
        {
            Rx_Adr = 0;
            if(Rx_Act)
            {
                Rx_Act = 0; 			//切换缓存
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

/**************************实现函数********************************************
*******************************************************************************/
uint8_t Uart1_Put_Char(unsigned char DataToSend)
{
    TxBuffer[count++] = DataToSend;
    USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
    while((USART1->SR & 0X40) == 0); //等待上一次发送完毕
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
    while((USART1->SR & 0X40) == 0); //等待上一次发送完毕
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
    //判断Str指向的数据是否有效.
    while(*Str)
    {
        //是否是回车字符 如果是,则发送相应的回车 0x0d 0x0a
        if(*Str == '\r')Uart1_Put_Char(0x0d);
        else if(*Str == '\n')Uart1_Put_Char(0x0a);
        else Uart1_Put_Char(*Str);
        //指针++ 指向下一个字节.
        Str++;
    }
}


void Uart_DataAnl(u8 buf_num)		//串口缓存数据分析
{
    if(Rx_Buf[buf_num][1] == 0x8A)		//串口收到的是上位机的遥控数据
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
        if(sum == Rx_Buf[0][31])		//和校验通过
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
        if(sum == Rx_Buf[1][31])		//和校验通过
        {
            Uart_DataAnl(1);
        }
    }
}

void Uart1_Send_Buf(u8 *buf, u8 len)		//发送buf,长度len,返回字节和sum
{
    while(len)
    {
        Uart1_Put_Char(*buf);
        buf++;
        len--;
    }
}


void PC_Debug_Show(u8 num, u16 sta) //sta=0 熄灭 sta=1 点亮  >1 取反
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





