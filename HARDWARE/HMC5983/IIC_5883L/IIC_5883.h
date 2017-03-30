#ifndef __IIC_H
#define __IIC_H 

#define SCL_H         GPIOB->BSRR = GPIO_Pin_6
#define SCL_L         GPIOB->BRR  = GPIO_Pin_6 
   
#define SDA_H         GPIOB->BSRR = GPIO_Pin_7
#define SDA_L         GPIOB->BRR  = GPIO_Pin_7

#define SCL_read      GPIOB->IDR  & GPIO_Pin_6
#define SDA_read      GPIOB->IDR  & GPIO_Pin_7

void I2C_GPIO_Config(void);
bool I2C_Start(void);
void I2C_Stop(void);
void I2C_Ack(void);
void I2C_NoAck(void);
bool I2C_WaitAck(void); 	 //返回为:=1有ACK,=0无ACK
void I2C_SendByte(u8 SendByte); //数据从高位到低位//
unsigned char I2C_RadeByte(void);  //数据从高位到低位//

#endif


