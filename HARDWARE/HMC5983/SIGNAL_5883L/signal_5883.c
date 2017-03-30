#include "stm32f10x_lib.h"
#include  "math.h"    
#include "IIC_5883.h"
#include "delay_5883.h"
#include "signal_5883.h"
#include "usart.h"

#define	HMC5883L_Addr   0x3C	              //磁场传感器器件地址


u8 test=0; 

u16 angle;
  
bool Single_Write(unsigned char SlaveAddress,unsigned char REG_Address,unsigned char REG_data)		     //void
{
  	if(!I2C_Start())return FALSE;
    I2C_SendByte(SlaveAddress);   //发送设备地址+写信号//I2C_SendByte(((REG_Address & 0x0700) >>7) | SlaveAddress & 0xFFFE);//设置高起始地址+器件地址 
    if(!I2C_WaitAck()){I2C_Stop(); return FALSE;}
    I2C_SendByte(REG_Address );   //设置低起始地址      
    I2C_WaitAck();	
    I2C_SendByte(REG_data);
    I2C_WaitAck();   
    I2C_Stop(); 
    delay5ms();
    return TRUE;
}

unsigned char Single_Read(unsigned char SlaveAddress,unsigned char REG_Address)
{   unsigned char REG_data;     	
	if(!I2C_Start())return FALSE;
    I2C_SendByte(SlaveAddress); //I2C_SendByte(((REG_Address & 0x0700) >>7) | REG_Address & 0xFFFE);//设置高起始地址+器件地址 
    if(!I2C_WaitAck()){I2C_Stop();test=1; return FALSE;}
    I2C_SendByte((u8) REG_Address);   //设置低起始地址      
    I2C_WaitAck();
    I2C_Start();
    I2C_SendByte(SlaveAddress+1);
    I2C_WaitAck();

	REG_data= I2C_RadeByte();
    I2C_NoAck();
    I2C_Stop();
    //return TRUE;
	return REG_data;

}						      
void conversion(u16 temp_data)  
{   
	printf("现在的角度为:=%d°\n",temp_data);
    Delayms(20);		
}

void read_hmc5883l()
{
       unsigned char BUF[8];                         //接收数据缓存区
       int   x,y;

       Single_Write(HMC5883L_Addr,0x00,0x14);    
       Single_Write(HMC5883L_Addr,0x02,0x00);   
  	   Delayms(10);

       BUF[1]=Single_Read(HMC5883L_Addr,0x03); 
       BUF[2]=Single_Read(HMC5883L_Addr,0x04); 

	   BUF[3]=Single_Read(HMC5883L_Addr,0x07); 
       BUF[4]=Single_Read(HMC5883L_Addr,0x08); 

       x=(BUF[1] << 8) | BUF[2];  
       y=(BUF[3] << 8) | BUF[4];  

       if(x>0x7fff)x-=0xffff;	  
       if(y>0x7fff)y-=0xffff;	  
       angle= atan2(y,x) * (180 / 3.14159265) + 180;  
	   conversion(angle);
	
	    
  }

