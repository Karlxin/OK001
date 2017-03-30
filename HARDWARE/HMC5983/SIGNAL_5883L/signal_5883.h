#ifndef __SIGNAL_H
#define __SIGNAL_H 

bool Single_Write(unsigned char SlaveAddress,unsigned char REG_Address,unsigned char REG_data);		     //void
unsigned char Single_Read(unsigned char SlaveAddress,unsigned char REG_Address);
void read_hmc5883l(void);
void conversion(u16 temp_data);  

#endif


