/*--------------------------------version information top-------------------------------------------
Started at 2016
Created by Karlxin(410824290@qq.com)
Github:https://github.com/Karlxin/OK001.git
OpenKarlCopter
























--------------------------------version information bottom-------------------------------------------*/
#include "usart.h"
#include "ANO-Tech.h"

//Data for ANO master
u8 data_to_send[50];
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)		) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )

/**
  **********************************
  * @brief  串口1发送1个字节
  * @param  c：发送数据
  * @retval None
  **********************************
*/
void usart1_send_char(u8 c)
{
    while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET); //循环发送,直到发送完毕
    USART_SendData(USART1, c);
}

/**
  **********************************
  * @brief  传送数据给匿名四轴上位机软件(V2.6版本)
  * @param  fun:功能字. 0XA0~0XAF
						data:数据缓存区,最多28字节!!
						len:data区有效数据个数
  * @retval None
  **********************************
*/
void usart1_niming_report(u8 fun, u8 *data, u8 len)
{
    u8 send_buf[32];
    u8 i;
    if (len > 28)return;	//最多28字节数据
    send_buf[len + 3] = 0;	//校验数置零
    send_buf[0] = 0X88;	//帧头
    send_buf[1] = fun;	//功能字
    send_buf[2] = len;	//数据长度
    for (i = 0; i < len; i++)send_buf[3 + i] = data[i];			//复制数据
    for (i = 0; i < len + 3; i++)send_buf[len + 3] += send_buf[i];	//计算校验和
    for (i = 0; i < len + 4; i++)usart1_send_char(send_buf[i]);	//发送数据到串口1
}

/**
  **********************************
  * @brief  发送加速度传感器数据和陀螺仪数据
  * @param  aacx,aacy,aacz:x,y,z三个方向上面的加速度值
						gyrox,gyroy,gyroz:x,y,z三个方向上面的陀螺仪值
  * @retval None
  **********************************
*/
void mpu6050_send_data(short aacx, short aacy, short aacz, short gyrox, short gyroy, short gyroz)
{
    u8 tbuf[12];
    tbuf[0] = (aacx >> 8) & 0XFF;
    tbuf[1] = aacx & 0XFF;
    tbuf[2] = (aacy >> 8) & 0XFF;
    tbuf[3] = aacy & 0XFF;
    tbuf[4] = (aacz >> 8) & 0XFF;
    tbuf[5] = aacz & 0XFF;
    tbuf[6] = (gyrox >> 8) & 0XFF;
    tbuf[7] = gyrox & 0XFF;
    tbuf[8] = (gyroy >> 8) & 0XFF;
    tbuf[9] = gyroy & 0XFF;
    tbuf[10] = (gyroz >> 8) & 0XFF;
    tbuf[11] = gyroz & 0XFF;
    usart1_niming_report(0XA1, tbuf, 12);//自定义帧,0XA1
}

/**
  **********************************
  * @brief  通过串口1上报结算后的姿态数据给电脑
  * @param  aacx,aacy,aacz:x,y,z三个方向上面的加速度值
						gyrox,gyroy,gyroz:x,y,z三个方向上面的陀螺仪值
						roll:横滚角.单位0.01度。 -18000 -> 18000 对应 -180.00  ->  180.00度
						pitch:俯仰角.单位 0.01度。-9000 - 9000 对应 -90.00 -> 90.00 度
						yaw:航向角.单位为0.1度 0 -> 3600  对应 0 -> 360.0度
  * @retval None
  **********************************
*/
void usart1_report_imu(short aacx, short aacy, short aacz, short gyrox, short gyroy, short gyroz, short roll, short pitch, short yaw)
{
    u8 tbuf[28];
    u8 i;
    for (i = 0; i < 28; i++)tbuf[i] = 0; //清0
    tbuf[0] = (aacx >> 8) & 0XFF;
    tbuf[1] = aacx & 0XFF;
    tbuf[2] = (aacy >> 8) & 0XFF;
    tbuf[3] = aacy & 0XFF;
    tbuf[4] = (aacz >> 8) & 0XFF;
    tbuf[5] = aacz & 0XFF;
    tbuf[6] = (gyrox >> 8) & 0XFF;
    tbuf[7] = gyrox & 0XFF;
    tbuf[8] = (gyroy >> 8) & 0XFF;
    tbuf[9] = gyroy & 0XFF;
    tbuf[10] = (gyroz >> 8) & 0XFF;
    tbuf[11] = gyroz & 0XFF;
    tbuf[18] = (roll >> 8) & 0XFF;
    tbuf[19] = roll & 0XFF;
    tbuf[20] = (pitch >> 8) & 0XFF;
    tbuf[21] = pitch & 0XFF;
    tbuf[22] = (yaw >> 8) & 0XFF;
    tbuf[23] = yaw & 0XFF;
    usart1_niming_report(0XAF, tbuf, 28);//飞控显示帧,0XAF
}

void ANO_DT_Send_Data(u8 *data, u8 len)
{
    u8 i;
    if (len > 28)return;	//最多28字节数据
    for (i = 0; i < len; i++)usart1_send_char(data[i]);	//发送数据到串口1
}


/*******************************************************************************
  * @Name			ANO_DT_Send_Status
  * @Description	send status to master
  * @Input    		angle_rol,angle_pit,angle_yaw,alt,fly_model,armed
  * @Output   		data_to_send[50]
  * @Return   		None
*******************************************************************************/
void ANO_DT_Send_Status
	(float angle_rol, 
		float angle_pit, 
			float angle_yaw,
				s32 alt, 
			u8 fly_model, 
			u8 armed)
{
    u8 _cnt = 0;
    vs16 _temp;
    vs32 _temp2 = alt;
    u8 sum = 0;
    u8 i = 0;

    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0x01;
    data_to_send[_cnt++] = 0;

    _temp = (int)(angle_rol * 100);
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = (int)(angle_pit * 100);
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = (int)(angle_yaw * 100);
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);

    data_to_send[_cnt++] = BYTE3(_temp2);
    data_to_send[_cnt++] = BYTE2(_temp2);
    data_to_send[_cnt++] = BYTE1(_temp2);
    data_to_send[_cnt++] = BYTE0(_temp2);

    data_to_send[_cnt++] = fly_model;

    data_to_send[_cnt++] = armed;

    data_to_send[3] = _cnt - 4;


    for(i = 0; i < _cnt; i++)
        sum += data_to_send[i];
    data_to_send[_cnt++] = sum;

    ANO_DT_Send_Data(data_to_send, _cnt);
}



/*******************************************************************************
  * @Name			ANO_DT_Send_Senser
  * @Description	send sensor raw to master
  * @Input    		a_X,a_y,a_z,g_x,g_y,g_z,m_x,m_y,m_z,bar
  * @Output   		data_to_send[50]
  * @Return   		None
*******************************************************************************/
void ANO_DT_Send_Senser
	(s16 a_x, 
s16 a_y, 
s16 a_z,
s16 g_x, 
s16 g_y,
s16 g_z, 
s16 m_x, 
s16 m_y, 
s16 m_z,
s32 bar)
{
    u8 _cnt = 0;
    vs16 _temp;
    u8 sum = 0;
    u8 i = 0;

    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0x02;
    data_to_send[_cnt++] = 0;

    _temp = a_x;
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = a_y;
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = a_z;
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);

    _temp = g_x;
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = g_y;
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = g_z;
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);

    _temp = m_x;
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = m_y;
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = m_z;
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);

    data_to_send[3] = _cnt - 4;


    for(i = 0; i < _cnt; i++)
        sum += data_to_send[i];
    data_to_send[_cnt++] = sum;

    ANO_DT_Send_Data(data_to_send, _cnt);
}


/*******************************************************************************
  * @Name			ANO_DT_Send_MotoPWM
  * @Description	send motor PWM to master
  * @Input    		m1,m2,m3,m4,m5,m6,m7,m8
  * @Output   		data_to_send[50]
  * @Return   		None
*******************************************************************************/
void ANO_DT_Send_MotoPWM
	(u16 m_1,
u16 m_2,
u16 m_3,
u16 m_4, 
u16 m_5,
u16 m_6,
u16 m_7, 
u16 m_8)
{
    u8 _cnt = 0;
    u8 sum = 0;
    u8 i = 0;

    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0x06;
    data_to_send[_cnt++] = 0;

    data_to_send[_cnt++] = BYTE1(m_1);
    data_to_send[_cnt++] = BYTE0(m_1);
    data_to_send[_cnt++] = BYTE1(m_2);
    data_to_send[_cnt++] = BYTE0(m_2);
    data_to_send[_cnt++] = BYTE1(m_3);
    data_to_send[_cnt++] = BYTE0(m_3);
    data_to_send[_cnt++] = BYTE1(m_4);
    data_to_send[_cnt++] = BYTE0(m_4);
    data_to_send[_cnt++] = BYTE1(m_5);
    data_to_send[_cnt++] = BYTE0(m_5);
    data_to_send[_cnt++] = BYTE1(m_6);
    data_to_send[_cnt++] = BYTE0(m_6);
    data_to_send[_cnt++] = BYTE1(m_7);
    data_to_send[_cnt++] = BYTE0(m_7);
    data_to_send[_cnt++] = BYTE1(m_8);
    data_to_send[_cnt++] = BYTE0(m_8);

    data_to_send[3] = _cnt - 4;

    for(i = 0; i < _cnt; i++)
        sum += data_to_send[i];

    data_to_send[_cnt++] = sum;

    ANO_DT_Send_Data(data_to_send, _cnt);
}


/*******************************************************************************
	* @Name				ANO_DT_Send_RCData
	* @Description		send Remote Controller data to master
	* @Input    		thr,yaw,rol,pit,aux1,aux2,aux3,aux4,aux5,aux6
	* @Output   		data_to_send[49:0]
	* @Return   		None
*******************************************************************************/
void ANO_DT_Send_RCData
	(u16 thr,
u16 yaw,
u16 rol,
u16 pit,
u16 aux1,
u16 aux2,
u16 aux3,
u16 aux4,
u16 aux5,
u16 aux6)
{
    u8 _cnt = 0;
    u8 i;
    u8 sum = 0;

    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0x03;
    data_to_send[_cnt++] = 0;
    data_to_send[_cnt++] = BYTE1(thr);
    data_to_send[_cnt++] = BYTE0(thr);
    data_to_send[_cnt++] = BYTE1(yaw);
    data_to_send[_cnt++] = BYTE0(yaw);
    data_to_send[_cnt++] = BYTE1(rol);
    data_to_send[_cnt++] = BYTE0(rol);
    data_to_send[_cnt++] = BYTE1(pit);
    data_to_send[_cnt++] = BYTE0(pit);
    data_to_send[_cnt++] = BYTE1(aux1);
    data_to_send[_cnt++] = BYTE0(aux1);
    data_to_send[_cnt++] = BYTE1(aux2);
    data_to_send[_cnt++] = BYTE0(aux2);
    data_to_send[_cnt++] = BYTE1(aux3);
    data_to_send[_cnt++] = BYTE0(aux3);
    data_to_send[_cnt++] = BYTE1(aux4);
    data_to_send[_cnt++] = BYTE0(aux4);
    data_to_send[_cnt++] = BYTE1(aux5);
    data_to_send[_cnt++] = BYTE0(aux5);
    data_to_send[_cnt++] = BYTE1(aux6);
    data_to_send[_cnt++] = BYTE0(aux6);

    data_to_send[3] = _cnt - 4;

    for(i = 0; i < _cnt; i++)
        sum += data_to_send[i];

    data_to_send[_cnt++] = sum;

    ANO_DT_Send_Data(data_to_send, _cnt);
}
