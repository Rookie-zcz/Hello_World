#include <stdint.h>  

uint8_t INV_Tx_Command;
uint8_t INV_Tx_Length;
uint8_t INV_Tx_data[256];

void UART_TX_Message(uint8_t Uart_Ch, uint8_t Command, uint16_t Length, uint8_t * data);

extern uint16_t Now_Minute;

extern uint8_t Remote_Ctrl_Mode;
extern uint16_t Work_Plan_Next_Stop_T;
extern uint8_t motor_rotate_flag;
extern uint8_t speed_gear;

uint8_t motor_Enable;

#define SPDLVL (4095/4) 

extern uint8_t FAN_SYS_Model[2];

extern uint8_t INV_Reset_Command;

uint8_t is_Model_Set(void);
uint8_t motor_Enable_Buf;

uint16_t INV_Connect_Prd = 250;
#define motor_Enable_Mask 0xFF

uint32_t motor_Rx_Time;
uint8_t motor_Rx_TimeOut;

uint32_t motor_Rx_TimeOutSet = 1000 * 60 * 5.5;

extern uint32_t Global_time_ms_32;
extern uint16_t Global_time_ms;
extern uint8_t Error_Reset_Req;

uint16_t abcd;
void INV_update_Command(void)
{
	uint16_t u16;
	uint32_t u32;	
	
	u32 = Global_time_ms_32 - motor_Rx_Time;
	if(u32 > motor_Rx_TimeOutSet)
	{
		motor_Rx_TimeOut = 1;
		motor_Rx_Time = Global_time_ms_32 - motor_Rx_TimeOutSet;
	}
		
	INV_Tx_Command = 0x38;
	INV_Tx_Length = 0;
	
	if(Remote_Ctrl_Mode == 1)
	{
			if(motor_Enable != 0)
			{
					if(Work_Plan_Next_Stop_T != 0xffff)
					{
						if(Now_Minute == Work_Plan_Next_Stop_T)
							motor_Enable = 0;
					};
			};
	}
	
	if(is_Model_Set() == 0)
		motor_Enable = 0;		
	
	//·çÉÈÆô¶¯Ç°£¬ÏÈÆô¶¯É¢ÈÈ·çÉÈ£¬·ÀÖ¹¿ØÖÆµçÔ´¼ì²âµ½Ç·Ñ¹
	motor_Enable_Buf = (motor_Enable_Buf << 1) | motor_Enable;
	if(motor_Enable_Buf == 0xFF) //0.25s * 8
		u16 = 1;
	else
		u16 = 0;		
		
	if(Error_Reset_Req == 1)	
		INV_Tx_data[INV_Tx_Length++] = 4;
	else
		INV_Tx_data[INV_Tx_Length++] = u16 | (INV_Reset_Command * 4);
	
	u16 = SPDLVL * speed_gear;
	abcd =u16;
	INV_Tx_data[INV_Tx_Length++] = u16 >> 8;
	INV_Tx_data[INV_Tx_Length++] = u16;	
	
	INV_Tx_data[INV_Tx_Length++] = FAN_SYS_Model[0];	
	INV_Tx_data[INV_Tx_Length++] = FAN_SYS_Model[1];
	
	UART_TX_Message(1, INV_Tx_Command, INV_Tx_Length, INV_Tx_data);
}

uint8_t Check_Download_Debug(void);

uint16_t INV_Connect_Time;  //5s exut

void Inv_Connect(void)
{
	uint16_t u16;
	
	if(Check_Download_Debug() == 0)
	{
		 u16 = Global_time_ms - INV_Connect_Time;
		 if(u16 > INV_Connect_Prd)
		 {
			  INV_Connect_Time = Global_time_ms;
			  INV_update_Command();
		 };	
	}
	else
		 INV_Connect_Time = Global_time_ms;
}

extern uint8_t motor_power;
extern uint16_t error_data;
extern uint16_t bus_volt;
extern uint16_t ctrl_volt;
extern uint16_t out_volt;
extern uint16_t out_curr;
extern uint16_t current_speed;
extern uint16_t set_speed;
extern uint16_t work_temperature;

uint16_t error_Count;
uint16_t error_Buf;

void INV_update_Status(uint8_t * data)
{
	uint16_t u16;
	int16_t i16;
	
	u16 = *data++;
	u16 = (u16<<8) | *data++;
	
	if(u16 != error_Buf)
	{
		error_Buf = u16;
		error_Count = 0;
		
		if(u16 == 0)
			error_data = 0;
	}
	else
	{
		if(error_Count < 5)
			error_Count++;
		else
			error_data = u16;
	};	

	motor_power = *data++;
	
	data++;
	
	u16 = *data++;
	u16 = (u16<<8) | *data++;	
	set_speed = u16 + 5;
	u16 = *data++;
	u16 = (u16<<8) | *data++;		
	current_speed = u16;
	
	u16 = *data++;
	u16 = (u16<<8) | *data++;	
	i16 = u16;
	if(i16 < 0)
		u16 = 0;	
	work_temperature = u16;
	
	u16 = *data++;
	u16 = (u16<<8) | *data++;		
	out_curr = u16;
	u16 = *data++;
	u16 = (u16<<8) | *data++;		
	bus_volt = u16;	
	u16 = *data++;
	u16 = (u16<<8) | *data++;		
	out_volt = u16;		
	u16 = *data++;
	u16 = (u16<<8) | *data++;		
	ctrl_volt = u16;		
}

uint8_t Inv_St_Buf[64];
uint8_t Inv_St_Buf_Length;
extern uint16_t DS18B20_Temper;  //unit in 1/16C

void Inv_St_Full(uint8_t Length, uint8_t * data)
{
	uint8_t i;
	
	for(i = 0; (i<Length) && (i<sizeof(Inv_St_Buf)); i++)
	{
		Inv_St_Buf[i] = *data++;	
	}
	
	Inv_St_Buf[i++] = (DS18B20_Temper + 8) >> 4;
	
	Inv_St_Buf_Length = i;
}

extern uint8_t  Remote_Debug_Mode;
void Download_Req(uint8_t * p_addr);

void UART1_RX_CMD(uint8_t Command, uint8_t Length, uint8_t * data)
{	
	motor_Rx_Time = Global_time_ms_32;
	motor_Rx_TimeOut = 0;
	
	switch(Command)
	{
		case 0xD0:
			Download_Req(data);
			break;
				
		case 0x38:
			  INV_update_Status(data);
				Inv_St_Full(Length, data);
			  break;
		case 0x79:		
			  INV_update_Status(data);
				UART_TX_Message(3, Command, Length, data);			
				break;		
		case 0xE0:
		case 0xE1:
		case 0xE2:
		case 0xE3:
		case 0xE4:
				UART_TX_Message(3, Command, Length, data);					
			  break;				
		case 0x78:			
				UART_TX_Message(3, Command, Length, data);					
			  break;			
	}	
}



