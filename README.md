#include <stdint.h> 

uint8_t PC_Tx_Command;
uint8_t PC_Tx_Length;
uint8_t PC_Tx_data[256];

void UART_TX_Message(uint8_t Uart_Ch, uint8_t Command, uint16_t Length, uint8_t * data);

__asm void GenerateSystemReset(void) 
{ 
 MOV R0, #1           //;  
 MSR FAULTMASK, R0    //; FAULTMASK ???????? 
 LDR R0, =0xE000ED0C  //; 
 LDR R1, =0x05FA0004  //;  
 STR R1, [R0]         //;    
  
deadloop 
    B deadloop        //;  
} 

uint8_t HW[3] = {0x88, 0x68, 0x82};

void Download_Req(uint8_t * p_addr)
{
   uint8_t *pdata;

   pdata = HW;//(uint8_t *)0x0800FF00L;
   if(*pdata++ == *p_addr++)
   {
	   if(*pdata++ == *p_addr++)
	   {
		   if(*pdata++ == *p_addr++)
		   {
			   GenerateSystemReset();
		   }
	   }
   }
 }
 
 uint8_t ResetKey[3] = {0x38, 0x38, 0x32};
 
 void Reset_Req(uint8_t * p_addr)
{
   uint8_t *pdata;

   pdata = ResetKey;//(uint8_t *)0x0800FF00L;
   if(*pdata++ == *p_addr++)
   {
	   if(*pdata++ == *p_addr++)
	   {
		   if(*pdata++ == *p_addr++)
		   {
			   GenerateSystemReset();
		   }
	   }
   }
 }
 
uint8_t * MODEL = (uint8_t *)(0x0800FF00L + 8);
uint8_t * HW_VER = (uint8_t *)(0x0800FF00L + 8 + 32);
uint8_t * SN = (uint8_t *)0x08001FF0L;
  
//char FW_VER[32] = "On_Off_Test 2020-7-21";
//char FW_VER[32] = "Panel_FAN_LC 2020-7-21";
char FW_VER[32] = "Panel_FAN_LC 2020-10-10";
//INV UART ¼ÓÈëRS-485 TX/RXÒý½Å¿ØÖÆ
// 2020-7-21, Ê§²½ºó30Ãëºó×Ô¶¯ÖØÆôÒ»´Î¡£

void ReadProductInfo(void)
{
	uint8_t i;
	uint8_t *pData;

	PC_Tx_data[PC_Tx_Length++] = HW[0];
	PC_Tx_data[PC_Tx_Length++] = HW[1];
	PC_Tx_data[PC_Tx_Length++] = HW[2];
	PC_Tx_data[PC_Tx_Length++] = ';';

	pData = (uint8_t *)MODEL;
	for(i = 0; i<32; i++)
	{
		if((*pData == 0) || (*pData == 0xFF))
			break;

		PC_Tx_data[PC_Tx_Length++] = *pData++;
	};
	PC_Tx_data[PC_Tx_Length++] = ';';

	pData = (uint8_t *)HW_VER;
	for(i = 0; i<8; i++)
	{
		if((*pData == 0) || (*pData == 0xFF))
			break;

		PC_Tx_data[PC_Tx_Length++] = *pData++;
	};
	PC_Tx_data[PC_Tx_Length++] = ';';

	pData = (uint8_t *)FW_VER;
	for(i = 0; i<32; i++)
	{
		if((*pData == 0) || (*pData == 0xFF))
			break;

		PC_Tx_data[PC_Tx_Length++] = *pData++;
	};
	PC_Tx_data[PC_Tx_Length++] = ';';

	pData = SN;
	PC_Tx_data[PC_Tx_Length++] = *pData++;
	PC_Tx_data[PC_Tx_Length++] = *pData++;
	PC_Tx_data[PC_Tx_Length++] = *pData++;
	PC_Tx_data[PC_Tx_Length++] = *pData++;
}

extern uint8_t FAN_SYS_Model[2], FAN_SYS_Model_Buf[2];

extern uint16_t Global_time_ms;
uint16_t INV_DLD_Time;  //5s exut
uint8_t  INV_Downloading;
uint16_t INV_Download_Delay = 10000;

extern uint32_t Global_time_ms_32;
uint32_t INV_RMV_Com_Time;  
uint8_t  Remote_Ctrl_Mode;
uint32_t INV_RMV_Com_Delay = 10 * 60 * 1000;//

extern uint8_t Inv_St_Buf[64];
extern uint8_t Inv_St_Buf_Length;

extern uint8_t speed_gear;

uint8_t Remote_Ctrl_Mode;

uint8_t Work_Plan_ID;
extern uint8_t speed_gear;
extern uint8_t temperature;
extern uint8_t motor_Enable;
extern uint16_t error_data;

uint16_t RM_On_Delay;
uint16_t RM_Enable_Buf;

extern uint8_t key_Value;
extern uint8_t V12_Volt; //0.1V

uint8_t is_Model_Set(void);
extern uint8_t speed_gear;
uint16_t Work_Plan_Next_Stop_T;//minute in week. less than a day; 0xFFFF always on
#define SPDLVL (4095/5 + 1) 

extern uint8_t V12_Volt;
extern uint8_t key_Value;
extern uint16_t DS18B20_Temper;  //unit in 1/16C

extern uint8_t FAN_SYS_Model[2];
extern uint8_t FAN_SYS_Model_Buf[2];

extern uint8_t week,hour,mintue;
void Set_RTC_Calib(uint16_t week_, uint8_t hour_, uint8_t mintue_);

void Write_Model(void);

void UART3_RX_CMD(uint8_t Command, uint8_t Length, uint8_t * data)
{	
	uint16_t u16;
	uint8_t Inv_Dld = 0;
	uint8_t week_, hour_,  mintue_;
	
	switch(Command)
	{
		case 0x10:  //Time Calibration, On / Off Time ; Status Readback		
			week_ = *data++;
			hour_ = *data++;
			mintue_ = *data++;
		
			Set_RTC_Calib(week_, hour_, mintue_);
				
			FAN_SYS_Model_Buf[0] = *data++;
			FAN_SYS_Model_Buf[1] = *data++;		
			Write_Model();
		
			PC_Tx_Command = 0x10;
		
			u16 = 0;

			PC_Tx_data[u16++] = week;
			PC_Tx_data[u16++] = hour;
			PC_Tx_data[u16++] = mintue;  	
		
			PC_Tx_data[u16++] = FAN_SYS_Model[0];
			PC_Tx_data[u16++] = FAN_SYS_Model[1]; 		
		
			PC_Tx_data[u16++] = V12_Volt;
			PC_Tx_data[u16++] = key_Value;

			PC_Tx_data[u16++] = DS18B20_Temper >> 3;  //0.5C

			PC_Tx_Length = u16;
			UART_TX_Message(3, PC_Tx_Command, PC_Tx_Length, PC_Tx_data);		
			break;
		
		case 0xD0:
			Download_Req(data);
			break;

		case 0xD1:
		case 0xD2:			
		case 0xD3:
  			break;

		case 0xD4:
			  PC_Tx_Command = 0xD4;
			  PC_Tx_Length = 0;

			  ReadProductInfo();

			  UART_TX_Message(3, PC_Tx_Command, PC_Tx_Length, PC_Tx_data);
				break;
		
		case 0xDF:
			Reset_Req(data);
			break;
		
		case 0xE0:		//download
		case 0xE1:		//download
		case 0xE2:		//download
		case 0xE3:		//download
		case 0xE4:		 //download
				INV_DLD_Time = Global_time_ms; 	
				Inv_Dld = 1;				
		
			  UART_TX_Message(1, Command, Length, data);		
 			  break;		
		case 0x78:		 //remote debug 
				INV_RMV_Com_Time = Global_time_ms_32; 	
				Remote_Ctrl_Mode = 1;				
		
			  UART_TX_Message(1, Command, Length, data);	
 			  break;		
		case 0x79:     //remote control
				INV_RMV_Com_Time = Global_time_ms_32; 	
				Remote_Ctrl_Mode = 1;
		
				motor_Enable = data[0] & 1;
				if(motor_Enable == 0)
				{
					RM_Enable_Buf = 0;
					RM_On_Delay = Global_time_ms;
				}				
				else
				{
					if(RM_Enable_Buf == 0)
					{
						u16 = Global_time_ms - RM_On_Delay;
						if(u16 > 500)
						{
							RM_Enable_Buf = motor_Enable;
						}							
					}
				};

				data[0] = (data[0] & 0xFE) | RM_Enable_Buf;
		
				u16 = data[1] * 256 + data[0];
				speed_gear = u16 / SPDLVL;
				if(speed_gear > 4)
					speed_gear = 4;
		
				if((data[3] == 0) && (data[4] == 0))  //
				{
					data[3] = FAN_SYS_Model[0];
					data[4] = FAN_SYS_Model[1];
				}
				else
				{
					FAN_SYS_Model_Buf[0] = data[3];
					FAN_SYS_Model_Buf[1] = data[4];					

					FAN_SYS_Model[0] = data[3];
					FAN_SYS_Model[1] = data[4];
				}
					
			  UART_TX_Message(1, Command, Length, data);		
 			  break;			
	}	
	
	INV_Downloading = Inv_Dld;	
}

uint8_t Check_Download_Debug(void)
{
	uint16_t	u16;
	uint32_t	u32;
	
	if(INV_Downloading != 0)
	{
			u16 = Global_time_ms - INV_DLD_Time;
		  if(u16 > INV_Download_Delay)
				INV_Downloading = 0;
	};
	
	if(Remote_Ctrl_Mode != 0)
	{
			u32 = Global_time_ms_32 - INV_RMV_Com_Time;
		  if(u32 > INV_RMV_Com_Delay)
				Remote_Ctrl_Mode = 0;
	};	
	
	return (INV_Downloading | Remote_Ctrl_Mode);
}
