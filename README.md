
#define __SYSCLK 36000000UL
#define __HCLK        (__SYSCLK)
#define __PCLK1       (__HCLK)
#define __PCLK2       (__HCLK)
#define __TIM1CLK     (__PCLK2)
#define __TIMXCLK     (__PCLK1)
 
#define __USART_SETUP             1                       //  0
#define __USART_USED              0x05                    //  1
#define __USART_DETAILS           0x00					  //  2
#define __USART_INTERRUPTS        0x00					  //  3
#define __USART1_BAUDRATE         38400					  //  4
#define __USART1_DATABITS         0x00000000
#define __USART1_STOPBITS         0x00000000
#define __USART1_PARITY           0x00000000
#define __USART1_FLOWCTRL         0x00000000
#define __USART1_REMAP            0x00000004
#define __USART1_CR1              0x000000A0
#define __USART1_CR2              0x00000000
#define __USART1_CR3              0x00000000

#define __USART3_BAUDRATE         38400 //57600                    // 22
#define __USART3_DATABITS         0x00000000
#define __USART3_STOPBITS         0x00000000
#define __USART3_PARITY           0x00000000
#define __USART3_FLOWCTRL         0x00000000
#define __USART3_REMAP            0x00000000
#define __USART3_CR1              0x00000000
#define __USART3_CR2              0x00000000
#define __USART3_CR3              0x00000000
 
 #define __DIV(__PCLK, __BAUD)       ((__PCLK*25)/(4*__BAUD))
#define __DIVMANT(__PCLK, __BAUD)   (__DIV(__PCLK, __BAUD)/100)
#define __DIVFRAQ(__PCLK, __BAUD)   (((__DIV(__PCLK, __BAUD) - (__DIVMANT(__PCLK, __BAUD) * 100)) * 16 + 50) / 100)
#define __USART_BRR(__PCLK, __BAUD) ((__DIVMANT(__PCLK, __BAUD) << 4)|(__DIVFRAQ(__PCLK, __BAUD) & 0x0F))
/**************************************************/
extern uint8_t commu_model_flag;
extern uint8_t zigbee_adress_buf;
extern void UART3_RX_CMD_MODBUS(uint8_t Command, uint8_t Length, uint16_t modbus_addr, uint8_t * data);

#define __USART3_BAUDRATE_MODBUS         9600                 // 22
#define __USART3_DATABITS_MODBUS         0x00001000
#define __USART3_STOPBITS_MODBUS         0x00000000
#define __USART3_PARITY_MODBUS           0x00000400
#define __USART3_FLOWCTRL_MODBUS         0x00000000
/***************************************************/
void stm32_UsartSetup (void) {
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;                     // enable clock for Alternate Function
    AFIO->MAPR   &= ~(1 << 2);                              // clear USART1 remap
    if      ((__USART1_REMAP & 0x04) == 0x00) {             // USART1 no remap
      RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;                   // enable clock for GPIOA
      GPIOA->CRH   &= ~(0xFFUL  << 4);                      // Clear PA9, PA10
      GPIOA->CRH   |=  (0x0BUL  << 4);                      // USART1 Tx (PA9)  alternate output push-pull
      GPIOA->CRH   |=  (0x04UL  << 8);                      // USART1 Rx (PA10) input floating
    }
    else {                                                  // USART1    remap
      RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;                   // enable clock for Alternate Function
      AFIO->MAPR   |= __USART1_REMAP;                       // set   USART1 remap
      RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;                   // enable clock for GPIOB
      GPIOB->CRL   &= ~(0xFFUL  << 24);                     // Clear PB6, PB7
      GPIOB->CRL   |=  (0x0BUL  << 24);                     // USART1 Tx (PB6)  alternate output push-pull
      GPIOB->CRL   |=  (0x04UL  << 28);                     // USART1 Rx (PB7) input floating
    }

    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;                   // enable clock for USART1

		USART1->BRR  = __USART_BRR(__PCLK2, __USART1_BAUDRATE); // set baudrate
		USART1->CR1  = __USART1_DATABITS;                       // set Data bits
		USART1->CR2  = __USART1_STOPBITS;                       // set Stop bits
		USART1->CR1 |= __USART1_PARITY;                         // set Parity
		USART1->CR3  = __USART1_FLOWCTRL;                       // Set Flow Control

    USART1->CR1 |= (USART_CR1_RE | USART_CR1_TE);           // RX, TX enable

    USART1->CR1 |= USART_CR1_UE;                            // USART enable

    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;                     // enable clock for Alternate Function
    AFIO->MAPR   &= ~(3 << 4);                              // clear USART3 remap
    if      ((__USART3_REMAP & 0x30) == 0x00) {             // USART3 no remap
      RCC->APB2ENR |=  RCC_APB2ENR_IOPBEN;                  // enable clock for GPIOB
      GPIOB->CRH   &= ~(0xFFUL  <<  8);                     // Clear PB10, PB11
      GPIOB->CRH   |=  (0x0BUL  <<  8);                     // USART3 Tx (PB10) alternate output push-pull
      GPIOB->CRH   |=  (0x04UL  << 12);                     // USART3 Rx (PB11) input floating
      if (__USART3_FLOWCTRL & 0x0300) {                     // HW flow control enabled
        GPIOB->CRH   &= ~(0xFFUL  << 20);                   // Clear PB13, PB14
        GPIOB->CRH   |=  (0x04UL  << 20);                   // USART3 CTS (PB13) input floating
        GPIOB->CRH   |=  (0x0BUL  << 24);                   // USART3 RTS (PB14) alternate output push-pull
      }
    }
    else if ((__USART3_REMAP & 0x30) == 0x10) {             // USART3 partial remap
      RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;                   // enable clock for Alternate Function
      AFIO->MAPR   |= __USART3_REMAP;                       // set   USART3 remap
      RCC->APB2ENR |=  RCC_APB2ENR_IOPCEN;                  // enable clock for GPIOC
      GPIOC->CRH   &= ~(0xFFUL  <<  8);                     // Clear PC10, PC11
      GPIOC->CRH   |=  (0x0BUL  <<  8);                     // USART3 Tx (PC10) alternate output push-pull
      GPIOC->CRH   |=  (0x04UL  << 12);                     // USART3 Rx (PC11) input floating
      if (__USART3_FLOWCTRL & 0x0300) {                     // HW flow control enabled
        RCC->APB2ENR |=  RCC_APB2ENR_IOPBEN;                // enable clock for GPIOB
        GPIOB->CRH   &= ~(0xFFUL  << 20);                   // Clear PB13, PB14
        GPIOB->CRH   |=  (0x04UL  << 20);                   // USART3 CTS (PB13) input floating
        GPIOB->CRH   |=  (0x0BUL  << 24);                   // USART3 RTS (PB14) alternate output push-pull
      }
    }
    else {                                                  // USART3 full remap
      RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;                   // enable clock for Alternate Function
      AFIO->MAPR   |= __USART3_REMAP;                       // set   USART3 remap
      RCC->APB2ENR |=  RCC_APB2ENR_IOPDEN;                  // enable clock for GPIOD
      GPIOD->CRH   &= ~(0xFFUL  <<  0);                     // Clear PD8, PD9
      GPIOD->CRH   |=  (0x0BUL  <<  0);                     // USART3 Tx (PD8) alternate output push-pull
      GPIOD->CRH   |=  (0x04UL  <<  4);                     // USART3 Rx (PD9) input floating
      if (__USART3_FLOWCTRL & 0x0300) {                     // HW flow control enabled
        GPIOD->CRH   &= ~(0xFFUL  << 12);                   // Clear PD11, PD12
        GPIOD->CRH   |=  (0x04UL  << 12);                   // USART3 CTS (PD11) input floating
        GPIOD->CRH   |=  (0x0BUL  << 16);                   // USART3 RTS (PD12) alternate output push-pull
      }
    } 

    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;                   // enable clock for USART3
		/*****************************************************************/
		if(commu_model_flag == 1)
		{
			USART3->BRR  = __USART_BRR(__PCLK1, __USART3_BAUDRATE_MODBUS); // set baudrate
			USART3->CR1  = __USART3_DATABITS_MODBUS;                       // set Data bits
			USART3->CR2  = __USART3_STOPBITS_MODBUS;                       // set Stop bits
			USART3->CR1 |= __USART3_PARITY_MODBUS;                         // set Parity
			USART3->CR3  = __USART3_FLOWCTRL_MODBUS;                       // Set Flow Control
		}
		else if(commu_model_flag == 0)
		{
			USART3->BRR  = __USART_BRR(__PCLK1, __USART3_BAUDRATE); // set baudrate
			USART3->CR1  = __USART3_DATABITS;                       // set Data bits
			USART3->CR2  = __USART3_STOPBITS;                       // set Stop bits
			USART3->CR1 |= __USART3_PARITY;                         // set Parity
			USART3->CR3  = __USART3_FLOWCTRL;                       // Set Flow Control
    };
		/********************************************************************/
    USART3->CR1 |= (USART_CR1_RE | USART_CR1_TE);           // RX, TX enable
    USART3->CR1 |= USART_CR1_UE;                            // USART enable
} // end of stm32_UsartSetup


void change_Uart3_Baud(int baud)
{
		USART3->BRR  = __USART_BRR(__PCLK1, baud);
}

#define XON 0x3A


typedef struct
{
	uint8_t Tx_Buffer[128 + 4];
	
	uint8_t Rx_Buffer[256];  //240
	uint8_t Rx_Data[256];
	uint16_t Rx_Offset;
	uint16_t Rx_Timer;
	uint16_t Rx_Count;
	
	uint8_t DMA_Buffer[256];
	uint16_t DMA_CNDTR;		
}USART_Def;

USART_Def UART1, UART3;

uint8_t * get_UART3_Tx_Buffer()
{
	return UART3.Tx_Buffer;
}

uint8_t * get_UART3_DMA_Buffer()
{
	return UART3.DMA_Buffer;
}


uint32_t DMA_Length = sizeof(UART1.DMA_Buffer);	
extern uint16_t Global_time_ms;

//Length <=256
void UART1_RX_CMD(uint8_t Command, uint8_t Length, uint8_t * data);
void UART3_RX_CMD(uint8_t Command, uint8_t Length, uint8_t * data);

//0x3A, Command, Length, data[Length], Lrc, summon from Command
uint8_t UART_RX_Message(USART_Def * Uart)
{
	uint16_t i, j, len;
	uint8_t data, sum;

	len = Uart->Rx_Buffer[2] + 3;

	sum = 0;
	j = 0;
	for(i = 0; i < len; i++)
	{
		data = Uart->Rx_Buffer[i + 1];
		Uart->Rx_Data[j++] = data;
		sum += data;
	};

	if(sum == 0)
		return 1;
	else
		return 0;
}

void DMA_Usart_Init()
{
	USART3->CR3 = (1<<7) | (1<<6);
	
	RCC->AHBENR |= 1;
	
	DMA1_Channel3->CCR = (1<<7);
	DMA1_Channel3->CNDTR = DMA_Length;	
	DMA1_Channel3->CPAR = (uint32_t)&USART3->DR;
	DMA1_Channel3->CMAR = (uint32_t)UART3.DMA_Buffer;	
	DMA1_Channel3->CCR = (1<<7) | 1;
	UART3.DMA_CNDTR = DMA_Length;	

	DMA1_Channel2->CCR = (1<<7) | (1<<4);
	DMA1_Channel2->CNDTR = 0;	
	DMA1_Channel2->CPAR = (uint32_t)&USART3->DR;
	DMA1_Channel2->CMAR = (uint32_t)UART3.Tx_Buffer;
	
	USART1->CR3 = (1<<7) | (1<<6);
		
	DMA1_Channel5->CCR = (1<<7);
	DMA1_Channel5->CNDTR = DMA_Length;	
	DMA1_Channel5->CPAR = (uint32_t)&USART1->DR;
	DMA1_Channel5->CMAR = (uint32_t)UART1.DMA_Buffer;	
	DMA1_Channel5->CCR = (1<<7) | 1;
	UART1.DMA_CNDTR = DMA_Length;	

	DMA1_Channel4->CCR = (1<<7) | (1<<4);
	DMA1_Channel4->CNDTR = 0;	
	DMA1_Channel4->CPAR = (uint32_t)&USART1->DR;
	DMA1_Channel4->CMAR = (uint32_t)UART1.Tx_Buffer;	
}

void CopyData(USART_Def * Uart, uint16_t Length)
{
	uint16_t i, j;
	uint8_t data;
	uint8_t RX_Start;
	
	RX_Start = 0;
	j = 0;
	
	for(i = 0; i<Length; i++)
	{
		data = Uart->DMA_Buffer[i];
		
		if(data == XON)
		{
			RX_Start = 1;		
		};
		
		if(RX_Start == 1)
		{
			Uart->Rx_Buffer[j++] = data;
		}			
	}
	
	if(RX_Start == 1)
	{
		if(j > 2)
		{
				if(UART_RX_Message(Uart) != 0)
				{
					Uart->Rx_Count++;
					if(Uart == &UART1)
						UART1_RX_CMD(Uart->Rx_Data[0], Uart->Rx_Data[1], &Uart->Rx_Data[2]);
					else if(Uart == &UART3)
						UART3_RX_CMD(Uart->Rx_Data[0], Uart->Rx_Data[1], &Uart->Rx_Data[2]);
				}	
		}			
	}
}
/******************************************************************************************/
const uint16_t polynomial = 0xa001;
uint16_t calculate_CRC(uint8_t *data,uint8_t length)
{
	uint16_t crc = 0xffff;
	uint8_t i,j;
	uint8_t crc_1;
	uint8_t crc_2;
	for(i = 0;i<length;i++)
	{
		crc_1 = crc;
		crc_1 ^= data[i];
		crc &= 0xff00;
		crc += crc_1;
		
		for(j = 0;j < 8;j++)
		{
			if((crc|0xfffe)==0xffff)
			{
				crc = crc>>1;
				crc ^= polynomial;
			}
			else
			{
				crc = crc>>1;
			}
		}
	}
	crc_1 = crc;
	crc_2 = crc>>8;
	crc = 0;
	crc = ((uint8_t)crc_1)<<8;
	crc += crc_2;
	
	return crc;
}


 
uint8_t modbus_rx_DataLength;
uint16_t abcd;
uint8_t UART_RX_Message_MODBUS(USART_Def * Uart)
{
	uint16_t i, j, len;
	uint8_t data;

	abcd = calculate_CRC(Uart->Rx_Buffer,modbus_rx_DataLength+5);
	if(abcd != 0)
		return 0;
	
	len = modbus_rx_DataLength+2;
	j = 0;
	for(i = 0; i < len; i++)
	{
		data = Uart->Rx_Buffer[i + 1];
		Uart->Rx_Data[j++] = data;
	};

	return 1;
}

void CopyData_MODBUS(USART_Def * Uart, uint16_t Length)
{
	uint16_t i, j,modbus_addr;
	uint8_t data;
	uint8_t RX_Start;
	
	
	RX_Start = 0;
	j = 0;
	
	for(i = 0; i<Length; i++)
	{
		data = Uart->DMA_Buffer[i];
		
		if(data == zigbee_adress_buf)
		{
			RX_Start = 1;		
		};
		
		if(RX_Start == 1)
		{
			Uart->Rx_Buffer[j++] = data;
		}			
	}
	//²»°üÀ¨´Ó»úµØzhi,crc,modbusµØÖ·
	modbus_rx_DataLength = j-5;
	
	if(RX_Start == 1)
	{
		if(j > 2)
		{
				if(UART_RX_Message_MODBUS(Uart) != 0)
				{
					Uart->Rx_Count++;
					modbus_addr = ((uint16_t)Uart->Rx_Data[1])<<8;
					modbus_addr += Uart->Rx_Data[2];
					if(Uart == &UART3)
						//command,²»°üÀ¨´Ó»úµØÖ·£¬crcµÄ³¤¶È£¬³ý´Ó»úµØÖ·£¬Ö¸ÁîÒÔÍâµÄÊý¾Ý
						UART3_RX_CMD_MODBUS(Uart->Rx_Data[0], modbus_rx_DataLength, modbus_addr, &Uart->Rx_Data[3]);
				}	
		}			
	}
}
/*******************************************************************************************************/
void USART_RX(USART_Def * Uart)
{
	uint32_t cnt;
	uint16_t t, len;
	DMA_Channel_TypeDef * DMA_Chan = 0;
	
	if(Uart == &UART1)
		DMA_Chan = DMA1_Channel5;
	else 	if(Uart == &UART3)
		DMA_Chan = DMA1_Channel3;
	
	cnt = DMA_Chan->CNDTR;
	
	if(cnt != Uart->DMA_CNDTR)
	{
		Uart->DMA_CNDTR = cnt;
		Uart->Rx_Timer = Global_time_ms;
	}
	else
	{
		t = Global_time_ms - Uart->Rx_Timer;
		if(t > 5)
		{
			if(DMA_Length > cnt)
			{
				len = DMA_Length - cnt;
				/**************************************************************/
				if(commu_model_flag==0||Uart == &UART1)
					CopyData(Uart, len);
        else
					CopyData_MODBUS(Uart,len);        					
				/****************************************************************/
			};
			
			if(Uart->DMA_CNDTR != DMA_Length)
			{
				DMA_Chan->CCR &= ~1;
				DMA_Chan->CMAR = (uint32_t)Uart->DMA_Buffer;
				DMA_Chan->CNDTR = DMA_Length;	
				Uart->DMA_CNDTR = DMA_Length;	
				DMA_Chan->CCR |= 1;				
			};
			
			Uart->Rx_Timer = (uint16_t)Global_time_ms - 5;
		}
	}
}

void USART_TXRX(void)
{
	USART_RX(&UART3);
	USART_RX(&UART1);	
}

//0x3A, Command, Length, data[Length], Lrc, summon from Command
//max of Length 256
//Uart_Ch 1 or 3

void UART_TX_Message(uint8_t Uart_Ch, uint8_t Command, uint16_t Length, uint8_t * data)
{
	uint16_t sum;
	uint16_t i, j = 0;
	uint8_t d;	
	USART_Def * Uart;
	
	if(Uart_Ch == 1)
		Uart = &UART1;
	else if(Uart_Ch == 3)
		Uart = &UART3;		
	else
		return;
	
	Uart->Tx_Buffer[j++] = XON;
	Uart->Tx_Buffer[j++] = Command;
	Uart->Tx_Buffer[j++] = Length;

	sum = Command + Length;

	i = 0;
	if(Length > 0)
	{
		for(i = 0; i<Length; i++)
		{
			d = *data++;
			sum += d;

			Uart->Tx_Buffer[j++] = d;
		};
	}

	sum = 0 - sum;

	Uart->Tx_Buffer[j++] = sum;
	
	if(Uart == &UART1)
	{
		DMA1_Channel4->CCR &= ~1;		
		DMA1_Channel4->CNDTR = j;
		DMA1_Channel4->CCR |= 1;
	}
	else 	if(Uart == &UART3)
	{
		DMA1_Channel2->CCR &= ~1;	
		DMA1_Channel2->CNDTR = j;
		DMA1_Channel2->CCR |= 1;
	}
}

extern uint8_t zigbee_adress_buf;
uint16_t sum_test;
void UART3_TX_Message_MODBUS( uint8_t Command, uint16_t modbus_addr, uint8_t length, uint8_t * data)
{
	uint16_t sum;
	uint16_t i, j = 0;
	USART_Def * Uart;

	Uart = &UART3;		
	
	Uart->Tx_Buffer[j++] = zigbee_adress_buf;
	Uart->Tx_Buffer[j++] = Command;
	Uart->Tx_Buffer[j++] = modbus_addr>>8;
  Uart->Tx_Buffer[j++] = modbus_addr;
	
  for(i=0;i<length;i++)
		Uart->Tx_Buffer[j++] = data[i];

	sum = calculate_CRC(Uart->Tx_Buffer,j);
	sum_test = sum;
	
	Uart->Tx_Buffer[j++] = sum>>8;
	Uart->Tx_Buffer[j++] = sum;
	
	DMA1_Channel2->CCR &= ~1;	
	DMA1_Channel2->CNDTR = j;
	DMA1_Channel2->CCR |= 1;
}

