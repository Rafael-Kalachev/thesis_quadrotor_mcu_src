#include "stm32f4xx.h"
#include "m_project_specific/inc/usart.h"
#include "m_math/inc/convert.h"

// This functions send text or char array passed as argument over USART
void USART_SendText(volatile char *s)
{
  while(*s){
    // wait until data register is empty
    while( !USART_GetFlagStatus(USART1, USART_FLAG_TXE) );
    USART_SendData(USART1, *s);
    s++;
  }
}

// This function sends numbers up to 32bit over USART
void USART_SendNumber(int32_t x)
{
	char value[20]; //a temp array to hold results of conversion
	int count = int_to_str(x, value, 0);
	int i = 0;

	for(i = 0; i < count; ++i)
	{
		//USART_SendNumber8b(USARTx, value[--i]);
    	while( !USART_GetFlagStatus(USART1, USART_FLAG_TXE) );
    	USART_SendData(USART1, value[i]);
	}
}

// This function sends numbers up to 32bit over USART
void USART_SendFloat(float32_t x, uint8_t precision)
{
  char value[20]; //a temp array to hold results of conversion
  int count = float_to_str(x, value, precision, 0);
  int i = 0;

  for(i = 0; i < count; ++i)
  {
    //USART_SendNumber8b(USARTx, value[--i]);
    while( !USART_GetFlagStatus(USART1, USART_FLAG_TXE) );
    USART_SendData(USART1, value[i]);
  }
}


void USART_project_specific_init()
{
    // Init clock to GPIOB for USART
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  // Init GPIOB
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.GPIO_Pin  = GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIO_InitStruct);
  // Connect pins to AF pins of USART
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);

  // Init clock for USART peripheral
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
  // Init USART
  USART_InitTypeDef USART_InitStruct;
  USART_InitStruct.USART_BaudRate = 115200;
  USART_InitStruct.USART_WordLength = USART_WordLength_8b;
  USART_InitStruct.USART_StopBits = USART_StopBits_1;
  USART_InitStruct.USART_Parity = USART_Parity_No ;
  USART_InitStruct.USART_Mode = USART_Mode_Tx;
  USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_Init(USART1, &USART_InitStruct);
  USART_Cmd(USART1, ENABLE); // Start USART1 peripheral
}


