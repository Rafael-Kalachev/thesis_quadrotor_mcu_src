/* This code shows how to use USART and send messages to RS232 module to PC */
#include "stm32f4xx.h"
#include "cmsis/inc/arm_math.h"

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
void USART_SendNumber(uint32_t x)
{
  char value[10]; //a temp array to hold results of conversion
  int i = 0; //loop index

  do
  {
    value[i++] = (char)(x % 10) + '0'; //convert integer to character
    x /= 10;
  } while(x);

  while(i) //send data
  {
    //USART_SendNumber8b(USARTx, value[--i]);
    while( !USART_GetFlagStatus(USART1, USART_FLAG_TXE) );
    USART_SendData(USART1, value[--i]);
  }
}


float32_t A_f32[10*10] = 
{
	1.0,	2.0,	3.0,	4.0,	5.0,	6.0,	7.0,	8.0,	9.0,	10.0,
	1.0,	2.0,	3.0,	4.0,	5.0,	6.0,	7.0,	8.0,	9.0,	10.0,
	1.0,	2.0,	3.0,	4.0,	5.0,	6.0,	7.0,	8.0,	9.0,	10.0,
	1.0,	2.0,	3.0,	4.0,	5.0,	6.0,	7.0,	8.0,	9.0,	10.0,
	1.0,	2.0,	3.0,	4.0,	5.0,	6.0,	7.0,	8.0,	9.0,	10.0,
	1.0,	2.0,	3.0,	4.0,	5.0,	6.0,	7.0,	8.0,	9.0,	10.0,
	1.0,	2.0,	3.0,	4.0,	5.0,	6.0,	7.0,	8.0,	9.0,	10.0,
	1.0,	2.0,	3.0,	4.0,	5.0,	6.0,	7.0,	8.0,	9.0,	10.0,
	1.0,	2.0,	3.0,	4.0,	5.0,	6.0,	7.0,	8.0,	9.0,	10.0,
	1.0,	2.0,	3.0,	4.0,	5.0,	6.0,	7.0,	8.0,	9.0,	10.0
};

float32_t B_f32[10*1] =
{
	10.0,
	9.0,
	8.0,
	7.0,
	6.0,
	5.0,
	4.0,
	3.0,
	2.0,
	1.0
};

float32_t C_f32[10*1];

int main(void)
{

  uint32_t* CPACR = (uint32_t*)0xE000ED88;

  *CPACR |=  0b00000000111100000000000000000000;

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




/*SETUP SPI for NRF..*/
/**Enable the clock to the SPI*/
RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
/*Enable the pins used by the SPI*/
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  // Init GPIOA
  GPIO_InitStruct.GPIO_Pin  = GPIO_Pin_4;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStruct);
  GPIO_SetBits(GPIOA,GPIO_Pin_4); /*set pin 4 HIGH so we disable CS for the SPI*/



  GPIO_InitStruct.GPIO_Pin  = /*GPIO_Pin_4 |*/ GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStruct);
  // Connect pins to AF pins of SPI
/*  GPIO_PinAFConfig(GPIOA, GPIO_PinSource4, GPIO_AF_SPI1);*/ /*DO NOT USE THE NSS pin drive it as GPIO*/
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);


SPI_InitTypeDef  spi_init_s;
spi_init_s.SPI_Direction	= SPI_Direction_2Lines_FullDuplex;
spi_init_s.SPI_Mode	= SPI_Mode_Master;
spi_init_s.SPI_DataSize = SPI_DataSize_8b;
spi_init_s.SPI_CPOL = SPI_CPOL_Low;
spi_init_s.SPI_CPHA = SPI_CPHA_1Edge;
spi_init_s.SPI_NSS	= SPI_NSS_Soft | SPI_NSSInternalSoft_Set ; 
spi_init_s.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
spi_init_s.SPI_FirstBit	= SPI_FirstBit_MSB;
spi_init_s.SPI_CRCPolynomial = 7;

SPI_Init(SPI1, &spi_init_s);
SPI_Cmd(SPI1, ENABLE);

/*SEND DATA SPI*/

/*drop_down pin4 to enable chip*/
GPIO_ResetBits(GPIOA, GPIO_Pin_4);
while ( !SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) );
SPI_SendData(SPI1, 0xFF ); /*NOP command*/ /**should at the same time recieve the status register*/
while ( !SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) );
uint16_t status = SPI_ReceiveData(SPI1);
/*close communication*/
GPIO_SetBits(GPIOA, GPIO_Pin_4);


	while(1) // this function can be called slower as you add data to be sent
	{

		USART_SendText("status =");
    USART_SendNumber(status);
    USART_SendText("\n\n");

    /*drop_down pin4 to enable chip*/
    GPIO_ResetBits(GPIOA, GPIO_Pin_4);
    while ( !SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) );
    SPI_SendData(SPI1, 0xFF ); /*NOP command*/ /**should at the same time recieve the status register*/
    while ( !SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) );
    status = SPI_ReceiveData(SPI1);
    /*close communication*/
    GPIO_SetBits(GPIOA, GPIO_Pin_4);

	}
}
