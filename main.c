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


void i2c_send_data(char data)
{
 
  while(!I2C_GetFlagStatus(I2C3, I2C_FLAG_TXE));
  I2C_SendData(I2C3, data);
//  USART_SendText("a");
}

int i2c_recieve_data()
{ 
  int a;
  while(!I2C_GetFlagStatus(I2C3, I2C_FLAG_RXNE));
  a = I2C_ReceiveData(I2C3);
  I2C3->CR1 &= ~(0b1<<10);
  return a;
}

int i2c_recieve_last_data()
{
  int a;

  while(!I2C_GetFlagStatus(I2C3, I2C_FLAG_RXNE));
  I2C3->CR1 |= 0b1<<9;
  a = I2C_ReceiveData(I2C3);
  return a;
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

  int a = 0;
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
  USART_InitStruct.USART_BaudRate = 9600;
  USART_InitStruct.USART_WordLength = USART_WordLength_8b;
  USART_InitStruct.USART_StopBits = USART_StopBits_1;
  USART_InitStruct.USART_Parity = USART_Parity_No ;
  USART_InitStruct.USART_Mode = USART_Mode_Tx;
  USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_Init(USART1, &USART_InitStruct);
  USART_Cmd(USART1, ENABLE); // Start USART1 peripheral

  USART_SendNumber(1);

  //I2C init
  // init clock for i2c
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C3, ENABLE);

  //enable the GPIOS for i2c
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  // Init GPIOB
  GPIO_InitStruct.GPIO_Pin  = GPIO_Pin_8;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStruct);
  // Connect pins to AF pins of USART
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_I2C3);

  //enable the GPIOS for i2c
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  // Init GPIOB
  GPIO_InitStruct.GPIO_Pin  = GPIO_Pin_9;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOC, &GPIO_InitStruct);
  // Connect pins to AF pins of USART
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_I2C3);


  USART_SendNumber(2);
  I2C_InitTypeDef I2C_init_struct;
  I2C_StructInit(&I2C_init_struct);
  I2C_init_struct.I2C_Ack = I2C_Ack_Enable;
  I2C_init_struct.I2C_Mode = I2C_Mode_I2C;

  I2C_Init(I2C3, &I2C_init_struct);
  I2C_Cmd(I2C3, ENABLE);
  USART_SendNumber(3);
  
  volatile I2C_TypeDef* i2c3 = I2C3;
  /*i2c3->CR2 |= (0b111 << 8);*/

  
	while(1) // this function can be called slower as you add data to be sent
	{
  I2C_GenerateSTART(I2C3,ENABLE);
    USART_SendNumber(3);
  while( !I2C_GetFlagStatus(I2C3, I2C_FLAG_SB));	
//  USART_SendNumber(4);
  I2C_Send7bitAddress(I2C3,0xD6, I2C_Direction_Transmitter);
//  USART_SendNumber(5);
  while(! I2C_GetFlagStatus(I2C3, I2C_FLAG_ADDR));
//  USART_SendText(">>");
  a = i2c3->SR1;
  a = i2c3->SR2;
  i2c_send_data(0x0F);
//  USART_SendNumber(8);
  while(! I2C_GetFlagStatus(I2C3, I2C_FLAG_TXE));
  I2C_GenerateSTART(I2C3,ENABLE);
//  USART_SendNumber(18);
  while(!I2C_GetFlagStatus(I2C3, I2C_FLAG_SB));
  I2C_Send7bitAddress(I2C3,0xD6, I2C_Direction_Receiver);
//  USART_SendNumber(9);
  while(! I2C_GetFlagStatus(I2C3, I2C_FLAG_ADDR));
  a = i2c3->SR1;
  a = i2c3->SR2;
  a = i2c_recieve_data();
//  USART_SendNumber(a);
  
  
  a = i2c_recieve_last_data();
//  USART_SendNumber(a);
  while(! I2C_GetFlagStatus(I2C3, I2C_FLAG_RXNE));


  USART_SendNumber(a);
  USART_SendText("\n");

  for(a=100000000; a!=0; --a);


	}
}





void I2C2_EV_IRQHandler()
{
	USART_SendText("i2c2_EV_handler");
  I2C_GetFlagStatus(I2C3, I2C_FLAG_SB);
}


void I2C2_ER_IRQHandler()
{
	USART_SendText("i2c2_ER_handler");

}
