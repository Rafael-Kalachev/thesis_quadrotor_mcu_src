/* This code shows how to use USART and send messages to RS232 module to PC */
#include "stm32f4xx.h"
#include "cmsis/inc/arm_math.h"

void log_msg(volatile char *s)
{
	USART_SendText(s);
}


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


uint8_t buffer[15];

typedef enum I2C_IO_STATE_ENUM
{
	I2C_IO_STATE_NOT_INITIALIZED,
	I2C_IO_STATE_REGISTER_WRITE_START,
	I2C_IO_STATE_REGISTER_WRITE_SEND_ADDRESS,
	I2C_IO_STATE_REGISTER_WRITE_SEND_REGISTER_ADDRESS,
	I2C_IO_STATE_REGISTER_WRITE_SEND_RESTART,
	I2C_IO_STATE_REGISTER_WRITE_SEND_ADDRESS2,
	I2C_IO_STATE_REGISTER_WRITE_WRITE_BYTE,
	I2C_IO_STATE_REGISTER_WRITE_STOP,
	I2C_IO_STATE_REGISTER_READ_START,
	I2C_IO_STATE_REGISTER_READ_SEND_ADDRESS,
	I2C_IO_STATE_REGISTER_READ_SEND_REGISTER_ADDRESS,
	I2C_IO_STATE_REGISTER_READ_SEND_RESTART,
	I2C_IO_STATE_REGISTER_READ_SEND_ADDRESS2,
	I2C_IO_STATE_REGISTER_READ_READ_BYTE,
	I2C_IO_STATE_REGISTER_READ_STOP,
	I2C_IO_STATE_ERROR
} i2c_io_state_enum;

typedef struct I2C_IO_STRUCT
{
	__IO uint8_t *buffer;
	__IO uint8_t  address;
	__IO uint8_t  register_address;
	__IO uint8_t  bytes_count;
	__IO i2c_io_state_enum state;
} I2C_io_t;


I2C_io_t _i2c3_io_struct  = { 
	.buffer = NULL,
	.state  = I2C_IO_STATE_NOT_INITIALIZED };


int main(void)
{

  int a = 0;
  uint32_t* CPACR = (uint32_t*)0xE000ED88;

  *CPACR |=  0b00000000111100000000000000000000;
  __enable_irq();

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


	// enable i2c 
	I2C_InitTypeDef I2C_init_struct;
	I2C_StructInit(&I2C_init_struct);
	I2C_init_struct.I2C_Ack = I2C_Ack_Enable;
	I2C_init_struct.I2C_Mode = I2C_Mode_I2C;
	I2C_init_struct.I2C_ClockSpeed = 100000;
	I2C_Init(I2C3, &I2C_init_struct);


	volatile I2C_TypeDef* i2c3 = I2C3;
	i2c3->CR2 |= (0b111 << 8); /*enable interrupts*/
	//USART_SendNumber(1);

	I2C_Cmd(I2C3, ENABLE);
	//USART_SendNumber(1);


	// enable i2c interrupts
	NVIC_InitTypeDef nvic_i2c_init;
	nvic_i2c_init.NVIC_IRQChannel = I2C3_EV_IRQn;
	nvic_i2c_init.NVIC_IRQChannelCmd = ENABLE;
	nvic_i2c_init.NVIC_IRQChannelPreemptionPriority = 1;
	nvic_i2c_init.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&nvic_i2c_init);
	//USART_SendNumber(1);

        nvic_i2c_init.NVIC_IRQChannel = I2C3_ER_IRQn;
        nvic_i2c_init.NVIC_IRQChannelCmd = ENABLE;
        nvic_i2c_init.NVIC_IRQChannelPreemptionPriority = 1;
        nvic_i2c_init.NVIC_IRQChannelSubPriority = 0;
        NVIC_Init(&nvic_i2c_init);


 	int i; 
	uint8_t bytes = 14;
	while(1) // this function can be called slower as you add data to be sent
	{
//		buffer[0] = 0x55;
//		buffer[1] = 0x66;
//		_i2c3_io_struct.address =  0xD6;
//		_i2c3_io_struct.buffer = buffer;
//		_i2c3_io_struct.register_address =0x10;
//		_i2c3_io_struct.bytes_count = bytes;
//		_i2c3_io_struct.state = I2C_IO_STATE_REGISTER_WRITE_START;
//		I2C_GenerateSTART(I2C3,ENABLE);
//
//		while (_i2c3_io_struct.state != I2C_IO_STATE_REGISTER_WRITE_STOP);
//	
//		for( i = 0 ; i < bytes; ++i)
//		{
//			USART_SendText(">");
//			USART_SendNumber(buffer[i]);
//			USART_SendText("<");
//		}
//
//		USART_SendText("\n");

		_i2c3_io_struct.address =  0xD6;
		_i2c3_io_struct.register_address =0x20;
		_i2c3_io_struct.buffer = buffer;
		_i2c3_io_struct.bytes_count = bytes;
		_i2c3_io_struct.state = I2C_IO_STATE_REGISTER_READ_START;
		I2C_GenerateSTART(I2C3, ENABLE);

		while(_i2c3_io_struct.state != I2C_IO_STATE_REGISTER_READ_STOP);

		for( i = 0 ; i < bytes; ++i)
		{
			USART_SendText(">");
			USART_SendNumber(buffer[i]);
			USART_SendText("<");
		}

		for(a=10; a!=0; --a);

	}
}





void I2C3_EV_IRQHandler()
{
	__disable_irq();
	int tempreg;
	switch (_i2c3_io_struct.state)
	{
		case I2C_IO_STATE_NOT_INITIALIZED:
			log_msg("ERROR: I2C3 NOT INITIALIZED\n");
			_i2c3_io_struct.state = I2C_IO_STATE_ERROR;
			break;

		case I2C_IO_STATE_REGISTER_WRITE_START:
			if (I2C_GetFlagStatus(I2C3, I2C_FLAG_SB))
			{
				/*SR1 is read by the status function, therefore interrupt is lowered*/
				I2C_Send7bitAddress(I2C3,_i2c3_io_struct.address, I2C_Direction_Transmitter);
				_i2c3_io_struct.state = I2C_IO_STATE_REGISTER_WRITE_SEND_ADDRESS;
			}
			else
			{
				log_msg("ERROR: Expected a start bit condition in this state\n");
				_i2c3_io_struct.state = I2C_IO_STATE_ERROR;

			}
			break;

		case I2C_IO_STATE_REGISTER_WRITE_SEND_ADDRESS:
			if (I2C_GetFlagStatus(I2C3, I2C_FLAG_ADDR))
			{
				tempreg = I2C3->SR1;
				tempreg = I2C3->SR2;
				I2C_SendData(I2C3, _i2c3_io_struct.register_address);
				_i2c3_io_struct.state = I2C_IO_STATE_REGISTER_WRITE_WRITE_BYTE;
			}
			else
			{
				log_msg("ERROR: Expected ADDRESS_FLAG to be set\n");
				_i2c3_io_struct.state = I2C_IO_STATE_ERROR;
			}
			break;
		
		case I2C_IO_STATE_REGISTER_WRITE_SEND_REGISTER_ADDRESS:
			if (I2C_GetFlagStatus(I2C3, I2C_FLAG_TXE))
			{
				I2C_GenerateSTART(I2C3,ENABLE);
				_i2c3_io_struct.state = I2C_IO_STATE_REGISTER_WRITE_SEND_RESTART;
			}
			else 
			{
				log_msg("ERROR: Expected TXE to be empty after writing to register\n");
				_i2c3_io_struct.state = I2C_IO_STATE_ERROR;
			}
			break;
		

		case I2C_IO_STATE_REGISTER_WRITE_SEND_RESTART:
			if (I2C_GetFlagStatus(I2C3, I2C_FLAG_SB))
			{
				I2C_Send7bitAddress(I2C3,_i2c3_io_struct.address, I2C_Direction_Transmitter);
				_i2c3_io_struct.state = I2C_IO_STATE_REGISTER_WRITE_SEND_ADDRESS2;
			}
			else
			{
//				log_msg("ERROR: Expected SB to be set after restart\n");
//				_i2c3_io_struct.state = I2C_IO_STATE_ERROR;
			}
			break;
		
		case I2C_IO_STATE_REGISTER_WRITE_SEND_ADDRESS2:
			if (I2C_GetFlagStatus(I2C3, I2C_FLAG_ADDR))
			{
				tempreg = I2C3->SR1;
				tempreg = I2C3->SR2;
				_i2c3_io_struct.state = I2C_IO_STATE_REGISTER_WRITE_WRITE_BYTE;
			}
			else
			{
				log_msg("ERROR: Expected ADDRESS_FLAG to be set\n");
				_i2c3_io_struct.state = I2C_IO_STATE_ERROR;
			}
			break;

		case I2C_IO_STATE_REGISTER_WRITE_WRITE_BYTE:
			if (I2C_GetFlagStatus(I2C3, I2C_FLAG_TXE))
			{
				if (_i2c3_io_struct.bytes_count > 0)
				{
					I2C_SendData(I2C3, *_i2c3_io_struct.buffer);
					++_i2c3_io_struct.buffer;
					--_i2c3_io_struct.bytes_count;
				}
				else
				{
					I2C_GenerateSTOP(I2C3, ENABLE);
					_i2c3_io_struct.state = I2C_IO_STATE_REGISTER_WRITE_STOP;
					/*No interrupt shall be given*/
				}
			}
			else 
			{
				log_msg("ERROR: Expected TXE flag to be set for transmission\n");
				_i2c3_io_struct.state = I2C_IO_STATE_ERROR;
			}
			
			break;

		case I2C_IO_STATE_REGISTER_READ_START:
			if (I2C_GetFlagStatus(I2C3, I2C_FLAG_SB))
			{
				/*SR1 is read by the status function, therefore interrupt is lowered*/
				I2C3->CR1 |=  0b1 << 10; // ENABLE ACK			
				I2C_Send7bitAddress(I2C3,_i2c3_io_struct.address, I2C_Direction_Transmitter);
				_i2c3_io_struct.state = I2C_IO_STATE_REGISTER_READ_SEND_ADDRESS;
			}
			else
			{
				log_msg("ERROR: Expected a start bit condition in this state r\n");
				_i2c3_io_struct.state = I2C_IO_STATE_ERROR;

			}
			break;

		case I2C_IO_STATE_REGISTER_READ_SEND_ADDRESS:
			if (I2C_GetFlagStatus(I2C3, I2C_FLAG_ADDR))
			{
				tempreg = I2C3->SR1;
				tempreg = I2C3->SR2;
				I2C_SendData(I2C3, _i2c3_io_struct.register_address);
				_i2c3_io_struct.state = I2C_IO_STATE_REGISTER_READ_SEND_REGISTER_ADDRESS;
			}
			else
			{
				log_msg("ERROR: Expected ADDRESS_FLAG to be set\n");
				_i2c3_io_struct.state = I2C_IO_STATE_ERROR;
			}
			break;
		
		case I2C_IO_STATE_REGISTER_READ_SEND_REGISTER_ADDRESS:
			if (I2C_GetFlagStatus(I2C3, I2C_FLAG_TXE))
			{
				I2C_GenerateSTART(I2C3,ENABLE);
				_i2c3_io_struct.state = I2C_IO_STATE_REGISTER_READ_SEND_RESTART;
			}
			else 
			{
				log_msg("ERROR: Expected TXE to be empty after writing to register\n");
				_i2c3_io_struct.state = I2C_IO_STATE_ERROR;
			}
			break;
		

		case I2C_IO_STATE_REGISTER_READ_SEND_RESTART:
			if (I2C_GetFlagStatus(I2C3, I2C_FLAG_SB))
			{
				I2C_Send7bitAddress(I2C3,_i2c3_io_struct.address, I2C_Direction_Receiver);
				_i2c3_io_struct.state = I2C_IO_STATE_REGISTER_READ_SEND_ADDRESS2;
			}
			else
			{
				//log_msg("A");
				//log_msg("ERROR: Expected SB to be set after restart\n");
				//_i2c3_io_struct.state = I2C_IO_STATE_ERROR;
			}
			break;
		
		case I2C_IO_STATE_REGISTER_READ_SEND_ADDRESS2:
			if (I2C_GetFlagStatus(I2C3, I2C_FLAG_ADDR))
			{

				if (_i2c3_io_struct.bytes_count == 1)
				{
					I2C3->CR1 &= ~(0b1<<10);
				}

				_i2c3_io_struct.state = I2C_IO_STATE_REGISTER_READ_READ_BYTE;
				tempreg = I2C3->SR1;
				tempreg = I2C3->SR2;

			}
			else
			{
				log_msg("ERROR: Expected ADDRESS_FLAG to be set\n");
				_i2c3_io_struct.state = I2C_IO_STATE_ERROR;
			}
			break;

		case I2C_IO_STATE_REGISTER_READ_READ_BYTE:
			if (I2C_GetFlagStatus(I2C3, I2C_FLAG_RXNE))
			{
				//if (_i2c3_io_struct.bytes_count > 0)
				{

					*_i2c3_io_struct.buffer = I2C_ReceiveData(I2C3);
					++_i2c3_io_struct.buffer;
					--_i2c3_io_struct.bytes_count;

					if (_i2c3_io_struct.bytes_count == 1)
					{
						I2C3->CR1 &= ~(0b1<<10);
					}

					if (_i2c3_io_struct.bytes_count == 0)
					{
						I2C3->CR1 |= 0b1<<9;
						_i2c3_io_struct.state = I2C_IO_STATE_REGISTER_READ_STOP;
					}

				}
			}
			else 
			{
				log_msg("ERROR: Expected RXNE flag to be set for transmission\n");
				_i2c3_io_struct.state = I2C_IO_STATE_ERROR;
			}
			break;
		


		case I2C_IO_STATE_ERROR:
			break;
	}
	__enable_irq();
}


void I2C3_ER_IRQHandler()
{
	USART_SendText("i2c2_ER_handler");

}
