/* This code shows how to use USART and send messages to RS232 module to PC */
#include "stm32f4xx.h"
#include "cmsis/inc/arm_math.h"
#include "m_math/inc/convert.h"
#include "m_project_specific/inc/usart.h"
#include "m_project_specific/inc/extended_kalman_filter.h"

#define PINS   ( GPIO_Pin_0 |  GPIO_Pin_1 |  GPIO_Pin_4 | GPIO_Pin_5)
#define GPIO_PORT  (GPIOB)

void log_msg(volatile char *s)
{
	USART_SendText(s);
}


float32_t A_f32[3*3] = 
{
	 4.0 , -2.0 ,  1.0,
	 5.0 ,  0.0 ,  3.0,
	-1.0 ,  2.0 ,  6.0
};

float32_t A_inv_f32[3*3] =
{
	1,1,1,
	1,1,1,
	1,1,1
};


uint8_t buffer[15];

float32_t inv_int16_max = (float32_t) 1 / (float32_t) INT16_MAX;


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
  __enable_irq();

  // Init clock to GPIOB for USART
  // Init GPIOB
  USART_project_specific_init();
  USART_SendNumber(1);

  //I2C init
  GPIO_InitTypeDef GPIO_InitStruct;
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
	uint8_t bytes = 16;

	// CONFIGURE ACC GYRO   FIFO
	_i2c3_io_struct.address =  0xD6;
	_i2c3_io_struct.register_address =0x06;
	buffer[0] = 0x00; //addr: 0x06
	buffer[1] = 0x00; //addr: 0x07
	buffer[2] = 0x00; //addr: 0x08
	buffer[3] = 0x00; //addr: 0x09
	buffer[4] = 0x00; //addr: 0x0A
	buffer[5] = 0x00; //addr: 0x0B
	_i2c3_io_struct.buffer = buffer;
	_i2c3_io_struct.bytes_count = 6;
	_i2c3_io_struct.state = I2C_IO_STATE_REGISTER_WRITE_START;
	I2C_GenerateSTART(I2C3, ENABLE);

	while(_i2c3_io_struct.state != I2C_IO_STATE_REGISTER_WRITE_STOP);

	// CONFIGURE ACC GYRO  PARAMETERS
	_i2c3_io_struct.address =  0xD6;
	_i2c3_io_struct.register_address =0x10;
	buffer[0] = 0x3B; //addr: 0x10 (ACCELEROMETER samp_rate=52Hz, full_scale=+-4g, filter=50Hz)
	buffer[1] = 0x34; //addr: 0x11 (GYRO samp_rate=52Hz, deg_per_second=500)
	buffer[2] = 0x04; //addr: 0x12 (DEFAULT)
	buffer[3] = 0x00; //addr: 0x13 (DEFAULT)
	buffer[4] = 0x00; //addr: 0x14 (DEFAULT)
	buffer[5] = 0x00; //addr: 0x15 (DEFAULT)
	buffer[6] = 0x00; //addr: 0x16 (DEFAULT)
	buffer[7] = 0x00; //addr: 0x17 (DEFAULT)
	buffer[8] = 0x38; //addr: 0x18 (DEFAULT)
	buffer[9] = 0x38; //addr: 0x19 (DEFAULT)
	_i2c3_io_struct.buffer = buffer;
	_i2c3_io_struct.bytes_count = 10;
	_i2c3_io_struct.state = I2C_IO_STATE_REGISTER_WRITE_START;
	I2C_GenerateSTART(I2C3, ENABLE);


	while(_i2c3_io_struct.state != I2C_IO_STATE_REGISTER_WRITE_STOP);

	// CONFIGURE GPIO
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);


	GPIO_InitStruct.GPIO_Pin  = PINS;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;

	GPIO_Init(GPIO_PORT, &GPIO_InitStruct);	

	// CONFIGURE TIMER

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
 
	TIM_TimeBaseInitTypeDef timerInitStructure; 
	timerInitStructure.TIM_Prescaler = 83;
	timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	timerInitStructure.TIM_Period = 20000;
	timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	timerInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2, &timerInitStructure);

	//	ENABLE IRQ TIM

	NVIC_InitTypeDef nvicStructure;
	nvicStructure.NVIC_IRQChannel = TIM2_IRQn;
	nvicStructure.NVIC_IRQChannelPreemptionPriority = 0;
	nvicStructure.NVIC_IRQChannelSubPriority = 1;
	nvicStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvicStructure);

	TIM_OCInitTypeDef oc_init;
	oc_init.TIM_OCMode = TIM_OCMode_Timing;
	oc_init.TIM_OCPolarity = TIM_OCPolarity_High;
	oc_init.TIM_OutputState = TIM_OutputState_Enable;
	oc_init.TIM_Pulse = 2000;
	

	TIM_OC1Init(TIM2, &oc_init );
	

	TIM_ITConfig(TIM2, TIM_IT_Update | TIM_IT_CC1 , ENABLE);

	//	ENABLE TIMER 
	TIM_Cmd(TIM2, ENABLE);


	// MAIN LOOP
	arm_matrix_instance_f32 A;
	A.numCols = 3;
	A.numRows = 3;
	A.pData = A_f32;
	
	arm_matrix_instance_f32 A_inv;
	A_inv.numCols = 3;
	A_inv.numRows = 3;
	A_inv.pData = A_inv_f32;
	
	arm_mat_inverse_f32(&A, &A_inv);

	USART_SendMatrix(&A, 3);
	USART_SendMatrix(&A_inv, 3);

	USART_SendText(">");
	ekf_init();
	USART_SendText("<\n");


	while(1) // this function can be called slower as you add data to be sent
	{
		
		// DATA GATHER 
		
		_i2c3_io_struct.address =  0xD6;
		_i2c3_io_struct.register_address =0x20;
		_i2c3_io_struct.buffer = buffer;
		_i2c3_io_struct.bytes_count = 14;
		_i2c3_io_struct.state = I2C_IO_STATE_REGISTER_READ_START;
		I2C_GenerateSTART(I2C3, ENABLE);

		while(_i2c3_io_struct.state != I2C_IO_STATE_REGISTER_READ_STOP);

		float32_t temperature = ((int16_t)((int32_t) buffer[0]  + ((int32_t) buffer[1]  << 8 ))) * inv_int16_max;
		float32_t gyro_x      = ((int16_t)((int32_t) buffer[2]  + ((int32_t) buffer[3]  << 8 ))) * inv_int16_max;
		float32_t gyro_y      = ((int16_t)((int32_t) buffer[4]  + ((int32_t) buffer[5]  << 8 ))) * inv_int16_max;
		float32_t gyro_z      = ((int16_t)((int32_t) buffer[6]  + ((int32_t) buffer[7]  << 8 ))) * inv_int16_max;
		float32_t acc_x	      = ((int16_t)((int32_t) buffer[8]  + ((int32_t) buffer[9]  << 8 ))) * inv_int16_max;
		float32_t acc_y	      = ((int16_t)((int32_t) buffer[10] + ((int32_t) buffer[11] << 8 ))) * inv_int16_max;
		float32_t acc_z	      = ((int16_t)((int32_t) buffer[12] + ((int32_t) buffer[13] << 8 ))) * inv_int16_max;

		USART_SendText("temp=");
		USART_SendFloat(temperature, 5);

		USART_SendText(" , gyro_x=");
		USART_SendFloat(gyro_x, 5);

		USART_SendText(" , gyro_y=");
		USART_SendFloat(gyro_y, 5);

		USART_SendText(" , gyro_z=");
		USART_SendFloat(gyro_z, 5);

		USART_SendText(" , acc_x=");
		USART_SendFloat(acc_x, 5);

		USART_SendText(" , acc_y=");
		USART_SendFloat(acc_y, 5);

		USART_SendText(" , acc_z=");
		USART_SendFloat(acc_z, 5);

		USART_SendText("\n");

		
		/*
		GPIO_SetBits(GPIO_PORT, PINS );
		for(a=0; a < 5000000; ++a)
		{
			__NOP;
		}
		GPIO_ResetBits(GPIO_PORT, PINS );

		for(a=0; a < 1000000; ++a)
		{
			__NOP;
		}
		*/

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
	log_msg("i2c2_ER_handler\n");

}

void TIM2_IRQHandler()
{
	__disable_irq();
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		GPIO_ToggleBits(GPIO_PORT, PINS);
	}

	if(TIM_GetITStatus(TIM2, TIM_IT_CC1) != RESET)
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);
		GPIO_ToggleBits(GPIO_PORT, PINS);
	}
	__enable_irq();
}
