/* This code shows how to use USART and send messages to RS232 module to PC */
#include "stm32f4xx.h"
#include "cmsis/inc/arm_math.h"
#include "m_math/inc/convert.h"
#include "m_math/inc/polynomial.h"
#include "m_project_specific/inc/usart.h"
#include "m_project_specific/inc/extended_kalman_filter.h"
#include "m_project_specific/inc/read_joystick.h"
#include "m_project_specific/inc/calibrate_sensors.h"
#include <math.h>

#define PINS   ( GPIO_Pin_1 |  GPIO_Pin_3 |  GPIO_Pin_5 | GPIO_Pin_7)
#define GPIO_PORT  (GPIOD)


#define DISABLE_LOGGING

#ifndef DISABLE_LOGGING
	#define log_msg(ARG) __int_log_msg(ARG)
#else
	#define log_msg(ARG) do{}while(0)
#endif




/* This function sends data over SPI1 peripheral to a certain register address */
void SPI_Tx(uint8_t address, uint8_t data)
{
  GPIO_ResetBits(GPIOE, GPIO_Pin_3);

  while(!SPI_I2S_GetFlagStatus(SPI4, SPI_I2S_FLAG_TXE));
  SPI_I2S_SendData(SPI4, address);
  while(!SPI_I2S_GetFlagStatus(SPI4, SPI_I2S_FLAG_RXNE));
  SPI_I2S_ReceiveData(SPI4);
  while(!SPI_I2S_GetFlagStatus(SPI4, SPI_I2S_FLAG_TXE));
  SPI_I2S_SendData(SPI4, data);
  while(!SPI_I2S_GetFlagStatus(SPI4, SPI_I2S_FLAG_RXNE));
  SPI_I2S_ReceiveData(SPI4);

  GPIO_SetBits(GPIOE, GPIO_Pin_3);
}

/* This function returns data from a certain address over SPI1 peripheral */
uint8_t SPI_Rx(uint8_t address)
{
  GPIO_ResetBits(GPIOE, GPIO_Pin_3);
  address = 0x00 | address; // this part can be different for your SPI device (this works only on nrf..)
  while(!SPI_I2S_GetFlagStatus(SPI4, SPI_I2S_FLAG_TXE));
  SPI_I2S_SendData(SPI4, address);
  while(!SPI_I2S_GetFlagStatus(SPI4, SPI_I2S_FLAG_RXNE));
  SPI_I2S_ReceiveData(SPI4);
  while(!SPI_I2S_GetFlagStatus(SPI4, SPI_I2S_FLAG_TXE));
  SPI_I2S_SendData(SPI4, 0x00);
  while(!SPI_I2S_GetFlagStatus(SPI4, SPI_I2S_FLAG_RXNE));

  GPIO_SetBits(GPIOE, GPIO_Pin_3);

  return SPI_I2S_ReceiveData(SPI4);
}






void __int_log_msg(volatile char *s)
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


uint8_t buffer[16];

float32_t inv_int16_max = (float32_t) 1 / (float32_t) INT16_MAX;


float32_t p_x_coeaf_buffer[] = {1 /*x^0*/, 1 /*x_1*/, 1 /*x^2*/};





 
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
	__IO uint8_t  is_in_irq;
} I2C_io_t;


I2C_io_t _i2c3_io_struct  = { 
	.buffer = NULL,
	.state  = I2C_IO_STATE_NOT_INITIALIZED,
	.is_in_irq = 0};

I2C_io_t _i2c3_io_struct_stage  = { 
	.buffer = NULL,
	.state  = I2C_IO_STATE_NOT_INITIALIZED };

void I2C_load(I2C_io_t* i2c_stage_struct)
{
	while ( _i2c3_io_struct.is_in_irq ||
	 !(	_i2c3_io_struct.state == I2C_IO_STATE_REGISTER_READ_STOP ||
	 	_i2c3_io_struct.state == I2C_IO_STATE_REGISTER_WRITE_STOP ||
	 	_i2c3_io_struct.state == I2C_IO_STATE_ERROR || 
		_i2c3_io_struct.state == I2C_IO_STATE_NOT_INITIALIZED ) );
	
	_i2c3_io_struct.buffer = i2c_stage_struct->buffer;
	_i2c3_io_struct.address = i2c_stage_struct->address;
	_i2c3_io_struct.register_address = i2c_stage_struct->register_address;
	_i2c3_io_struct.bytes_count = i2c_stage_struct->bytes_count;
	_i2c3_io_struct.state = i2c_stage_struct->state;
}


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
	nvic_i2c_init.NVIC_IRQChannelPreemptionPriority = 3;
	nvic_i2c_init.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&nvic_i2c_init);
	//USART_SendNumber(1);

	nvic_i2c_init.NVIC_IRQChannel = I2C3_ER_IRQn;
	nvic_i2c_init.NVIC_IRQChannelCmd = ENABLE;
	nvic_i2c_init.NVIC_IRQChannelPreemptionPriority = 3;
	nvic_i2c_init.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&nvic_i2c_init);


 	int i; 
	uint8_t bytes = 16;

	// CONFIGURE ACC GYRO   FIFO
	_i2c3_io_struct_stage.address =  0xD6;
	_i2c3_io_struct_stage.register_address =0x06;
	buffer[0] = 0x00; //addr: 0x06
	buffer[1] = 0x00; //addr: 0x07
	buffer[2] = 0x00; //addr: 0x08
	buffer[3] = 0x00; //addr: 0x09
	buffer[4] = 0x00; //addr: 0x0A
	buffer[5] = 0x00; //addr: 0x0B
	_i2c3_io_struct_stage.buffer = buffer;
	_i2c3_io_struct_stage.bytes_count = 6;
	_i2c3_io_struct_stage.state = I2C_IO_STATE_REGISTER_WRITE_START;
	I2C_load(&_i2c3_io_struct_stage);
	I2C_GenerateSTART(I2C3, ENABLE);

	while(_i2c3_io_struct.state != I2C_IO_STATE_REGISTER_WRITE_STOP);

	// CONFIGURE ACC GYRO  PARAMETERS
	_i2c3_io_struct_stage.address =  0xD6;
	_i2c3_io_struct_stage.register_address =0x10;
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
	_i2c3_io_struct_stage.buffer = buffer;
	_i2c3_io_struct_stage.bytes_count = 10;
	_i2c3_io_struct_stage.state = I2C_IO_STATE_REGISTER_WRITE_START;
	I2C_load(&_i2c3_io_struct_stage);
	I2C_GenerateSTART(I2C3, ENABLE);


	while(_i2c3_io_struct.state != I2C_IO_STATE_REGISTER_WRITE_STOP);


		// CONFIGURE MAGNETOMETER  PARAMETERS
	_i2c3_io_struct_stage.address =  0x3C;
	_i2c3_io_struct_stage.register_address =0x20;
	buffer[0] = 0b00011100; //addr: 0x20 (NO_TEMP_SENS, OM=ULP, ODR=80Hz)
	buffer[1] = 0b00100000; //addr: 0x21 ( FS=+-8gauss, )
	buffer[2] = 0b00000000; //addr: 0x22 (LP=NO, SIM=4wire, OM=continious)
	buffer[3] = 0b00000000; //addr: 0x23 (OMZ=LP, BLE=LSbLower)
	buffer[4] = 0b00000000; //addr: 0x24 (DEFAULT)
	_i2c3_io_struct_stage.buffer = buffer;
	_i2c3_io_struct_stage.bytes_count = 5;
	_i2c3_io_struct_stage.state = I2C_IO_STATE_REGISTER_WRITE_START;
	I2C_load(&_i2c3_io_struct_stage);
	I2C_GenerateSTART(I2C3, ENABLE);


	while(_i2c3_io_struct.state != I2C_IO_STATE_REGISTER_WRITE_STOP);










	// CONFIGURE GPIO
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);


	GPIO_InitStruct.GPIO_Pin  = PINS;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD; //TODO change to OD later for 5V 
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;

	GPIO_Init(GPIO_PORT, &GPIO_InitStruct);	

	// CONFIGURE TIMER OC

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
 
	TIM_TimeBaseInitTypeDef timerInitStructure2; 
	timerInitStructure2.TIM_Prescaler = 83;
	timerInitStructure2.TIM_CounterMode = TIM_CounterMode_Up;
	timerInitStructure2.TIM_Period = 20000;
	timerInitStructure2.TIM_ClockDivision = TIM_CKD_DIV1;
	timerInitStructure2.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2, &timerInitStructure2);

	//	ENABLE IRQ TIM

	NVIC_InitTypeDef nvicStructure2;
	nvicStructure2.NVIC_IRQChannel = TIM2_IRQn;
	nvicStructure2.NVIC_IRQChannelPreemptionPriority = 0;
	nvicStructure2.NVIC_IRQChannelSubPriority = 0;
	nvicStructure2.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvicStructure2);

	TIM_OCInitTypeDef oc_init;
	oc_init.TIM_OCMode = TIM_OCMode_Timing;
	oc_init.TIM_OCPolarity = TIM_OCPolarity_High;
	oc_init.TIM_OutputState = TIM_OutputState_Enable;
	oc_init.TIM_Pulse = 2000;
	
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);

	TIM_OC1Init(TIM2, &oc_init );
	TIM_OC2Init(TIM2, &oc_init );
	TIM_OC3Init(TIM2, &oc_init );
	TIM_OC4Init(TIM2, &oc_init );


	TIM2->CCR1 = 900;
	TIM2->CCR2 = 900;
	TIM2->CCR3 = 900;
	TIM2->CCR4 = 900;
	

	TIM_ITConfig(TIM2, TIM_IT_Update | TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4 , ENABLE);

	//	ENABLE TIMER 	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5);

	TIM_Cmd(TIM2, ENABLE);












	// CONFIGURE TIMER INPUT CAPTURE

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);




	// CONFIGURE TIMER IC

 
	TIM_TimeBaseInitTypeDef timerInitStructure; 
	timerInitStructure.TIM_Prescaler = 419; /*TEST*/
	timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	timerInitStructure.TIM_Period = 0xFFFF;
	timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	timerInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM5, &timerInitStructure);

	//	ENABLE IRQ TIM

	NVIC_InitTypeDef nvicStructure;
	nvicStructure.NVIC_IRQChannel = TIM5_IRQn;
	nvicStructure.NVIC_IRQChannelPreemptionPriority = 2;
	nvicStructure.NVIC_IRQChannelSubPriority = 0;
	nvicStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvicStructure);
	
	TIM_ICInitTypeDef tim_ic;
	tim_ic.TIM_Channel = TIM_Channel_1;
	tim_ic.TIM_ICPolarity = TIM_ICPolarity_Rising;
	tim_ic.TIM_ICSelection = TIM_ICSelection_DirectTI;
	tim_ic.TIM_ICFilter = 0x0;
	tim_ic.TIM_ICPrescaler = TIM_ICPSC_DIV1;

	tim_ic.TIM_Channel = TIM_Channel_1;
	TIM_ICInit(TIM5, &tim_ic);

	tim_ic.TIM_Channel = TIM_Channel_4;
	TIM_ICInit(TIM5, &tim_ic);

	TIM_ITConfig(TIM5,  TIM_IT_CC1 | TIM_IT_CC4 , ENABLE);
	
	TIM_Cmd(TIM5, ENABLE);


	GPIO_InitStruct.GPIO_Pin  = GPIO_Pin_0 | GPIO_Pin_3;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM5);


	GPIO_Init(GPIOA, &GPIO_InitStruct);	




	// TIM3

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	timerInitStructure.TIM_Prescaler = 20; 
	timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	timerInitStructure.TIM_Period = 0xFFFF;
	timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	timerInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3, &timerInitStructure);


	nvicStructure.NVIC_IRQChannel = TIM3_IRQn;
	nvicStructure.NVIC_IRQChannelPreemptionPriority = 2;
	nvicStructure.NVIC_IRQChannelSubPriority = 1;
	nvicStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvicStructure);

	tim_ic.TIM_Channel = TIM_Channel_1;
	tim_ic.TIM_ICPolarity = TIM_ICPolarity_Rising;
	tim_ic.TIM_ICSelection = TIM_ICSelection_DirectTI;
	tim_ic.TIM_ICFilter = 0x0;
	tim_ic.TIM_ICPrescaler = TIM_ICPSC_DIV1;

	tim_ic.TIM_Channel = TIM_Channel_1;
	TIM_ICInit(TIM3, &tim_ic);

	tim_ic.TIM_Channel = TIM_Channel_2;
	TIM_ICInit(TIM3, &tim_ic);

	tim_ic.TIM_Channel = TIM_Channel_3;
	TIM_ICInit(TIM3, &tim_ic);

	tim_ic.TIM_Channel = TIM_Channel_4;
	TIM_ICInit(TIM3, &tim_ic);

	TIM_ITConfig(TIM3,  TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4 , ENABLE);

	TIM_Cmd(TIM3, ENABLE);


	GPIO_InitStruct.GPIO_Pin  = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;


	GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_TIM3);

	GPIO_Init(GPIOB, &GPIO_InitStruct);




	// SPI

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI4, ENABLE);


	GPIO_InitStruct.GPIO_Pin  = GPIO_Pin_2 | GPIO_Pin_5 | GPIO_Pin_6;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOE, &GPIO_InitStruct);

	// Connect pins to AF pins of SPI1
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource2, GPIO_AF_SPI4);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource5, GPIO_AF_SPI4);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource6, GPIO_AF_SPI4);

	// Setup GPIOE pin for chip select for onboard SPI device
	GPIO_InitStruct.GPIO_Pin  = GPIO_Pin_3;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_Init(GPIOE, &GPIO_InitStruct);
	GPIO_SetBits(GPIOE, GPIO_Pin_3);

	SPI_InitTypeDef SPI_InitStruct;

	SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStruct.SPI_NSS = SPI_NSS_Soft | SPI_NSSInternalSoft_Set;
	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStruct.SPI_CRCPolynomial = 7;
	SPI_Init(SPI4, &SPI_InitStruct);
	SPI_Cmd(SPI4, ENABLE);



	for(a=0; a < 50000000; ++a)
	{
		__NOP;
	}
	TIM2->CCR1 = 960;
	




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


	raw_sensors_struct raw_sensors_data;


	while(1) // this function can be called slower as you add data to be sent
	{
		// DATA GATHER 
		
		_i2c3_io_struct_stage.address =  0xD6;
		_i2c3_io_struct_stage.register_address =0x20;
		_i2c3_io_struct_stage.buffer = buffer;
		_i2c3_io_struct_stage.bytes_count = 14;
		_i2c3_io_struct_stage.state = I2C_IO_STATE_REGISTER_READ_START;
		I2C_load(&_i2c3_io_struct_stage);
		I2C_GenerateSTART(I2C3, ENABLE);

		while(!(_i2c3_io_struct.state == I2C_IO_STATE_REGISTER_READ_STOP || _i2c3_io_struct.state == I2C_IO_STATE_ERROR));

		float32_t temperature = ((int16_t)((int32_t) buffer[0]  + ((int32_t) buffer[1]  << 8 ))) /** inv_int16_max*/;
		float32_t gyro_x      = ((int16_t)((int32_t) buffer[2]  + ((int32_t) buffer[3]  << 8 ))) * inv_int16_max;
		float32_t gyro_y      = ((int16_t)((int32_t) buffer[4]  + ((int32_t) buffer[5]  << 8 ))) * inv_int16_max;
		float32_t gyro_z      = ((int16_t)((int32_t) buffer[6]  + ((int32_t) buffer[7]  << 8 ))) * inv_int16_max;
		float32_t acc_x	      = ((int16_t)((int32_t) buffer[8]  + ((int32_t) buffer[9]  << 8 ))) * inv_int16_max;
		float32_t acc_y	      = ((int16_t)((int32_t) buffer[10] + ((int32_t) buffer[11] << 8 ))) * inv_int16_max;
		float32_t acc_z	      = ((int16_t)((int32_t) buffer[12] + ((int32_t) buffer[13] << 8 ))) * inv_int16_max;

		raw_sensors_data.temperature = ((int16_t)((int32_t) buffer[0]  + ((int32_t) buffer[1]  << 8 )));
		raw_sensors_data.gyro_x      = ((int16_t)((int32_t) buffer[2]  + ((int32_t) buffer[3]  << 8 )));
		raw_sensors_data.gyro_y      = ((int16_t)((int32_t) buffer[4]  + ((int32_t) buffer[5]  << 8 )));
		raw_sensors_data.gyro_z      = ((int16_t)((int32_t) buffer[6]  + ((int32_t) buffer[7]  << 8 )));
		raw_sensors_data.acc_x       = ((int16_t)((int32_t) buffer[8]  + ((int32_t) buffer[9]  << 8 )));
		raw_sensors_data.acc_y       = ((int16_t)((int32_t) buffer[10] + ((int32_t) buffer[11] << 8 )));
		raw_sensors_data.acc_z       = ((int16_t)((int32_t) buffer[12] + ((int32_t) buffer[13] << 8 )));


		_i2c3_io_struct_stage.address =  0x3C;
		_i2c3_io_struct_stage.register_address =0x28;
		_i2c3_io_struct_stage.buffer = buffer;
		_i2c3_io_struct_stage.bytes_count = 6;
		_i2c3_io_struct_stage.state = I2C_IO_STATE_REGISTER_READ_START;
		I2C_load(&_i2c3_io_struct_stage);
		I2C_GenerateSTART(I2C3, ENABLE);

		while(!(_i2c3_io_struct.state == I2C_IO_STATE_REGISTER_READ_STOP || _i2c3_io_struct.state == I2C_IO_STATE_ERROR));

		float32_t mag_x	      = ((int16_t)((int32_t) buffer[0]  + ((int32_t) buffer[1]  << 8 ))) * inv_int16_max;
		float32_t mag_y	      = ((int16_t)((int32_t) buffer[2]  + ((int32_t) buffer[3]  << 8 ))) * inv_int16_max;
		float32_t mag_z	      = ((int16_t)((int32_t) buffer[4]  + ((int32_t) buffer[5]  << 8 ))) * inv_int16_max;

		raw_sensors_data.mag_x = ((int16_t)((int32_t) buffer[0]  + ((int32_t) buffer[1]  << 8 )));
		raw_sensors_data.mag_y = ((int16_t)((int32_t) buffer[2]  + ((int32_t) buffer[3]  << 8 )));
		raw_sensors_data.mag_z = ((int16_t)((int32_t) buffer[4]  + ((int32_t) buffer[5]  << 8 )));
		
		calibrated_sensors_struct* calibrated_sensors_p = calibrate_sensors(&raw_sensors_data);

		

		USART_SendText("temp=");
		USART_SendFloat(temperature, 5);

		USART_SendText(" , gyro_x=");
		USART_SendFloat(calibrated_sensors_p->gyro_x, 5);

		USART_SendText(" , gyro_y=");
		USART_SendFloat(calibrated_sensors_p->gyro_y, 5);

		USART_SendText(" , gyro_z=");
		USART_SendFloat(calibrated_sensors_p->gyro_z, 5);

		USART_SendText(" , acc_x=");
		USART_SendFloat(calibrated_sensors_p->acc_x, 5);

		USART_SendText(" , acc_y=");
		USART_SendFloat(calibrated_sensors_p->acc_y, 5);

		USART_SendText(" , acc_z=");
		USART_SendFloat(calibrated_sensors_p->acc_z, 5);
		
		USART_SendText(" ,  mag_x=");
		USART_SendFloat(calibrated_sensors_p->mag_x, 5);

		USART_SendText(" , mag_y=");
		USART_SendFloat(calibrated_sensors_p->mag_y, 5);

		USART_SendText(" , mag_z=");
		USART_SendFloat(calibrated_sensors_p->mag_z, 5);

		USART_SendText("\n");


			float32_t p51 = tim5_ch1.period;
			float32_t p54 = tim5_ch4.period;
			float32_t p31 = tim3_ch1.period;
			float32_t p32 = tim3_ch2.period;
			float32_t p33 = tim3_ch3.period;
			float32_t p34 = tim3_ch4.period;

/*
			USART_SendText("PERIOD51: ");
			USART_SendFloat(p51/4,1);
			USART_SendText("\n");

			USART_SendText("PERIOD54: ");
			USART_SendFloat(p54/4,1);
			USART_SendText("\n");

			USART_SendText("PERIOD31: ");
			USART_SendFloat(p31/4,1);
			USART_SendText("\n");

			USART_SendText("PERIOD32: ");
			USART_SendFloat(p32/4,1);
			USART_SendText("\n");

			USART_SendText("PERIOD32: ");
			USART_SendFloat(p32/4,1);
			USART_SendText("\n");
			
			USART_SendText("PERIOD33: ");
			USART_SendFloat(p33/4,1);
			USART_SendText("\n");

			USART_SendText("PERIOD34: ");
			USART_SendFloat(p34/4,1);
			USART_SendText("\n");
*/

			for(a=0; a < 7000; ++a)
			{
				__NOP;
			}

			/*for(a=0; a < 50000000; ++a)
			{
				__NOP;
			}
			*/

			TIM2->CCR1 = p34/4.01;
			TIM2->CCR2 = p34/4.01;
			TIM2->CCR3 = p34/4.01;
			TIM2->CCR4 = p34/4.01;
			/*
			USART_SendText("PERIOD34: ");
			USART_SendFloat(p34/4,1);
			USART_SendText("\n");
			*/	

			/*
			for(a=0; a < 50000000; ++a)
			{
				__NOP;
			}
			TIM2->CCR1 = 1100;
			TIM2->CCR3 = 1100;
			TIM2->CCR2 = 1100;
			TIM2->CCR4 = 1100;

			for(a=0; a < 50000000; ++a)
			{
				__NOP;
			}
			TIM2->CCR1 = 1200;
			TIM2->CCR2 = 1200;
			TIM2->CCR3 = 1200;
			TIM2->CCR4 = 1200;
			*/

	/*
		GPIO_SetBits(GPIO_PORT, PINS );
		for(a=0; a < 5000000; ++a)
		{
			__NOP;
		}
		GPIO_ResetBits(GPIO_PORT, PINS );
		
		for(a=0; a < 100000; ++a)
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
						I2C3->CR1 |= I2C_CR1_STOP;/*0b1<<9;*/
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
			log_msg("I2C3: ERROR_STATE");
			I2C_SoftwareResetCmd(I2C3, ENABLE);

			break;
	}

	__enable_irq();
}


void I2C3_ER_IRQHandler()
{
	log_msg("I2C3_ER_Handler:");

	if(I2C_GetFlagStatus(I2C3, I2C_FLAG_BERR))
	{
		log_msg("BERR");
		I2C_ClearFlag(I2C3, I2C_FLAG_BERR);
		I2C3->CR1 |= I2C_CR1_STOP;
		_i2c3_io_struct.state = I2C_IO_STATE_ERROR;

	}

	if(I2C_GetFlagStatus(I2C3, I2C_FLAG_ARLO))
	{
		log_msg("ARLO");
		I2C_ClearFlag(I2C3, I2C_FLAG_ARLO);
		I2C3->CR1 |= I2C_CR1_STOP;
		_i2c3_io_struct.state = I2C_IO_STATE_ERROR;
	}
	
	if(I2C_GetFlagStatus(I2C3, I2C_FLAG_AF))
	{
		log_msg("AF");
		I2C_ClearFlag(I2C3, I2C_FLAG_AF);
		I2C3->CR1 |= I2C_CR1_STOP;
		_i2c3_io_struct.state = I2C_IO_STATE_ERROR;
		
	}
	
	if(I2C_GetFlagStatus(I2C3, I2C_FLAG_OVR))
	{
		log_msg("OVR");
		I2C_ClearFlag(I2C3, I2C_FLAG_OVR);
		I2C3->CR1 |= I2C_CR1_STOP;
		_i2c3_io_struct.state = I2C_IO_STATE_ERROR;		
	}
	
	if(I2C_GetFlagStatus(I2C3, I2C_FLAG_PECERR))
	{
		log_msg("PCEERR");
		I2C_ClearFlag(I2C3, I2C_FLAG_PECERR);
		I2C3->CR1 |= I2C_CR1_STOP;
		_i2c3_io_struct.state = I2C_IO_STATE_ERROR;	
	}
	
	if(I2C_GetFlagStatus(I2C3, I2C_FLAG_TIMEOUT))
	{
		log_msg("TIMEOUT");
		I2C_ClearFlag(I2C3, I2C_FLAG_TIMEOUT);
		I2C3->CR1 |= I2C_CR1_STOP;
		_i2c3_io_struct.state = I2C_IO_STATE_ERROR;	
	}

	I2C_DeInit(I2C3);
	
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

	log_msg("END");
}

void TIM2_IRQHandler()
{
	__disable_irq();
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		GPIO_SetBits(GPIO_PORT, PINS);
	}

	if(TIM_GetITStatus(TIM2, TIM_IT_CC1) != RESET)
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);
		GPIO_ResetBits(GPIO_PORT, GPIO_Pin_1);
	}

	if(TIM_GetITStatus(TIM2, TIM_IT_CC2) != RESET)
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);
		GPIO_ResetBits(GPIO_PORT, GPIO_Pin_3);
	}

	if(TIM_GetITStatus(TIM2, TIM_IT_CC3) != RESET)
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC3);
		GPIO_ResetBits(GPIO_PORT, GPIO_Pin_5);
	}

	if(TIM_GetITStatus(TIM2, TIM_IT_CC4) != RESET)
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC4);
		GPIO_ResetBits(GPIO_PORT, GPIO_Pin_7);
	}
	__enable_irq();
}

/*
void NMI_Handler()
{
		USART_SendText("nmi\n");
}

void HardFault_Handler()
{
		USART_SendText("hf\n");
}

void MemManage_Handler()
{
		USART_SendText("mem-man\n");
}

void BusFault_Handler()
{
		USART_SendText("bf\n");
}

void UsageFault_Handler()
{
			USART_SendText("uf\n");

}

*/
