#include "stm32f4xx.h"
#include "m_project_specific/inc/usart.h"
#include "m_math/inc/convert.h"


typedef enum USART_RCV_CLI_STATE_ENUM
{
	USART_RCV_CLI_STATE_START,  /*Start of command line*/
	USART_RCV_CLI_STATE_ERR, /*Mistake was made, redirectind to error message after the new line*/
	USART_RCV_CLI_STATE_CMD_SET,
	USART_RCV_CLI_STATE_CMD_SET_RPM,
	USART_RCV_CLI_STATE_CMD_SET_RPM_A,
	USART_RCV_CLI_STATE_CMD_SET_RPM_B,
	USART_RCV_CLI_STATE_CMD_SET_RPM_C,
	USART_RCV_CLI_STATE_CMD_SET_RPM_D,
	USART_RCV_CLI_STATE_END
}usart_rcv_cli_state_enum_t;

typedef struct USART_RCV_CLI_STRUCT
{
	usart_rcv_cli_state_enum_t state;
	char 	buffer[10];
	uint8_t count;
}usart_rcv_cli_struct_t;

usart_rcv_cli_struct_t usart_1_cmd_handler = 
{
	.state = USART_RCV_CLI_STATE_START,
	.buffer = {0,0,0,0,0,0,0,0,0,0},
	.count = 0
};



// This functions send text or char array passed as argument over USART
void USART_SendText(volatile char *s)
{
	while(*s)
	{
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
	int count = int_to_str(x, value, 1);
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

// This function sends numbers up to 32bit over USART
void USART_SendMatrix(arm_matrix_instance_f32* matrix, uint8_t precision)
{
	int row_i = 0;
	int col_i = 0;
	float32_t *val_ptr=matrix->pData;
	char value[20]; //a temp array to hold results of conversion

	for(row_i = 0; row_i < matrix->numRows; ++row_i)
	{

		for(col_i = 0; col_i < matrix->numCols; ++col_i)
		{
			USART_SendFloat(*val_ptr, precision);
			USART_SendText("\t");
			++val_ptr;
    	}
		USART_SendText("\n");
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
	USART_InitStruct.USART_BaudRate = 9600;
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;
	USART_InitStruct.USART_StopBits = USART_StopBits_1;
	USART_InitStruct.USART_Parity = USART_Parity_No ;
	USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART1, &USART_InitStruct);


	USART_Cmd(USART1, ENABLE); // Start USART1 peripheral

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);


	NVIC_InitTypeDef nvicStructure;
	nvicStructure.NVIC_IRQChannel = USART1_IRQn;
	nvicStructure.NVIC_IRQChannelPreemptionPriority = 4;
	nvicStructure.NVIC_IRQChannelSubPriority = 0;
	nvicStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvicStructure);

}


/* This is the function that gets executed on USART1 interrupt */
void USART1_IRQHandler(void)
{
	//USART_SendText("AAA");
	if(USART_GetITStatus(USART1, USART_IT_RXNE)) // was is triggered by RXNE trigger
	{
		uint16_t data = USART_ReceiveData(USART1);

//		USART_SendText("DATA:");
//		USART_SendNumber(data);
//		USART_SendText("\n");
	
		switch(usart_1_cmd_handler.state)
		{
			case USART_RCV_CLI_STATE_START:
				usart_1_cmd_handler.count = 0;
				if(data  == 's' || data == 'S')
				{
					usart_1_cmd_handler.state = USART_RCV_CLI_STATE_CMD_SET;
//					USART_SendText("Set ");
				}
				else if(data != '\n')
				{
					usart_1_cmd_handler.state = USART_RCV_CLI_STATE_ERR;
				}
			break;

			case USART_RCV_CLI_STATE_ERR:
				if(data  == '\n')
				{
					usart_1_cmd_handler.state = USART_RCV_CLI_STATE_START;
					USART_SendText("Unrecognized command\n");
				}
			break;

			case USART_RCV_CLI_STATE_CMD_SET:
				if(data  == 'r' || data == 'R')
				{
					usart_1_cmd_handler.state = USART_RCV_CLI_STATE_CMD_SET_RPM;
//					USART_SendText("RPM ");
				}
				else
				{
					usart_1_cmd_handler.state = USART_RCV_CLI_STATE_ERR;
				}
			break;
	
			case USART_RCV_CLI_STATE_CMD_SET_RPM:
				if(data  == 'a' || data == 'A')
				{
					usart_1_cmd_handler.state = USART_RCV_CLI_STATE_CMD_SET_RPM_A;
					USART_SendText("A ");
				}
				else if(data == 'b' || data == 'B' )
				{
					usart_1_cmd_handler.state = USART_RCV_CLI_STATE_CMD_SET_RPM_B;
					USART_SendText("B ");
				}
				else if(data == 'c' || data == 'C' )
				{
					usart_1_cmd_handler.state = USART_RCV_CLI_STATE_CMD_SET_RPM_C;
					USART_SendText("C ");
				}
				else if(data == 'd' || data == 'D' )
				{
					usart_1_cmd_handler.state = USART_RCV_CLI_STATE_CMD_SET_RPM_D;
					USART_SendText("D ");
				}
				else
				{
					usart_1_cmd_handler.state = USART_RCV_CLI_STATE_ERR;
				}

			break;
	
			case USART_RCV_CLI_STATE_CMD_SET_RPM_A:

			
			case USART_RCV_CLI_STATE_CMD_SET_RPM_B:
	

			
			case USART_RCV_CLI_STATE_CMD_SET_RPM_C:


			case USART_RCV_CLI_STATE_CMD_SET_RPM_D:

				if(data >= '0' && data <= '9')
				{
					if(usart_1_cmd_handler.count < 4 )
					{

						usart_1_cmd_handler.buffer[usart_1_cmd_handler.count] = data;
						++usart_1_cmd_handler.count;
					}
					else
					{
						usart_1_cmd_handler.state = USART_RCV_CLI_STATE_ERR;

					}
					
				}
				else if(data == ' ' && usart_1_cmd_handler.count > 0)
				{
					int rpm = 0;
					usart_1_cmd_handler.buffer[usart_1_cmd_handler.count] = 0;
					str_to_int(usart_1_cmd_handler.buffer, &rpm);
					USART_SendNumber(rpm);
					usart_1_cmd_handler.state = USART_RCV_CLI_STATE_END;
				}
				else
				{
					usart_1_cmd_handler.state = USART_RCV_CLI_STATE_ERR;
				}
	
			break;
			
			case USART_RCV_CLI_STATE_END:
				if(data  == '\n')
				{
					usart_1_cmd_handler.state = USART_RCV_CLI_STATE_START;
					USART_SendText("\n");
				}
			break;
		}
	}
}

