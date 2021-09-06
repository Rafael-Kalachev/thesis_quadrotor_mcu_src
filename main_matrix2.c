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

/*A =

   7   6   5   4
   6   5   8   2
   5   9   7   1
   6   2   1   0

octave:16> inv(A)
ans =

   0.0015699   0.0251177  -0.0565149   0.1868132
   0.0502355  -0.1962323   0.1915228  -0.0219780
  -0.1098901   0.2417582  -0.0439560  -0.0769231
   0.3092622  -0.0518053  -0.1334380  -0.1978022

*/


float32_t A_f32[16] =
{
  /* Const,   numTaps,   blockSize,   numTaps*blockSize */
  1.0,     32.0,      4.0,     128.0,
  1.0,     32.0,     64.0,    2048.0,
  1.0,     16.0,      4.0,      64.0,
  1.0,     16.0,     64.0,    1024.0,
};

float32_t AT_f32[16];
/* (Transpose of A * A) Buffer */
float32_t ATMA_f32[16];
/* Inverse(Transpose of A * A)  Buffer */
float32_t ATMAI_f32[16];

float32_t B_f32[2*2];

float32_t C_f32[16];

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
  USART_InitStruct.USART_BaudRate = 9600;
  USART_InitStruct.USART_WordLength = USART_WordLength_8b;
  USART_InitStruct.USART_StopBits = USART_StopBits_1;
  USART_InitStruct.USART_Parity = USART_Parity_No ;
  USART_InitStruct.USART_Mode = USART_Mode_Tx;
  USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_Init(USART1, &USART_InitStruct);
  USART_Cmd(USART1, ENABLE); // Start USART1 peripheral

	arm_matrix_instance_f32 A;
	arm_matrix_instance_f32 B;
	arm_matrix_instance_f32 C;
  arm_matrix_instance_f32 AT;     /* Matrix AT(A transpose) instance */
  arm_matrix_instance_f32 ATMA;   /* Matrix ATMA( AT multiply with A) instance */
  arm_matrix_instance_f32 ATMAI;  /* Matrix ATMAI(Inverse of ATMA) instance */
    uint32_t srcRows, srcColumns;  /* Temporary variables */
	int row,col;
	int i,j;
	arm_status status = 0;

	row = 4;
	col = 4;
	arm_mat_init_f32(&A, row,col, (float32_t *) A_f32);

	row = 2;
	col = 2;
	arm_mat_init_f32(&B, row,col, (float32_t *) B_f32);

	row = 4;
	col = 4;
	arm_mat_init_f32(&C, row,col, (float32_t *) C_f32);


  srcRows = 4;
  srcColumns = 4;
  arm_mat_init_f32(&AT, srcRows, srcColumns, AT_f32);
  /* calculation of A transpose */
  status = arm_mat_trans_f32(&A, &AT);
  /* Initialise ATMA Matrix Instance with numRows, numCols and data array(ATMA_f32) */
  srcRows = 4;
  srcColumns = 4;
  arm_mat_init_f32(&ATMA, srcRows, srcColumns, ATMA_f32);
  /* calculation of AT Multiply with A */
  status = arm_mat_mult_f32(&AT, &A, &ATMA);
  /* Initialise ATMAI Matrix Instance with numRows, numCols and data array(ATMAI_f32) */
  srcRows = 4;
  srcColumns = 4;
  status = arm_mat_inverse_f32(&ATMA, &C);

  while(1) // this function can be called slower as you add data to be sent
  {
    // Send a letter, number and text over USART
	for(i=100;--i;)
	{
    
		status = arm_mat_inverse_f32(&A, &C);
		/*status = arm_mat_scale_f32(&B, 1000.0, &C);*/
	}
	USART_SendText("status =")	;
	USART_SendNumber(status);
	USART_SendText("\n\n");
	for(i=0;i<C.numRows; ++i) 
	{
		for(j=0; j<C.numCols; ++j)
		{
			int a = (int) (C_f32[j+i*C.numCols]*1000.0);
			USART_SendNumber( a ); //These two functions already check for empty transmit buffer
			USART_SendText(",  ");
		}
		USART_SendText("\n\n");
  	}
//	    for(long i = 0; i < SystemCoreClock/13; i++){__NOP();} // wait ~1s
   }
}
