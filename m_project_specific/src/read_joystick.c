#define READ_JOYSTICK_INTERNAL

#include "m_project_specific/inc/read_joystick.h"


tim_ic_ch_state_struct  tim5_ch1 = 
{
	.current_state=TIM_IC_CH_STATE_FIRST,
	.falling = 0,
	.rising = 0,
	.period = 0
};

tim_ic_ch_state_struct  tim5_ch4 = 
{
	.current_state=TIM_IC_CH_STATE_FIRST, 
};


static void TI1_Config(TIM_TypeDef* TIMx, uint16_t TIM_ICPolarity, uint16_t TIM_ICSelection,
                       uint16_t TIM_ICFilter);
static void TI2_Config(TIM_TypeDef* TIMx, uint16_t TIM_ICPolarity, uint16_t TIM_ICSelection,
                       uint16_t TIM_ICFilter);
static void TI3_Config(TIM_TypeDef* TIMx, uint16_t TIM_ICPolarity, uint16_t TIM_ICSelection,
                       uint16_t TIM_ICFilter);
static void TI4_Config(TIM_TypeDef* TIMx, uint16_t TIM_ICPolarity, uint16_t TIM_ICSelection,
                       uint16_t TIM_ICFilter);

/**
  * @brief  Configure the TI1 as Input.
  * @param  TIMx: where x can be 1, 2, 3, 4, 5, 8, 9, 10, 11, 12, 13 or 14 
  *         to select the TIM peripheral.
  * @param  TIM_ICPolarity : The Input Polarity.
  *          This parameter can be one of the following values:
  *            @arg TIM_ICPolarity_Rising
  *            @arg TIM_ICPolarity_Falling
  *            @arg TIM_ICPolarity_BothEdge  
  * @param  TIM_ICSelection: specifies the input to be used.
  *          This parameter can be one of the following values:
  *            @arg TIM_ICSelection_DirectTI: TIM Input 1 is selected to be connected to IC1.
  *            @arg TIM_ICSelection_IndirectTI: TIM Input 1 is selected to be connected to IC2.
  *            @arg TIM_ICSelection_TRC: TIM Input 1 is selected to be connected to TRC.
  * @param  TIM_ICFilter: Specifies the Input Capture Filter.
  *          This parameter must be a value between 0x00 and 0x0F.
  * @retval None
  */
static void TI1_Config(TIM_TypeDef* TIMx, uint16_t TIM_ICPolarity, uint16_t TIM_ICSelection,
                       uint16_t TIM_ICFilter)
{
  uint16_t tmpccmr1 = 0, tmpccer = 0;

  /* Disable the Channel 1: Reset the CC1E Bit */
  TIMx->CCER &= (uint16_t)~TIM_CCER_CC1E;
  tmpccmr1 = TIMx->CCMR1;
  tmpccer = TIMx->CCER;

  /* Select the Input and set the filter */
  tmpccmr1 &= ((uint16_t)~TIM_CCMR1_CC1S) & ((uint16_t)~TIM_CCMR1_IC1F);
  tmpccmr1 |= (uint16_t)(TIM_ICSelection | (uint16_t)(TIM_ICFilter << (uint16_t)4));

  /* Select the Polarity and set the CC1E Bit */
  tmpccer &= (uint16_t)~(TIM_CCER_CC1P | TIM_CCER_CC1NP);
  tmpccer |= (uint16_t)(TIM_ICPolarity | (uint16_t)TIM_CCER_CC1E);

  /* Write to TIMx CCMR1 and CCER registers */
  TIMx->CCMR1 = tmpccmr1;
  TIMx->CCER = tmpccer;
}

/**
  * @brief  Configure the TI2 as Input.
  * @param  TIMx: where x can be 1, 2, 3, 4, 5, 8, 9 or 12 to select the TIM 
  *         peripheral.
  * @param  TIM_ICPolarity : The Input Polarity.
  *          This parameter can be one of the following values:
  *            @arg TIM_ICPolarity_Rising
  *            @arg TIM_ICPolarity_Falling
  *            @arg TIM_ICPolarity_BothEdge   
  * @param  TIM_ICSelection: specifies the input to be used.
  *          This parameter can be one of the following values:
  *            @arg TIM_ICSelection_DirectTI: TIM Input 2 is selected to be connected to IC2.
  *            @arg TIM_ICSelection_IndirectTI: TIM Input 2 is selected to be connected to IC1.
  *            @arg TIM_ICSelection_TRC: TIM Input 2 is selected to be connected to TRC.
  * @param  TIM_ICFilter: Specifies the Input Capture Filter.
  *          This parameter must be a value between 0x00 and 0x0F.
  * @retval None
  */
static void TI2_Config(TIM_TypeDef* TIMx, uint16_t TIM_ICPolarity, uint16_t TIM_ICSelection,
                       uint16_t TIM_ICFilter)
{
  uint16_t tmpccmr1 = 0, tmpccer = 0, tmp = 0;

  /* Disable the Channel 2: Reset the CC2E Bit */
  TIMx->CCER &= (uint16_t)~TIM_CCER_CC2E;
  tmpccmr1 = TIMx->CCMR1;
  tmpccer = TIMx->CCER;
  tmp = (uint16_t)(TIM_ICPolarity << 4);

  /* Select the Input and set the filter */
  tmpccmr1 &= ((uint16_t)~TIM_CCMR1_CC2S) & ((uint16_t)~TIM_CCMR1_IC2F);
  tmpccmr1 |= (uint16_t)(TIM_ICFilter << 12);
  tmpccmr1 |= (uint16_t)(TIM_ICSelection << 8);

  /* Select the Polarity and set the CC2E Bit */
  tmpccer &= (uint16_t)~(TIM_CCER_CC2P | TIM_CCER_CC2NP);
  tmpccer |=  (uint16_t)(tmp | (uint16_t)TIM_CCER_CC2E);

  /* Write to TIMx CCMR1 and CCER registers */
  TIMx->CCMR1 = tmpccmr1 ;
  TIMx->CCER = tmpccer;
}

/**
  * @brief  Configure the TI3 as Input.
  * @param  TIMx: where x can be 1, 2, 3, 4, 5 or 8 to select the TIM peripheral.
  * @param  TIM_ICPolarity : The Input Polarity.
  *          This parameter can be one of the following values:
  *            @arg TIM_ICPolarity_Rising
  *            @arg TIM_ICPolarity_Falling
  *            @arg TIM_ICPolarity_BothEdge         
  * @param  TIM_ICSelection: specifies the input to be used.
  *          This parameter can be one of the following values:
  *            @arg TIM_ICSelection_DirectTI: TIM Input 3 is selected to be connected to IC3.
  *            @arg TIM_ICSelection_IndirectTI: TIM Input 3 is selected to be connected to IC4.
  *            @arg TIM_ICSelection_TRC: TIM Input 3 is selected to be connected to TRC.
  * @param  TIM_ICFilter: Specifies the Input Capture Filter.
  *          This parameter must be a value between 0x00 and 0x0F.
  * @retval None
  */
static void TI3_Config(TIM_TypeDef* TIMx, uint16_t TIM_ICPolarity, uint16_t TIM_ICSelection,
                       uint16_t TIM_ICFilter)
{
  uint16_t tmpccmr2 = 0, tmpccer = 0, tmp = 0;

  /* Disable the Channel 3: Reset the CC3E Bit */
  TIMx->CCER &= (uint16_t)~TIM_CCER_CC3E;
  tmpccmr2 = TIMx->CCMR2;
  tmpccer = TIMx->CCER;
  tmp = (uint16_t)(TIM_ICPolarity << 8);

  /* Select the Input and set the filter */
  tmpccmr2 &= ((uint16_t)~TIM_CCMR1_CC1S) & ((uint16_t)~TIM_CCMR2_IC3F);
  tmpccmr2 |= (uint16_t)(TIM_ICSelection | (uint16_t)(TIM_ICFilter << (uint16_t)4));

  /* Select the Polarity and set the CC3E Bit */
  tmpccer &= (uint16_t)~(TIM_CCER_CC3P | TIM_CCER_CC3NP);
  tmpccer |= (uint16_t)(tmp | (uint16_t)TIM_CCER_CC3E);

  /* Write to TIMx CCMR2 and CCER registers */
  TIMx->CCMR2 = tmpccmr2;
  TIMx->CCER = tmpccer;
}

/**
  * @brief  Configure the TI4 as Input.
  * @param  TIMx: where x can be 1, 2, 3, 4, 5 or 8 to select the TIM peripheral.
  * @param  TIM_ICPolarity : The Input Polarity.
  *          This parameter can be one of the following values:
  *            @arg TIM_ICPolarity_Rising
  *            @arg TIM_ICPolarity_Falling
  *            @arg TIM_ICPolarity_BothEdge     
  * @param  TIM_ICSelection: specifies the input to be used.
  *          This parameter can be one of the following values:
  *            @arg TIM_ICSelection_DirectTI: TIM Input 4 is selected to be connected to IC4.
  *            @arg TIM_ICSelection_IndirectTI: TIM Input 4 is selected to be connected to IC3.
  *            @arg TIM_ICSelection_TRC: TIM Input 4 is selected to be connected to TRC.
  * @param  TIM_ICFilter: Specifies the Input Capture Filter.
  *          This parameter must be a value between 0x00 and 0x0F.
  * @retval None
  */
static void TI4_Config(TIM_TypeDef* TIMx, uint16_t TIM_ICPolarity, uint16_t TIM_ICSelection,
                       uint16_t TIM_ICFilter)
{
  uint16_t tmpccmr2 = 0, tmpccer = 0, tmp = 0;

  /* Disable the Channel 4: Reset the CC4E Bit */
  TIMx->CCER &= (uint16_t)~TIM_CCER_CC4E;
  tmpccmr2 = TIMx->CCMR2;
  tmpccer = TIMx->CCER;
  tmp = (uint16_t)(TIM_ICPolarity << 12);

  /* Select the Input and set the filter */
  tmpccmr2 &= ((uint16_t)~TIM_CCMR1_CC2S) & ((uint16_t)~TIM_CCMR1_IC2F);
  tmpccmr2 |= (uint16_t)(TIM_ICSelection << 8);
  tmpccmr2 |= (uint16_t)(TIM_ICFilter << 12);

  /* Select the Polarity and set the CC4E Bit */
  tmpccer &= (uint16_t)~(TIM_CCER_CC4P | TIM_CCER_CC4NP);
  tmpccer |= (uint16_t)(tmp | (uint16_t)TIM_CCER_CC4E);

  /* Write to TIMx CCMR2 and CCER registers */
  TIMx->CCMR2 = tmpccmr2;
  TIMx->CCER = tmpccer ;
}









void TIM5_IRQHandler()
{
	__disable_irq();
	if(TIM_GetITStatus(TIM5, TIM_IT_CC1) != RESET)
	{
		switch (tim5_ch1.current_state)
		{
			case TIM_IC_CH_STATE_FIRST:
				tim5_ch1.rising = TIM_GetCapture1(TIM5);
				TI1_Config(TIM5, TIM_ICPolarity_BothEdge, TIM_ICSelection_DirectTI, 0x0);
				tim5_ch1.current_state = TIM_IC_CH_STATE_FALLING;
				break;
			
			case TIM_IC_CH_STATE_FALLING:
				tim5_ch1.falling = TIM_GetCapture1(TIM5);
				tim5_ch1.period = tim5_ch1.falling - tim5_ch1.rising;
				if(tim5_ch1.period < 0)
				{
					tim5_ch1.period = tim5_ch1.period + 0xFFFF;
				}
				tim5_ch1.current_state = TIM_IC_CH_STATE_RISING;
				break;
			
			case TIM_IC_CH_STATE_RISING:
				tim5_ch1.rising = TIM_GetCapture1(TIM5);
				tim5_ch1.current_state = TIM_IC_CH_STATE_FALLING;

		}
		TIM_ClearITPendingBit(TIM5,TIM_IT_CC1);
	}

		if(TIM_GetITStatus(TIM5, TIM_IT_CC4) != RESET)
	{
		switch (tim5_ch4.current_state)
		{
			case TIM_IC_CH_STATE_FIRST:
				tim5_ch4.rising = TIM_GetCapture4(TIM5);
				TI4_Config(TIM5, TIM_ICPolarity_BothEdge, TIM_ICSelection_DirectTI, 0x0);
				tim5_ch4.current_state = TIM_IC_CH_STATE_FALLING;
				break;
			
			case TIM_IC_CH_STATE_FALLING:
				tim5_ch4.falling = TIM_GetCapture4(TIM5);
				tim5_ch4.period = tim5_ch4.falling - tim5_ch4.rising;
				if(tim5_ch4.period < 0)
				{
					tim5_ch4.period = tim5_ch4.period + 0xFFFF;
				}
				tim5_ch4.current_state = TIM_IC_CH_STATE_RISING;
				break;
			
			case TIM_IC_CH_STATE_RISING:
				tim5_ch4.rising = TIM_GetCapture4(TIM5);
				tim5_ch4.current_state = TIM_IC_CH_STATE_FALLING;
		}

		TIM_ClearITPendingBit(TIM5,TIM_IT_CC4);
	}


	__enable_irq();
}
