/*
This module contains matrix definitions and calculations
*/

#ifndef M_PROJECT_SPECIFIC_READ_JOYSTICK_H
#define M_PROJECT_SPECIFIC_READ_JOYSTICK_H
/**
 * INCLUDES
 **/
	#include "m_common/inc/assert.h"
	#include "m_common/inc/result.h"
	#include "m_math/inc/math.h"
	#include "system/stm32f4xx.h"

/**
 * MACROS
 **/


/**
 * ENUMS
 **/

typedef enum TIM_IC_CH_STATE
{
	TIM_IC_CH_STATE_FIRST = 0,
	TIM_IC_CH_STATE_RISING,
	TIM_IC_CH_STATE_FALLING
} tim_ic_ch_state_enum;

typedef struct TIM_IC_CH_STRUCT
{
	__IO tim_ic_ch_state_enum current_state;
	__IO int32_t rising;
	__IO int32_t falling;
	__IO int32_t period;
} tim_ic_ch_state_struct;

/**
 * TYPEDEFS
 **/


/**
 * GLOBAL VARIABLES
 **/


extern tim_ic_ch_state_struct  tim5_ch1;
extern tim_ic_ch_state_struct  tim5_ch4;



/**
 * FUNCTION DEFINITIONS
 **/



#endif /*M_PROJECT_SPECIFIC_READ_JOYSTICK_H*/
