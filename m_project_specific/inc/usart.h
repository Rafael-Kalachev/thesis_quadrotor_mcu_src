/*
This module contains matrix definitions and calculations
*/

#ifndef M_PROJECT_SPECIFIC_USART_H
#define M_PROJECT_SPECIFIC_USART_H
/**
 * INCLUDES
 **/
	#include "m_common/inc/assert.h"
	#include "m_common/inc/result.h"
	#include "m_math/inc/math.h"

/**
 * MACROS
 **/


/**
 * ENUMS
 **/


/**
 * TYPEDEFS
 **/


/**
 * GLOBAL VARIABLES
 **/



/**
 * FUNCTION DEFINITIONS
 **/
// This functions send text or char array passed as argument over USART
void USART_SendText(volatile char *s);

// This function sends numbers up to 32bit over USART
void USART_SendNumber(int32_t x);

// This function sends numbers up to 32bit over USART
void USART_SendFloat(float32_t x, uint8_t precision);

void USART_SendMatrix(arm_matrix_instance_f32* matrix, uint8_t precision);

void USART_project_specific_init();


#endif /*M_PROJECT_SPECIFIC_USART_H*/
