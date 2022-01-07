/*
This module contains matrix definitions and calculations
*/

#ifndef M_MATH_CONVERT_H
#define M_MATH_CONVERT_H
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

/**
 * @description: Convert float to string
 * @param number: a float32_t number to be converted to string
 * @param out_str: pointer to an allocated array of type char with length >= needed for the conversion where the conversion will be saved
 * @param afterpoint: digits cout after the decimal point to be converted to string
*/
int float_to_str(float32_t number, char* out_str, uint8_t after_point, uint8_t before_point);


/**
 * @description: Convert int to string
 * @param number: a int number to be converted to string
 * @param out_str: pointer to an allocated array of type char with length >= needed for the conversion where the conversion will be saved
 * @param digits: number of digits (if bigger than the number it will fill it with '0's)
 * @return number of chars written
*/
int int_to_str(int number, char* out_str, uint8_t digits);



#endif /*M_MATH_CONVERT_H*/
