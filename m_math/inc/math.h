/*
This module contains matrix definitions and calculations
*/

#ifndef M_MATH_H
#define M_MATH_H
/**
 * INCLUDES
 **/

#include "cmsis/inc/arm_math.h"

/**
 * CONSTANT DEFINES
 **/

//	#define PI	( 3.14159265359 )
	#define RAD2DEG_CONV	( 180.0 / PI )
	#define DEG2RAD_CONV	( PI / 180.0 ) 
	#define INV_INT16_MAX   (float32_t)( (float32_t) 1 / (float32_t) INT16_MAX )


/**
 * MACROS
 **/

	#define ADD(a,b)	( (a) + (b) ) 
	#define	SUB(a,b)	( (a) - (b) )
	#define MUL(a,b)	( (a) * (b) )
	#define DIV(a,b)	( (a) / (b) )
	
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


#endif /*M_MATH_H*/
