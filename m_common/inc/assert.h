/*
This module contains matrix definitions and calculations
*/

#ifndef M_MAT_H
#define M_MAT_H
/**
 * INCLUDES
 **/
	#include "system/conf.h"
/**
 * MACROS
 **/
	/*TODO	improve*/

	#if	defined(CONFIGURATION_RELEASE) | defined(NO_ASSERT)
		#define ASSERT_EQ(EXPR,VAL,MESSAGE)	(void 0)
	#else
		/*TODO*/
		#error	"For now there is no implementation of assert"
	#endif	
	
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


#endif /*M_MAT_H*/
