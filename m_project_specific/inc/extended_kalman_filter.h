/*
This module contains matrix definitions and calculations
*/

#ifndef M_PROJECT_SPECIFIC_EXTENDED_KALMAN_FILTER_H
#define M_PROJECT_SPECIFIC_EXTENDED_KALMAN_FILTER_H
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

void predict();
void ekf_init();

#endif /*M_PROJECT_SPECIFIC_EXTENDED_KALMAN_FILTER_H*/
