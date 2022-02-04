/*
This module contains matrix definitions and calculations
*/

#ifndef M_MATRIX_H
#define M_MATRIX_H
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


	typedef struct POLYNOMIAL_STRUCT_TAG
	{
		float32_t* polynomial_coeficients_p; /*Pointer to alocated array of float32_t with size (self.degree+1) containing the coeficients where the first coeficient is the x^0 second is x_1 ...*/
		uint8_t degree; /*uint8_t degree of the polinomial*/
	} polynomial_struct_t;

/**
 * GLOBAL VARIABLES
 **/



/**
 * FUNCTION DEFINITIONS
 **/

	/**
	 * @description: Evaluate polinomial at a specific point
	 * @param polinomial: The polinomial that will be evaluated
	 * @param x: The point at which we avaluate the polinomial
	*/
	float32_t polinomial_evaluate(polynomial_struct_t *polinomial, float32_t x);

#endif /*M_MATRIX_H*/
