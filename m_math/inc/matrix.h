/*
This module contains matrix definitions and calculations
*/

#ifndef M_MATRIX_H
#define M_MATRIX_H
/**
 * INCLUDES
 **/
	#include "system/conf.h"
	#include "m_common/inc/assert.h"
	#include "m_common/inc/result.h"
	#include "m_math/inc/math.h"

/**
 * MACROS
 **/
	#define	MATRIX_MAX_SIZE	20
	#define MATRIX_MAX_ROW_SIZE	MATRIX_MAX_SIZE
	#define	MATRIX_MAX_COL_SIZE	MATRIX_MAX_SIZE
	
	#define MATRIX_SIZE_ROW_ID	0
	#define MATRIX_SIZE_COL_ID	1


/**
 * ENUMS
 **/


/**
 * TYPEDEFS
 **/

typedef	char	matrix_index_t;
typedef	float	matrix_element_t;

	typedef struct MATRIX2D
	{
		matrix_element_t*	data;
		matrix_index_t		size[2];
	
	} matrix2d_t

/**
 * GLOBAL VARIABLES
 **/



/**
 * FUNCTION DEFINITIONS
 **/

	result_t matrix2d_add( matrix2d_t* a, matrix2d_t* b, matrix2d_t* res);
	result_t matrix2d_sub( matrix2d_t* a, matrix2d_t* b, matrix2d_t* res);
	result_t matrix2d_mul( matrix2d_t* a, matrix2d_t* b, matrix2d_t* res);
	result_t matrix2d_neg( matrix2d_t* a, matrix2d_t* res);

#endif /*M_MATRIX_H*/
