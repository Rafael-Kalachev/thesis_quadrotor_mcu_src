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
	#include "m_math/inc/quaternion.h"
	#include "m_project_specific/inc/calibrate_sensors.h"


/**
 * MACROS
 **/


/**
 * ENUMS
 **/


/**
 * TYPEDEFS
 **/

typedef struct ORIENTATION_STRUCT_TAG
{
	float32_t pitch;
	float32_t roll;
	float32_t yaw;

} orientation_struct_t;

/**
 * GLOBAL VARIABLES
 **/


/**
 * FUNCTION DEFINITIONS
 **/

void ekf_init();
void ekf_next(calibrated_sensors_struct* calibrated_sensors, quaternion_struct_t * out_quaternion);

void kf_init();
void kf_next(calibrated_sensors_struct* calibrated_sensors, orientation_struct_t * out_orientation);

void quaternion_to_orientation(quaternion_struct_t * in_quaternion, orientation_struct_t *out_orientation);




#endif /*M_PROJECT_SPECIFIC_EXTENDED_KALMAN_FILTER_H*/
