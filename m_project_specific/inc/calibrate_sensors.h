/*
This module contains matrix definitions and calculations
*/

#ifndef M_PROJECT_SPECIFIC_CALIBRATE_SENSORS_H
#define M_PROJECT_SPECIFIC_CALIBRATE_SENSORS_H
/**
 * INCLUDES
 **/
	#include "m_common/inc/assert.h"
	#include "m_common/inc/result.h"
	#include "m_math/inc/math.h"


/**
 * DEFINES
 **/


/**
 * MACROS
 **/


/**
 * ENUMS
 **/


/**
 * TYPEDEFS
 **/

typedef struct RAW_SENSORS_STRUCT_TAG
{
    int16_t temperature; 
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    int16_t acc_x;
    int16_t acc_y;
    int16_t acc_z;
    int16_t mag_x;
    int16_t mag_y;
    int16_t mag_z;
    int16_t distance_sonar; 
} raw_sensors_struct;


typedef struct CALIBRATED_SENSORS_STRUCT_TAG
{
    float32_t temperature; /*DEG C*/
    float32_t gyro_x;
    float32_t gyro_y;
    float32_t gyro_z;
    float32_t acc_x;
    float32_t acc_y;
    float32_t acc_z;
    float32_t mag_x;
    float32_t mag_y;
    float32_t mag_z;
    float32_t distance_sonar; /*cm*/
} calibrated_sensors_struct;


/**
 * GLOBAL VARIABLES
 **/



/**
 * FUNCTION DEFINITIONS
 **/

void calibrate_sensors( raw_sensors_struct *raw_sensor_data,calibrated_sensors_struct* out_calibrated_sensors_p );



#endif /*M_PROJECT_SPECIFIC_CALIBRATE_SENSORS_H*/
