#include "stm32f4xx.h"
#include "m_project_specific/inc/calibrate_sensors.h"
#include "m_math/inc/convert.h"
#include "m_math/inc/polynomial.h"
#include "m_project_specific/inc/usart.h"



/**
 * MODULE STATIC VARIABLES 
 **/

static calibrated_sensors_struct calibrated_sensors_buffer_1;
static calibrated_sensors_struct calibrated_sensors_buffer_2;

float32_t poly_gyro_x_temp_f32[] = 
{ 
	-9.8393520e-04 /*x^0*/, 
	-8.4714632e-04 /*x^1*/, 
	-3.4361692e-05 /*x^2*/,
	-6.9101105e-07 /*x^3*/,
	-7.3368422e-09 /*x^4*/,
	-3.9557652e-11 /*x^5*/,
	-9.0997204e-14 /*x^6*/,
	-3.8057584e-17 /*x^7*/
};
static const polynomial_struct_t poly_gyro_x_temp =
{
	.degree=7,
	.polynomial_coeficients_p = poly_gyro_x_temp_f32
};

float32_t poly_gyro_y_temp_f32[] = 
{ 
	 0.0056154 /*x^0*/, 
	 0.0015251 /*x^1*/, 
	 6.2525978e-05 /*x^2*/,
	 1.2539944e-06 /*x^3*/,
	 1.3248271e-08 /*x^4*/,
	 7.0612030e-11 /*x^5*/,
	 1.5667841e-13 /*x^6*/,
	 4.7236212e-17 /*x^7*/
};
static const polynomial_struct_t poly_gyro_y_temp =
{
	.degree=7,
	.polynomial_coeficients_p = poly_gyro_y_temp_f32
};

float32_t poly_gyro_z_temp_f32[] = 
{ 
	-5.6591426e-04 /*x^0*/, 
	 4.5805777e-04 /*x^1*/, 
	 1.8339437e-05 /*x^2*/,
	 3.6834075e-07 /*x^3*/,
	 3.9355625e-09 /*x^4*/,
	 2.1660148e-11 /*x^5*/,
	 5.3144087e-14 /*x^6*/,
	 3.3000864e-17 /*x^7*/
};

static const polynomial_struct_t poly_gyro_z_temp =
{
	.degree=7,
	.polynomial_coeficients_p = poly_gyro_z_temp_f32
};


/**
 * STATIC FUNCTIONS
 **/


static calibrated_sensors_struct* s_select_buffer()
{
	static calibrated_sensors_struct *current_calibrated_sensors_buffer_p = &calibrated_sensors_buffer_1;

	if (current_calibrated_sensors_buffer_p == &calibrated_sensors_buffer_1)
	{
		current_calibrated_sensors_buffer_p = &calibrated_sensors_buffer_2;
	}
	else
	{
		current_calibrated_sensors_buffer_p = &calibrated_sensors_buffer_1;
	}

	return current_calibrated_sensors_buffer_p;
}


static float32_t polinomial_compensation(polynomial_struct_t* polinomial, float32_t point_of_compensation ,float32_t value_to_be_compensated)
{
	float32_t compensation = polinomial_evaluate(polinomial, point_of_compensation);
	return value_to_be_compensated - compensation;
}


calibrated_sensors_struct* calibrate_sensors( raw_sensors_struct* raw_sensors_data )
{
	float32_t tempval;
	calibrated_sensors_struct *out_calibrated_sensors_p = s_select_buffer();
	
	/*calibrate gyro x*/
	tempval = (float32_t)raw_sensors_data->gyro_x * INV_INT16_MAX;
	tempval = polinomial_compensation(&poly_gyro_x_temp, raw_sensors_data->temperature, tempval);
	out_calibrated_sensors_p->gyro_x = tempval /* * 500*/ /*dps*/; /*others use 64 for 500 and 32 for 1000 ? check*/

	/*calibrate gyro y*/
	tempval = (float32_t)raw_sensors_data->gyro_y * INV_INT16_MAX;
	tempval = polinomial_compensation(&poly_gyro_y_temp, raw_sensors_data->temperature, tempval);
	out_calibrated_sensors_p->gyro_y = tempval /* * 500*/ /*dps*/;

	/*calibrate gyro z*/
	tempval = (float32_t)raw_sensors_data->gyro_z * INV_INT16_MAX;
	tempval = polinomial_compensation(&poly_gyro_z_temp, raw_sensors_data->temperature, tempval);
	out_calibrated_sensors_p->gyro_z = tempval /* * 500*/ /*dps*/;

	/*calibrate acc x*/
	tempval = (float32_t)raw_sensors_data->acc_x * INV_INT16_MAX;
	out_calibrated_sensors_p->acc_x = tempval;

	/*calibrate acc y*/
	tempval = (float32_t)raw_sensors_data->acc_y * INV_INT16_MAX;
	out_calibrated_sensors_p->acc_y = tempval;

	/*calibrate acc z*/
	tempval = (float32_t)raw_sensors_data->acc_z * INV_INT16_MAX;
	out_calibrated_sensors_p->acc_z = tempval;

	/*calibrate mag x*/
	tempval = (float32_t)raw_sensors_data->mag_x * INV_INT16_MAX;
	out_calibrated_sensors_p->mag_x = tempval;

	/*calibrate mag y*/
	tempval = (float32_t)raw_sensors_data->mag_y * INV_INT16_MAX;
	out_calibrated_sensors_p->mag_y = tempval;

	/*calibrate mag z*/
	tempval = (float32_t)raw_sensors_data->mag_z * INV_INT16_MAX;
	out_calibrated_sensors_p->mag_z = tempval;




	return out_calibrated_sensors_p;
}