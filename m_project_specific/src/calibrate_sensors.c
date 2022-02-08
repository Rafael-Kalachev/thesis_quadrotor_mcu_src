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
	0.0137509 /*x^0*/, 
	6.8506080e-04 /*x^1*/, 
	2.6430291e-05 /*x^2*/,
	4.6991656e-07 /*x^3*/,
	3.5212322e-09 /*x^4*/,
	9.4697491e-14 /*x^5*/,
	-1.3444270e-13 /*x^6*/,
	-4.9663850e-16 /*x^7*/
};
static const polynomial_struct_t poly_gyro_x_temp =
{
	.degree=7,
	.polynomial_coeficients_p = poly_gyro_x_temp_f32
};

float32_t poly_gyro_y_temp_f32[] = 
{ 
	 -0.0214579 /*x^0*/, 
	 -0.0012760 /*x^1*/, 
	 -4.7865615e-05 /*x^2*/,
	 -8.3421094e-07 /*x^3*/,
	 -5.9833902e-09 /*x^4*/,
	 3.2100889e-12 /*x^5*/,
	 2.5291431e-13 /*x^6*/,
	 8.9518084e-16 /*x^7*/
};
static const polynomial_struct_t poly_gyro_y_temp =
{
	.degree=7,
	.polynomial_coeficients_p = poly_gyro_y_temp_f32
};

float32_t poly_gyro_z_temp_f32[] = 
{ 
	-0.0079808 /*x^0*/, 
	-3.2266506e-04 /*x^1*/, 
	-1.3153715e-05 /*x^2*/,
	-2.4726185e-07 /*x^3*/,
	-2.0426889e-09 /*x^4*/,
	-2.3442576e-12 /*x^5*/,
	 6.1749332e-14 /*x^6*/,
	 2.5545603e-16 /*x^7*/
};

static const polynomial_struct_t poly_gyro_z_temp =
{
	.degree=7,
	.polynomial_coeficients_p = poly_gyro_z_temp_f32
};


float32_t magnetometer_elipsoid_f32[3*3] = 
{
    21.4229 ,  0.8379  ,  3.086,
    0       ,  21.6201 ,  2.0207,
    0       ,  0       ,  27.2441
};

arm_matrix_instance_f32 magnetometer_elipsoid =
{
	.numCols = 3,
	.numRows = 3,
	.pData = magnetometer_elipsoid_f32
};

float32_t magnetometer_center_f32[3*1] = 
{
	-0.0016,
    0.0199,
    0.0457 
};

arm_matrix_instance_f32 magnetometer_center =
{
	.numCols = 1,
	.numRows = 3,
	.pData = magnetometer_center_f32
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


static void elipsoid_sphere_calibration(arm_matrix_instance_f32 *elipsoid, arm_matrix_instance_f32 *center, arm_matrix_instance_f32 *in_vector, arm_matrix_instance_f32 *out_vector)
{
	float32_t tmp_buf[1*3];
	arm_matrix_instance_f32 tmp = {.numCols=1, .numRows=3, .pData=&tmp_buf};
	arm_mat_sub_f32(in_vector, center, &tmp);
	arm_mat_mult_f32(elipsoid, &tmp, out_vector);
}



void calibrate_sensors( raw_sensors_struct* raw_sensors_data, calibrated_sensors_struct *out_calibrated_sensors_p)
{
	static int i = 0;
	static float32_t gx =0.0;
	static float32_t gy =0.0;
	static float32_t gz =0.0;


	float32_t tempval;
	
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

	if (i<50)
	{
		++i;

		gz += out_calibrated_sensors_p->gyro_z;
		gy += out_calibrated_sensors_p->gyro_y;
		gx += out_calibrated_sensors_p->gyro_x;
	}
	else
	{
		out_calibrated_sensors_p->gyro_z -= gz/50.0;
		out_calibrated_sensors_p->gyro_y -= gy/50.0;
		out_calibrated_sensors_p->gyro_x -= gx/50.0;
	}

	/*calibrate acc x*/
	tempval = (float32_t)raw_sensors_data->acc_x * INV_INT16_MAX;
	out_calibrated_sensors_p->acc_x = tempval*4;

	/*calibrate acc y*/
	tempval = (float32_t)raw_sensors_data->acc_y * INV_INT16_MAX;
	out_calibrated_sensors_p->acc_y = tempval*4;

	/*calibrate acc z*/
	tempval = (float32_t)raw_sensors_data->acc_z * INV_INT16_MAX;
	out_calibrated_sensors_p->acc_z = tempval*4;

	/*calibrate MAGNETOMETER*/
	tempval = (float32_t)raw_sensors_data->mag_x * INV_INT16_MAX;
	out_calibrated_sensors_p->mag_x = tempval;

	tempval = (float32_t)raw_sensors_data->mag_y * INV_INT16_MAX;
	out_calibrated_sensors_p->mag_y = tempval;

	tempval = (float32_t)raw_sensors_data->mag_z * INV_INT16_MAX;
	out_calibrated_sensors_p->mag_z = tempval;
	float32_t tmp_buff[3*1] = {out_calibrated_sensors_p->mag_x, out_calibrated_sensors_p->mag_y, out_calibrated_sensors_p->mag_z};
	arm_matrix_instance_f32 tmp = {.numCols=1, .numRows=3, .pData=&tmp_buff};
	elipsoid_sphere_calibration(&magnetometer_elipsoid, &magnetometer_center, &tmp_buff, &tmp_buff);
	out_calibrated_sensors_p->mag_x = tmp_buff[0];
	out_calibrated_sensors_p->mag_y = tmp_buff[1];
	out_calibrated_sensors_p->mag_z = tmp_buff[2];
}