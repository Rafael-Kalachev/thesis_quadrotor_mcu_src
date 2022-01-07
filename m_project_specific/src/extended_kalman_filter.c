#include "stm32f4xx.h"
#include "m_project_specific/inc/extended_kalman_filter.h"
#include "m_project_specific/inc/usart.h"
#include "m_math/inc/convert.h"


#define CAL_D 0.1567 /*calibration dat dont forget to change*/
float32_t Q_f32[7*7] =
{
	 0 ,  0 ,  0 ,  0 ,  0 ,  0 ,  0 ,  
	 0 ,  0 ,  0 ,  0 ,  0 ,  0 ,  0 ,  
	 0 ,  0 ,  0 ,  0 ,  0 ,  0 ,  0 ,  
	 0 ,  0 ,  0 ,  0 ,  0 ,  0 ,  0 ,  
	 0 ,  0 ,  0 ,  0 ,  0.2 ,  0 ,  0 ,  
	 0 ,  0 ,  0 ,  0 ,  0 ,  0.2 ,  0 ,  
	 0 ,  0 ,  0 ,  0 ,  0 ,  0 ,  0.2   

};

float32_t x_f32[7];

float32_t F_f32[7*7];

float32_t TMP_7_7_f32[7*7];
float32_t TMP2_7_7_f32[7*7];
float32_t TMP3_7_7_f32[7*7];


float32_t P_f32[7*7];

float32_t z_f32[3];

float32_t h_f32[3];

float32_t y_f32[3];

float32_t H_f32[3*7];

float32_t S_f32[3*3];

float32_t R_f32[3*3] = 
{
	 1 ,  0 ,  0,
	 0 ,  1 ,  0,
	 0 ,  0 ,  1
};

float32_t K_f32[3*3];

arm_matrix_instance_f32 TMP3_7_7 = {.numCols=7, .numRows=7, .pData=TMP3_7_7_f32};
arm_matrix_instance_f32 TMP_7_7 = {.numCols=7, .numRows=7, .pData=TMP_7_7_f32};
arm_matrix_instance_f32 TMP2_7_7 = {.numCols=7, .numRows=7, .pData=TMP2_7_7_f32};
arm_matrix_instance_f32 Q = {.numCols=7, .numRows=7, .pData=Q_f32};
arm_matrix_instance_f32 x = {.numCols=1, .numRows=7, .pData=x_f32};
arm_matrix_instance_f32 F = {.numCols=7, .numRows=7, .pData=F_f32};
arm_matrix_instance_f32 P = {.numCols=7, .numRows=7, .pData=P_f32};
arm_matrix_instance_f32 z = {.numCols=1, .numRows=3, .pData=z_f32};
arm_matrix_instance_f32 h = {.numCols=1, .numRows=3, .pData=h_f32};
arm_matrix_instance_f32 y = {.numCols=1, .numRows=3, .pData=y_f32};
arm_matrix_instance_f32 H = {.numCols=7, .numRows=3, .pData=H_f32};
arm_matrix_instance_f32 S = {.numCols=3, .numRows=3, .pData=S_f32};
arm_matrix_instance_f32 R = {.numCols=3, .numRows=3, .pData=R_f32};
arm_matrix_instance_f32 K = {.numCols=3, .numRows=3, .pData=K_f32};

void ekf_init()
{
	arm_mat_scale_f32(&R,10000, &R);

	arm_mat_scale_f32(&Q,10000, &Q);

	arm_mat_scale_f32(&x,10000, &x);
	
	arm_mat_scale_f32(&F,10000, &F);
	
	arm_mat_scale_f32(&P,10000, &P);
	
	arm_mat_scale_f32(&z,10000, &z);

	arm_mat_scale_f32(&h,10000, &h);
	
	arm_mat_scale_f32(&y,10000, &y);
	
	arm_mat_scale_f32(&H,10000, &H);
	
	arm_mat_scale_f32(&K,10000, &K);

	float32_t pi = 3.1415;
	float32_t q0 = x_f32[0];
    float32_t q1 = x_f32[1];
    float32_t q2 = x_f32[2];
    float32_t q3 = x_f32[3];
    float32_t wxb = x_f32[4];
    float32_t wyb = x_f32[5];
    float32_t wzb = x_f32[6];
    float32_t wx = -CAL_D * (pi / 180);
    float32_t wy = CAL_D * (pi / 180);
    float32_t wz = CAL_D * (pi / 180);

	float32_t dt = 50.0;
	float32_t *Fp = F_f32;
	float32_t *Hp = H_f32;

	float32_t norm = 0;


	dt = 1/dt;
	dt = dt/2.0;

	x_f32[0] = q0 + dt * (-q1*(wx-wxb) - q2*(wy-wyb) - q3*(wz-wzb));
	x_f32[1] = q1 + dt * ( q0*(wx-wxb) - q3*(wy-wyb) + q2*(wz-wzb));
    x_f32[2] = q2 + dt * ( q3*(wx-wxb) + q0*(wy-wyb) - q1*(wz-wzb));
    x_f32[3] = q3 + dt * (-q2*(wx-wxb) + q1*(wy-wyb) + q0*(wz-wzb));

	arm_sqrt_f32(x_f32[0]*x_f32[0] +x_f32[1]*x_f32[1] + x_f32[2]*x_f32[2] + x_f32[3]*x_f32[3], &norm);

	norm = 1/norm;
    x_f32[0] = x_f32[0] * norm;
    x_f32[1] = x_f32[1] * norm;
    x_f32[2] = x_f32[2] * norm;
    x_f32[3] = x_f32[3] * norm;

	q0 = x_f32[0];
    q1 = x_f32[1];
    q2 = x_f32[2];
    q3 = x_f32[3];
	
	*Fp++ = 1;
    *Fp++ = -dt*(wx-wxb);
    *Fp++ = -dt*(wy-wyb);
    *Fp++ = -dt*(wz-wzb);
    *Fp++ =  dt*q1;
    *Fp++ =  dt*q2;
    *Fp++ =  dt*q3;
    *Fp++ =  dt*(wx-wxb);
    *Fp++ = 1;
    *Fp++ =  dt*(wz-wzb);
    *Fp++ = -dt*(wy-wyb);
    *Fp++ = -dt*q0;
    *Fp++ =  dt*q3;
    *Fp++ = -dt*q2;
    *Fp++ =  dt*(wy-wyb);
    *Fp++ = -dt*(wz-wzb);
    *Fp++ = 1;
    *Fp++ =  dt*(wx-wxb);
    *Fp++ = -dt*q3;
    *Fp++ = -dt*q0;
    *Fp++ =  dt*q1;
    *Fp++ =  dt*(wz-wzb);
    *Fp++ =  dt*(wy-wyb);
    *Fp++ = -dt*(wx-wxb);
    *Fp++ = 1;
    *Fp++ =  dt*q2;
    *Fp++ = -dt*q1;
    *Fp++ = -dt*q0;
    *Fp++ = 0;
    *Fp++ = 0;
    *Fp++ = 0;
    *Fp++ = 0;
    *Fp++ = 1;
    *Fp++ = 0;
    *Fp++ = 0;
    *Fp++ = 0;
    *Fp++ = 0;
    *Fp++ = 0;
    *Fp++ = 0;
    *Fp++ = 0;
    *Fp++ = 1;
    *Fp++ = 0;
    *Fp++ = 0;
    *Fp++ = 0;
    *Fp++ = 0;
    *Fp++ = 0;
    *Fp++ = 0;
    *Fp++ = 0;
    *Fp++ = 1;

	arm_mat_mult_f32(&F, &P, &TMP_7_7);
	arm_mat_trans_f32(&F, &TMP3_7_7);
	arm_mat_mult_f32(&TMP_7_7, &TMP3_7_7, &TMP2_7_7);
	arm_mat_add_f32(&TMP2_7_7,&Q, &P);

	// UPDATE 
	q0 = x_f32[0];
    q1 = x_f32[1];
    q2 = x_f32[2];
    q3 = x_f32[3];

	arm_sqrt_f32(CAL_D*CAL_D + CAL_D*CAL_D + CAL_D*CAL_D, &norm);
	norm = 1/ norm;

    z_f32[0] = CAL_D * norm;
    z_f32[1] = CAL_D * norm;
    z_f32[2] = CAL_D * norm;

	h_f32[0] = 2*q0*q2 - 2*q1*q3;
    h_f32[1] = -2*q0*q1 - 2*q2*q3;
    h_f32[2] = -q0*q0 + q1*q1 + q2*q2 - q3*q3;

	arm_mat_sub_f32(&z, &h, &y);

	*Hp++ = 2*q2;
    *Hp++ = -2*q3;
    *Hp++ = 2*q0;
    *Hp++ = -2*q1;
    *Hp++ = 0;
    *Hp++ = 0;
    *Hp++ = 0;
    *Hp++ = -2*q1;
    *Hp++ = -2*q0;
    *Hp++ = -2*q3;
    *Hp++ = -2*q2;
    *Hp++ = 0;
    *Hp++ = 0;
    *Hp++ = 0;
    *Hp++ = -2*q0;
    *Hp++ = 2*q1;
    *Hp++ = 2*q2;
    *Hp++ = -2*q3;
    *Hp++ = 0;
    *Hp++ = 0;
    *Hp++ = 0;

	TMP2_7_7.numRows = 7;
	TMP2_7_7.numCols = 3;

	arm_mat_trans_f32(&H, &TMP2_7_7);
	arm_mat_mult_f32(&H,&P,&TMP_7_7);
	arm_mat_mult_f32(&TMP_7_7, &TMP2_7_7,&TMP3_7_7);
	arm_mat_add_f32(&TMP3_7_7, &R, &S);

	arm_mat_inverse_f32(&S, &TMP_7_7);
	arm_mat_trans_f32(&H, &TMP2_7_7);
	arm_mat_mult_f32(&P, &TMP2_7_7,&TMP3_7_7);
	arm_mat_mult_f32(&TMP3_7_7, &TMP_7_7, &P);

	arm_mat_mult_f32(&K, &y, &TMP2_7_7);
	arm_mat_add_f32(&x,&TMP2_7_7, &TMP_7_7);

	arm_sqrt_f32(x_f32[0]*x_f32[0] +x_f32[1]*x_f32[1] + x_f32[2]*x_f32[2] + x_f32[3]*x_f32[3], &norm);

	norm = 1/norm;
    x_f32[0] = x_f32[0] * norm;
    x_f32[1] = x_f32[1] * norm;
    x_f32[2] = x_f32[2] * norm;
    x_f32[3] = x_f32[3] * norm;



}

void predict()
{

}