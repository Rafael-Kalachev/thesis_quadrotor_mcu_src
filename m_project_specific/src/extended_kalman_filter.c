#include "stm32f4xx.h"
#include "m_project_specific/inc/extended_kalman_filter.h"
#include "m_project_specific/inc/calibrate_sensors.h"
#include "m_project_specific/inc/usart.h"
#include "m_math/inc/convert.h"
#include "m_math/inc/math.h"
#include "m_math/inc/quaternion.h"




#define CAL_D 0.1567 /*calibration dat dont forget to change*/
float32_t Q_f32[7*7] =
{
	 0 ,  0 ,  0 ,  0 ,  0   ,  0   ,  0   ,  
	 0 ,  0 ,  0 ,  0 ,  0   ,  0   ,  0   ,  
	 0 ,  0 ,  0 ,  0 ,  0   ,  0   ,  0   ,  
	 0 ,  0 ,  0 ,  0 ,  0   ,  0   ,  0   ,  
	 0 ,  0 ,  0 ,  0 ,  0.2 ,  0   ,  0   ,  
	 0 ,  0 ,  0 ,  0 ,  0   ,  0.2 ,  0   ,  
	 0 ,  0 ,  0 ,  0 ,  0   ,  0   ,  0.2 
};

float32_t x_f32[7] = {1,0,0,0,0,0,0};

float32_t F_f32[7*7];

float32_t TMP_7_7_f32[7*7];
float32_t TMP2_7_7_f32[7*7];
float32_t TMP3_7_7_f32[7*7];


float32_t P_f32[7*7] =
{
	 1 ,  0 ,  0 ,  0 ,  0 ,  0 ,  0 ,  
	 0 ,  1 ,  0 ,  0 ,  0 ,  0 ,  0 ,  
	 0 ,  0 ,  1 ,  0 ,  0 ,  0 ,  0 ,  
	 0 ,  0 ,  0 ,  1 ,  0 ,  0 ,  0 ,  
	 0 ,  0 ,  0 ,  0 ,  1 ,  0 ,  0 ,  
	 0 ,  0 ,  0 ,  0 ,  0 ,  1 ,  0 ,  
	 0 ,  0 ,  0 ,  0 ,  0 ,  0 ,  1 
};

float32_t z_f32[3];

float32_t h_f32[3];

float32_t y_f32[3];

float32_t H_f32[7*7];

float32_t S_f32[3*3];

float32_t R_f32[3*3] = 
{
	 1 ,  0 ,  0,
	 0 ,  1 ,  0,
	 0 ,  0 ,  1
};

float32_t K_f32[3*7];

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
arm_matrix_instance_f32 K = {.numCols=3, .numRows=7, .pData=K_f32};

void ekf_init()
{
    arm_mat_scale_f32(&R,50, &R);
    USART_SendText("R = ");
    USART_SendMatrix(&R,5);
    arm_mat_scale_f32(&P,100000, &P);
    USART_SendText("P = ");
    USART_SendMatrix(&P,5);
}

void ekf_next(calibrated_sensors_struct* calibrated_sensors, quaternion_struct_t * out_quaternion)
{

	float32_t q0 = x_f32[0];
    float32_t q1 = x_f32[1];
    float32_t q2 = x_f32[2];
    float32_t q3 = x_f32[3];

    float32_t wxb = x_f32[4];
    float32_t wyb = x_f32[5];
    float32_t wzb = x_f32[6];

    float32_t wx = calibrated_sensors->gyro_x;
    float32_t wy = calibrated_sensors->gyro_y;
    float32_t wz = calibrated_sensors->gyro_z;

	float32_t dt = 50.0; //Hz (later inverted to period)
	float32_t *Fp = F_f32;
	float32_t *Hp = H_f32;

	float32_t norm = 0;


	dt = 1.0 / dt; //dt
	dt = dt / 2.0; // in all cases we use dt/2, so divided in advance to save processing time;

	x_f32[0] = q0 + dt * (-q1*(wx-wxb) - q2*(wy-wyb) - q3*(wz-wzb));
	x_f32[1] = q1 + dt * ( q0*(wx-wxb) - q3*(wy-wyb) + q2*(wz-wzb));
    x_f32[2] = q2 + dt * ( q3*(wx-wxb) + q0*(wy-wyb) - q1*(wz-wzb));
    x_f32[3] = q3 + dt * (-q2*(wx-wxb) + q1*(wy-wyb) + q0*(wz-wzb));


	norm = sqrtf(x_f32[0]*x_f32[0] +x_f32[1]*x_f32[1] + x_f32[2]*x_f32[2] + x_f32[3]*x_f32[3]);
	norm = 1.0 / norm;
    x_f32[0] = x_f32[0] * norm;
    x_f32[1] = x_f32[1] * norm;
    x_f32[2] = x_f32[2] * norm;
    x_f32[3] = x_f32[3] * norm;

    USART_SendText("x = ");
    USART_SendMatrix(&x,5);

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

    USART_SendText("F = ");
    USART_SendMatrix(&F,5);

	arm_mat_mult_f32(&F, &P, &TMP_7_7);
    USART_SendText("F*P = ");
    USART_SendMatrix(&TMP_7_7,5);

	arm_mat_trans_f32(&F, &TMP3_7_7);
    USART_SendText("F_trans = ");
    USART_SendMatrix(&TMP3_7_7,5);

	arm_mat_mult_f32(&TMP_7_7, &TMP3_7_7, &TMP2_7_7);
    USART_SendText("F*P*F_trans = ");
    USART_SendMatrix(&TMP2_7_7,5);

	arm_mat_add_f32(&TMP2_7_7,&Q, &P);
    USART_SendText("F*P*F_trans + Q = P = ");
    USART_SendMatrix(&P,5);

	// UPDATE 
	q0 = x_f32[0];
    q1 = x_f32[1];
    q2 = x_f32[2];
    q3 = x_f32[3];

	norm = sqrtf(calibrated_sensors->acc_x*calibrated_sensors->acc_x + calibrated_sensors->acc_y*calibrated_sensors->acc_y +  calibrated_sensors->acc_z*calibrated_sensors->acc_z);
	norm = 1.0 / norm;

    z_f32[0] = calibrated_sensors->acc_x * norm;
    z_f32[1] = calibrated_sensors->acc_y * norm;
    z_f32[2] = -calibrated_sensors->acc_z * norm;
    USART_SendText("z = ");
    USART_SendMatrix(&z,5);

	h_f32[0] = 2*q0*q2 - 2*q1*q3;
    h_f32[1] = -2*q0*q1 - 2*q2*q3;
    h_f32[2] = -q0*q0 + q1*q1 + q2*q2 - q3*q3;

    USART_SendText("h = ");
    USART_SendMatrix(&h,5);

	arm_mat_sub_f32(&z, &h, &y);
    USART_SendText("y = ");
    USART_SendMatrix(&y,5);

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


    USART_SendText("H = ");
    USART_SendMatrix(&H,5);

	arm_mat_trans_f32(&H, &TMP2_7_7);
    USART_SendText("H_trans = ");
    USART_SendMatrix(&TMP2_7_7,5);

	arm_mat_mult_f32(&H,&P,&TMP_7_7);
    USART_SendText("H*P = ");
    USART_SendMatrix(&TMP_7_7,5);

	arm_mat_mult_f32(&TMP_7_7, &TMP2_7_7,&TMP3_7_7);
    USART_SendText("H*P*H_trans = ");
    USART_SendMatrix(&TMP3_7_7,5);

	arm_mat_add_f32(&TMP3_7_7, &R, &S);
    USART_SendText("H*P*H_trans+R = S = ");
    USART_SendMatrix(&S,5);

	arm_mat_inverse_f32(&S, &TMP_7_7);
    USART_SendText("S_inv = ");
    USART_SendMatrix(&TMP_7_7,5);

	arm_mat_trans_f32(&H, &TMP2_7_7);
    USART_SendText("H_trans = ");
    USART_SendMatrix(&TMP2_7_7,5);

	arm_mat_mult_f32(&P, &TMP2_7_7,&TMP3_7_7);
    USART_SendText("P*H_trans = ");
    USART_SendMatrix(&TMP3_7_7,5);

	arm_mat_mult_f32(&TMP3_7_7, &TMP_7_7, &K);

	arm_mat_mult_f32(&K, &y, &TMP2_7_7);
	arm_mat_add_f32(&x,&TMP2_7_7, &TMP_7_7);
    arm_mat_scale_f32(&TMP_7_7, 1.0, &x);

	arm_sqrt_f32(x_f32[0]*x_f32[0] +x_f32[1]*x_f32[1] + x_f32[2]*x_f32[2] + x_f32[3]*x_f32[3], &norm);

	norm = 1/norm;
    x_f32[0] = x_f32[0] * norm;
    x_f32[1] = x_f32[1] * norm;
    x_f32[2] = x_f32[2] * norm;
    x_f32[3] = x_f32[3] * norm;

    TMP_7_7_f32[0] = 1;
    TMP_7_7_f32[1] = 0;
    TMP_7_7_f32[2] = 0;
    TMP_7_7_f32[3] = 0;
    TMP_7_7_f32[4] = 1;
    TMP_7_7_f32[5] = 0;
    TMP_7_7_f32[6] = 0;
    TMP_7_7_f32[7] = 0;
    TMP_7_7_f32[8] = 1;
    TMP_7_7.numCols=3;
    TMP_7_7.numRows=3;

    arm_mat_trans_f32(&H, &TMP2_7_7);
    arm_mat_sub_f32(&TMP_7_7, &TMP2_7_7, &TMP3_7_7);
    arm_mat_mult_f32(&TMP3_7_7, &P, &TMP2_7_7);
    arm_mat_scale_f32(&TMP2_7_7, 1, &P);

    out_quaternion->w = x_f32[0];
    out_quaternion->x = x_f32[1];
    out_quaternion->y = x_f32[2];
    out_quaternion->z = x_f32[3];

}


void quaternion_to_orientation(quaternion_struct_t * q, orientation_struct_t *out_orientation)
{
    out_orientation->pitch = -(float32_t)RAD2DEG_CONV *  atan2f(2*(q->w*q->x + q->y*q->z), 1 - 2*((q->x*q->x)+(q->y*q->y)));
    out_orientation->roll  =  (float32_t)RAD2DEG_CONV *  asinf(2*(q->w*q->y - q->z*q->x));
    out_orientation->yaw   =  (float32_t)RAD2DEG_CONV *  atan2f(2*(q->w*q->z + q->x*q->y), 1 - 2*((q->y*q->y)+(q->z*q->z)));

}


#define Q_phi  2.0
#define Q_psi  2.0
#define Q_bp   0.005
#define Q_bq   0.005
#define R_phi  150.0
#define R_psi  150.0

float32_t Q_kf_f32[4*4] = 
{
    Q_phi,     0,    0,    0,
        0, Q_psi,    0,    0,
        0,     0, Q_bp,    0,
        0,     0,    0, Q_bq
};
arm_matrix_instance_f32 Q_kf = {.numCols=4, .numRows=4, .pData=Q_kf_f32};

float32_t R_kf_f32[4*4] = 
{
     R_phi,     0,  0,  0,
         0, R_psi,  0,  0,
         0,     0,  0,  0,
         0,     0,  0,  0
};
arm_matrix_instance_f32 R_kf = {.numCols=4, .numRows=4, .pData=R_kf_f32};


float32_t P_kf_f32[4*4] = 
{
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 10000, 0,
    0, 0, 0, 10000
};
arm_matrix_instance_f32 P_kf = {.numCols=4, .numRows=4, .pData=P_kf_f32};


float32_t H_kf_f32[4*4] = 
{  
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0
};
arm_matrix_instance_f32 H_kf = {.numCols=4, .numRows=4, .pData=H_kf_f32};


float32_t I_kf_f32[4*4] =
{
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0,
    0, 0, 0, 1
};
arm_matrix_instance_f32 I_kf = {.numCols=4, .numRows=4, .pData=I_kf_f32};


float32_t S_kf_f32[4*4] =
{
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0,
    0, 0, 0, 1
};
arm_matrix_instance_f32 S_kf = {.numCols=4, .numRows=4, .pData=S_kf_f32};

float32_t K_kf_f32[4*4] =
{
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0,
    0, 0, 0, 1
};
arm_matrix_instance_f32 K_kf = {.numCols=4, .numRows=4, .pData=K_kf_f32};


float32_t x_kf_f32[1*4] =
{
     1,
     1,
     0,
     0
};
arm_matrix_instance_f32 x_kf = {.numCols=1, .numRows=4, .pData=x_kf_f32};

float32_t y_kf_f32[1*4] =
{
     0,
     0,
     0,
     0
};
arm_matrix_instance_f32 y_kf = {.numCols=1, .numRows=4, .pData=y_kf_f32};



float32_t F_kf_f32[4*4]  = 
{
    1, 0, -(1.0/50.0),       0,
    0, 1,       0, -(1.0/50.0),
    0, 0,       1,       0,
    0, 0,       0,       1
};
arm_matrix_instance_f32 F_kf = {.numCols=4, .numRows=4, .pData=F_kf_f32};

    
float32_t B_kf_f32[4*4] = 
{
    (1.0/50.0),      0,   0, 0,
         0, (1.0/50.0),   0, 0,
         0,      0,   0, 0,
         0,      0,   0, 0
};
arm_matrix_instance_f32 B_kf = {.numCols=4, .numRows=4, .pData=B_kf_f32};




float32_t u_kf_f32[1*4] = 
{
    0,
    0,
    0,
    0
};
arm_matrix_instance_f32 u_kf = {.numCols=1, .numRows=4, .pData=u_kf_f32};

    
float32_t z_kf_f32[1*4] = 
{
    0,
    0,
    0,
    0
};
arm_matrix_instance_f32 z_kf = {.numCols=1, .numRows=4, .pData=z_kf_f32};

float32_t tmp1_1_4_kf_f32[1*4] = 
{
    0,
    0,
    0,
    0
};
arm_matrix_instance_f32 tmp1_1_4_kf = {.numCols=1, .numRows=4, .pData=tmp1_1_4_kf_f32};

float32_t tmp2_1_4_kf_f32[1*4] = 
{
    0,
    0,
    0,
    0
};
arm_matrix_instance_f32 tmp2_1_4_kf = {.numCols=1, .numRows=4, .pData=tmp2_1_4_kf_f32};

    
float32_t tmp1_4_4_kf_f32[4*4] = 
{
    (1/50),      0,   0, 0,
         0, (1/50),   0, 0,
         0,      0,   0, 0,
         0,      0,   0, 0
};
arm_matrix_instance_f32 tmp1_4_4_kf = {.numCols=4, .numRows=4, .pData=tmp1_4_4_kf_f32};

float32_t tmp2_4_4_kf_f32[4*4] = 
{
    (1/50),      0,   0, 0,
         0, (1/50),   0, 0,
         0,      0,   0, 0,
         0,      0,   0, 0
};
arm_matrix_instance_f32 tmp2_4_4_kf = {.numCols=4, .numRows=4, .pData=tmp2_4_4_kf_f32};

float32_t tmp3_4_4_kf_f32[4*4] = 
{
    (1/50),      0,   0, 0,
         0, (1/50),   0, 0,
         0,      0,   0, 0,
         0,      0,   0, 0
};
arm_matrix_instance_f32 tmp3_4_4_kf = {.numCols=4, .numRows=4, .pData=tmp3_4_4_kf_f32};


void kf_init()
{
 __NOP;

}

void pinv(float32_t *a, int m, int n, float32_t *X);
#define PRECISION1 32768
#define PRECISION2 16384
/*#define PI 3.1415926535897932*/
#ifndef MIN
#define MIN(x,y) ( (x) < (y) ? (x) : (y) )
#endif

#ifndef MAX
#define MAX(x,y) ((x)>(y)?(x):(y))
#endif

#define SIGN(a, b) ((b) >= 0.0 ? fabs(a) : -fabs(a))
#define MAXINT 2147483647
#define ASCII_TEXT_BORDER_WIDTH 4
#define MAXHIST 100
#define STEP0 0.01
#define FORWARD 1
#define BACKWARD -1
#define PROJ_DIM 5
#define True 1
#define False 0


typedef struct {
	float x, y, z;
} fcoords;

typedef struct {
	long x, y, z;
} lcoords;

typedef struct {
	int x, y, z;
} icoords;

typedef struct {
	float min, max;
} lims;


/* grand tour history */
typedef struct hist_rec {
  struct hist_rec *prev, *next;
  float *basis[3];
  int pos;
} hist_rec;



void kf_next(calibrated_sensors_struct* calibrated_sensors, orientation_struct_t * out_orientation)
{
    float32_t phi_acc = atan2f(calibrated_sensors->acc_x, calibrated_sensors->acc_z) * RAD2DEG_CONV;
    float32_t psi_acc = atan2f(calibrated_sensors->acc_y, calibrated_sensors->acc_z) * RAD2DEG_CONV;
    float32_t phi_dot = calibrated_sensors->gyro_x;
    float32_t psi_dot = calibrated_sensors->gyro_y;

    u_kf.pData[0] = phi_dot;
    u_kf.pData[1] = psi_dot;
    u_kf.pData[2] = 0;
    u_kf.pData[3] = 0;

    z_kf.pData[0] = phi_acc;
    z_kf.pData[1] = psi_acc;
    z_kf.pData[2] = 0;
    z_kf.pData[3] = 0;

 //   USART_SendText("x =\n");
//    USART_SendMatrix(&x_kf, 5);
   // USART_SendText("u =\n");
  //  USART_SendMatrix(&u_kf, 5);
 //   USART_SendText("z =\n");
//    USART_SendMatrix(&z_kf, 5);

//    USART_SendText("F =\n");
//    USART_SendMatrix(&F_kf, 5);

    arm_mat_mult_f32(&F_kf, &x_kf, &tmp1_1_4_kf);
//    USART_SendText("tmp114 =\n");
//    USART_SendMatrix(&tmp1_1_4_kf, 5);
//        USART_SendText("B =\n");
//    USART_SendMatrix(&B_kf, 5);

    arm_mat_mult_f32(&B_kf, &u_kf, &tmp2_1_4_kf);
//        USART_SendText("tmp214 =\n");
//    USART_SendMatrix(&tmp2_1_4_kf, 5);
    arm_mat_add_f32(&tmp1_1_4_kf, &tmp2_1_4_kf, &x_kf);
//    USART_SendText("x =\n");
//    USART_SendMatrix(&x_kf, 5);

//    USART_SendText("F =\n");
//    USART_SendMatrix(&F_kf, 5);
//        USART_SendText("P=\n");
//    USART_SendMatrix(&P_kf, 5);
    arm_mat_mult_f32(&F_kf, &P_kf, &tmp1_4_4_kf);
//    USART_SendText("FP=\n");
//    USART_SendMatrix(&tmp1_4_4_kf, 5);
    arm_mat_trans_f32(&F_kf, &tmp2_4_4_kf);
//    USART_SendText("F_t=\n");
//    USART_SendMatrix(&tmp2_4_4_kf, 5);
    arm_mat_mult_f32(&tmp1_4_4_kf, &tmp2_4_4_kf, &tmp3_4_4_kf);
//        USART_SendText("FPF_t=\n");
//    USART_SendMatrix(&tmp3_4_4_kf, 5);
    arm_mat_scale_f32(&Q_kf, 1.0/50.0, &tmp1_4_4_kf);
    arm_mat_add_f32(&tmp3_4_4_kf, &tmp1_4_4_kf, &P_kf);
//            USART_SendText("P=\n");
//    USART_SendMatrix(&P_kf, 5);

  //          USART_SendText("H=\n");
 //   USART_SendMatrix(&H_kf, 5);
  //              USART_SendText("x=\n");
//USART_SendMatrix(&x_kf, 5);
    arm_mat_mult_f32(&H_kf, &x_kf, &tmp1_1_4_kf);
  //              USART_SendText("Hx=\n");
//USART_SendMatrix(&tmp1_1_4_kf, 5);
    arm_mat_sub_f32(&z_kf, &tmp1_1_4_kf, &y_kf);
 //   USART_SendText("z- Hx=y=\n");
   // USART_SendMatrix(&y_kf, 5);

 //   USART_SendText("============================================");
//    USART_SendText("H=\n");
//    USART_SendMatrix(&H_kf, 5);
//    USART_SendText("P=\n");
//    USART_SendMatrix(&P_kf, 5);
    arm_mat_mult_f32(&H_kf, &P_kf, &tmp1_4_4_kf);
//    USART_SendText("HP=\n");
//    USART_SendMatrix(&tmp1_4_4_kf, 5);
    arm_mat_trans_f32(&H_kf, &tmp2_4_4_kf);
//    USART_SendText("H_t=\n");
//    USART_SendMatrix(&tmp2_4_4_kf, 5);
    arm_mat_mult_f32(&tmp1_4_4_kf, &tmp2_4_4_kf, &tmp3_4_4_kf);

    arm_mat_add_f32(&tmp3_4_4_kf, &R_kf, &S_kf);

//        USART_SendText("S=\n");
//    USART_SendMatrix(&S_kf, 5);

    arm_mat_trans_f32(&H_kf, &tmp2_4_4_kf);
    arm_mat_mult_f32(&P_kf, &tmp2_4_4_kf, &tmp1_4_4_kf);
   // arm_mat_inverse_f32(&S_kf, &tmp3_4_4_kf);
    pinv(S_kf.pData, 4,4,tmp3_4_4_kf.pData);
//        USART_SendText("s_n=\n");
  //  USART_SendMatrix(&S_kf, 5);
 //       USART_SendText("S_inv=\n");
//    USART_SendMatrix(&tmp3_4_4_kf, 5);
    arm_mat_mult_f32(&tmp1_4_4_kf, &tmp3_4_4_kf, &K_kf);

    arm_mat_mult_f32(&K_kf, &y_kf, &tmp1_1_4_kf);
    arm_mat_scale_f32(&x_kf, 1.000, &tmp2_1_4_kf);
    arm_mat_add_f32(&tmp2_1_4_kf, &tmp1_1_4_kf, &x_kf);

    arm_mat_mult_f32(&K_kf, &H_kf, &tmp1_4_4_kf);
    arm_mat_add_f32(&tmp1_4_4_kf, &I_kf, &tmp2_4_4_kf);
    arm_mat_mult_f32(&P_kf, &tmp2_4_4_kf, &tmp1_1_4_kf);
    arm_mat_scale_f32(&tmp1_1_4_kf, 1.00, &P_kf);

    out_orientation->pitch = x_kf.pData[0];
    out_orientation->roll = x_kf.pData[1];

}




void pinv(float32_t *a, int m, int n, float32_t *X)
{
    for(int i=0; i<n;++i)
    {
        for(int j=0; j<m;++j)
        {
            *(X+(j+i*m)) = 0;
            if (i==j)
            {
                if( *(a+(j+i*m)) > 0.000001 )
                {
                    *(X+(j+i*m)) = 1.0/(*(a+(j+i*m)));
                }
            }
        }
    }
}