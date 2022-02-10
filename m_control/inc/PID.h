#ifndef M_CONTROL_PID_H
#define M_CONTROL_PID_H

#include "m_math/inc/math.h"

typedef struct {

	/* Controller gains */
	float Kp;
	float Ki;
	float Kd;

	/* Derivative low-pass filter time constant */
	float tau;

	/* Output limits */
	float limMin;
	float limMax;
	
	/* Integrator limits */
	float limMinInt;
	float limMaxInt;

	/* Sample time (in seconds) */
	float T;

	/* Controller "memory" */
	float integrator;
	float prevError;			/* Required for integrator */
	float differentiator;
	float prevMeasurement;		/* Required for differentiator */

	/* Controller output */
	float out;

} pid_controller_t;

void  PIDController_Init(pid_controller_t *pid);
float32_t PIDController_Update(pid_controller_t *pid, float32_t setpoint, float32_t measurement);

#endif /*M_CONTROL_PID_H*/