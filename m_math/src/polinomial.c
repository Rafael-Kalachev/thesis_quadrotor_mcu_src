#include "m_math/inc/polynomial.h"
#include "cmsis/inc/arm_math.h"


float32_t polinomial_evaluate(polynomial_struct_t *polinomial, float32_t x)
{
	float32_t result = polinomial->polynomial_coeficients_p[0];
	float32_t x_i = x;
	uint8_t i;


	for(i = 1; i <= polinomial->degree; ++i)
	{
		result = result + polinomial->polynomial_coeficients_p[i] * x_i;
		x_i = x_i * x;
	}

	return result;
}
