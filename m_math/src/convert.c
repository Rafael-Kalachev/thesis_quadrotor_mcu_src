
#include "m_math/inc/convert.h"
#include "cmsis/inc/arm_math.h"

static void reverse_str(char * in_out_str, int string_length)
{
    int i = 0;
    int j = string_length -1;
    char temp;
    while (i < j)
    {
        temp = in_out_str[i];
        in_out_str[i] = in_out_str[j];
        in_out_str[j] = temp;
        ++i;
        --j;
    }
}


int int_to_str(int number, char *out_str, uint8_t digits)
{
    int i = 0;
    int count = 0;

    if(number < 0)
    {
        number = - number;
        out_str[0]  = '-';
        ++out_str;
        ++count;
    }

    while(number)
    {
        out_str[i] = (number % 10) + '0';
        number = number / 10;
        ++i;
    }

    /*If number of digits required is more,
     * then add 0s at the beginning 
     */
    while (i < digits)
    {
        out_str[i] = '0';
        ++i;
    }

    count = count + i;

    reverse_str(out_str, i);
    out_str[i] = '\0';

    return count;
}


int float_to_str(float32_t number, char* out_str, uint8_t after_point, uint8_t before_point)
{
    int count = 0;
    int integer_part ;
    float32_t float_part;
    int i;
    int j;

    if(number < 0)
    {
        number = - number;
        out_str[0]  = '-';
        ++out_str;
        ++count;
    }

    integer_part = (int) number;
    float_part = number - (float32_t)integer_part;
    i = int_to_str(integer_part, out_str, before_point);
    count = count + i;

    if (after_point != 0)
    {
        out_str[i] = '.';
        ++count;
        int pow = 1;
        int base = 10;
        for (j = 0; j < after_point; ++j)
        {
            pow = pow * base;
        }
        float_part = float_part * (float32_t) pow;

        int_to_str((int)float_part, out_str + i + 1, after_point);
        count = count + after_point;
    }

    return count;
}
