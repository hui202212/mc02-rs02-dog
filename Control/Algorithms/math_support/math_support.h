#ifndef MATH_SUPPORT_H
#define MATH_SUPPORT_H

#include "struct_typedef.h"
#include "arm_math.h"


extern const float MATH_RPM_TO_RADPS;
extern const float MATH_DEG_TO_RAD;
extern const float MATH_CELSIUS_TO_KELVIN;
extern const float PI2;

template<typename Type>
Type Basic_Math_Abs(Type x)
{
    return ((x > 0) ? x : -x);
}


/**
  * @brief          int 转 fp32 
  * @param[in]      x_int 要转换的数据
  * @param[in]      x_min 要转换的数据的最小值
  * @param[in]      x_max 要转换的数据的最大值
  * @param[in]      bits  要转换的数据位数
  * @retval         none
  */

fp32 uint_to_float(uint16_t x_int, fp32 x_min, fp32 x_max, uint8_t bits);

/**
  * @brief          fp32 转 int 
  * @param[in]      x     要转换的数据
  * @param[in]      x_min 要转换的数据的最小值
  * @param[in]      x_max 要转换的数据的最大值
  * @param[in]      bits  要转换的数据位数
  * @retval         none
  */
uint16_t float_to_uint_f(fp32 x, fp32 x_min, fp32 x_max, uint8_t bits);

uint16_t float_to_uint_i(fp32 x, uint16_t x_min, uint16_t x_max, uint8_t bits);

fp32 motor_max_min(fp32 proto,fp32 max,fp32 min);

fp32 LimitMax(fp32 input,fp32 max);

fp32 chassis_math_angle_set_ref(fp32 ref,fp32 set);

uint16_t Math_Endian_Reverse_16(void *Source, void *Destination);

bool math_matrix_contrastl4x4(const fp32 __array_set1[4][4],const fp32 __array_set2[4][4],const fp32 __Erroe);

void math_matrix_equal4x4(const fp32 __array_set[4][4],fp32 __array_ret[4][4]);

void math_matrix_multiplication4x4(const fp32 __array1[4][4],const fp32 __array2[4][4],fp32 __array_ret[4][4]);

void math_matrix_DH(fp32 __array_ret[4][4],const fp32 __theta,const fp32 __alpha,const fp32 __a,const fp32 __d);

void math_matrix_DH3x3(fp32 __array_ret[4][4],const fp32 __theta,const fp32 __alpha);

#endif
