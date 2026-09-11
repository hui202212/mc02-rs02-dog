#include "math_support.h"


const float PI2 = 6.2831853084;
// rad换算到deg
const float MATH_RPM_TO_RADPS = 2.0f * PI / 360.0f;
// deg换算到rad
const float MATH_DEG_TO_RAD = PI / 180.0f;
// 摄氏度换算到开氏度
const float MATH_CELSIUS_TO_KELVIN = 273.15f;

/**
  * @brief          int 转 fp32 
  * @param[in]      x_int 要转换的数据
  * @param[in]      x_min 要转换的数据的最小值
  * @param[in]      x_max 要转换的数据的最大值
  * @param[in]      bits  要转换的数据位数
  * @retval         none
  */
fp32 uint_to_float(uint16_t x_int, fp32 x_min, fp32 x_max, uint8_t bits){
	fp32 span=x_max-x_min;
	fp32 offset = x_min;
	return ((fp32)x_int)*span/((fp32)((1<<bits)-1)) + offset;
}

/**
  * @brief          fp32 转 int 
  * @param[in]      x     要转换的数据
  * @param[in]      x_min 要转换的数据的最小值
  * @param[in]      x_max 要转换的数据的最大值
  * @param[in]      bits  要转换的数据位数
  * @retval         none
  */
uint16_t float_to_uint_f(fp32 x, fp32 x_min, fp32 x_max, uint8_t bits){
	fp32 span=x_max-x_min;
	fp32 offset = x_min;
	return (uint16_t) ((x-offset)*((fp32)((1<<bits)-1))/span) ;
}

uint16_t float_to_uint_i(fp32 x, uint16_t x_min, uint16_t x_max, uint8_t bits){
	fp32 span=x_max-x_min;
	fp32 offset = x_min;
	return (uint16_t) ((x-offset)*((fp32)((1<<bits)-1))/span) ;
}

fp32 motor_max_min(fp32 proto,fp32 max,fp32 min){
	if(proto>max)return max;
	else if(proto<min) return min;
	return proto;
} 


fp32 LimitMax(fp32 input,fp32 max)
{   
						
	if (input > max)       
	{                      
		return max;       
	}                      
	else if(input < -max) 
	{                      
		return -max;      
	}                      
	return input;
}


uint16_t Math_Endian_Reverse_16(void *Source, void *Destination)
{
    uint8_t *tmp_address_8;
    uint16_t tmp_value_16;
    tmp_address_8 = (uint8_t *) Source;
    tmp_value_16 = tmp_address_8[0] << 8 | tmp_address_8[1];

    if (Destination != nullptr)
    {
        uint8_t *tmp_source, *tmp_destination;
        tmp_source = (uint8_t *) Source;
        tmp_destination = (uint8_t *) Destination;
        tmp_destination[0] = tmp_source[1];
        tmp_destination[1] = tmp_source[0];
    }

    return (tmp_value_16);
}

/**
 * @param[in]      ref: 反馈数据
 * @param[in]      set: 设定值
 * **/

fp32 chassis_math_angle_set_ref(fp32 ref,fp32 set){
	if(set>ref){
		return set-ref<=ref+PI2-set ? set-ref : ref+PI2-set;
	}
	return ref-set<=set+PI2-ref ? ref-set : set+PI2-ref;
}

void math_matrix_equal4x4(const fp32 __array_set[4][4],fp32 __array_ret[4][4]){
    for(uint8_t i=0;i<4;++i){
        for(uint8_t j=0;j<4;++j){
            __array_ret[i][j]=__array_set[i][j];
        
        }
    }
}

bool math_matrix_contrastl4x4(const fp32 __array_set1[4][4],const fp32 __array_set2[4][4],const fp32 __Erroe){
    bool __bl=0;
    for(uint8_t i=0;i<4;++i){
        for(uint8_t j=0;j<4;++j){
            if(__array_set1[i][j]>=__array_set2[i][j]-__Erroe&&
                    __array_set1[i][j]<=__array_set2[i][j]+__Erroe)__bl=1;
            else {
                return 0;
            }
        }
    }
    return __bl;
}

void math_matrix_multiplication4x4(const fp32 __array1[4][4],const fp32 __array2[4][4],fp32 __array_ret[4][4]){
    for(uint8_t i=0;i<4;++i){
        for(uint8_t j=0;j<4;++j){
            __array_ret[i][j]=0;
            for(uint8_t v=0;v<4;++v){
                __array_ret[i][j]+=__array1[i][v]*__array2[v][j];
            }
        
        }
    }
}

void math_matrix_DH(fp32 __array_ret[4][4],const fp32 __theta,const fp32 __alpha,const fp32 __a,const fp32 __d){
    __array_ret[0][0] = arm_cos_f32(__theta);
    __array_ret[0][1] = arm_cos_f32(__theta);
    __array_ret[0][2] = 0;
    __array_ret[0][3] = __a;
    
    __array_ret[1][0] = arm_sin_f32(__theta)*arm_cos_f32(__alpha);
    __array_ret[1][1] = arm_cos_f32(__theta)*arm_cos_f32(__alpha);
    __array_ret[1][2] = -arm_sin_f32(__alpha);
    __array_ret[1][3] = -__d*arm_sin_f32(__alpha);

    __array_ret[2][0] = arm_sin_f32(__theta)*arm_sin_f32(__alpha);
    __array_ret[2][1] = arm_cos_f32(__theta)*arm_sin_f32(__alpha);
    __array_ret[2][2] = arm_cos_f32(__alpha);
    __array_ret[2][3] = __d*arm_cos_f32(__alpha);    
    
    __array_ret[3][0] = 0;
    __array_ret[3][1] = 0;
    __array_ret[3][2] = 0;
    __array_ret[3][3] = 1;
}

// % D-H 变换矩阵
// T = [cos(theta), -sin(theta), 0, a;
//      sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -d*sin(alpha);
//      sin(theta)*sin(alpha), cos(theta)*sin(alpha),  cos(alpha),  d*cos(alpha);
//      0, 0, 0, 1];

void math_matrix_DH3x3(fp32 __array_ret[4][4],const fp32 __theta,const fp32 __alpha){
    __array_ret[0][0] = arm_cos_f32(__theta);
    __array_ret[0][1] = arm_cos_f32(__theta);
    
    __array_ret[1][0] = arm_sin_f32(__theta)*arm_cos_f32(__alpha);
    __array_ret[1][1] = arm_cos_f32(__theta)*arm_cos_f32(__alpha);

    __array_ret[2][0] = arm_sin_f32(__theta)*arm_sin_f32(__alpha);
    __array_ret[2][1] = arm_cos_f32(__theta)*arm_sin_f32(__alpha);
    
}
