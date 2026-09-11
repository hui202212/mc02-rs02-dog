/**
 * @file    kalman_filter.h
 * @brief   通用卡尔曼滤波框架（保持 arm_math 依赖）
 *
 * 从原项目复制，仅调整头文件路径。
 * 依赖 CMSIS-DSP (arm_math.h)。
 */
#ifndef __KALMAN_FILTER_H
#define __KALMAN_FILTER_H

#include "arm_math.h"
#include "math.h"
#include "stdint.h"
#include "stdlib.h"

#ifndef user_malloc
#ifdef _CMSIS_OS_H
#define user_malloc pvPortMalloc
#else
#define user_malloc malloc
#endif
#endif

#define mat arm_matrix_instance_f32
#define Matrix_Init arm_mat_init_f32
#define Matrix_Add arm_mat_add_f32
#define Matrix_Subtract arm_mat_sub_f32
#define Matrix_Multiply arm_mat_mult_f32
#define Matrix_Transpose arm_mat_trans_f32
#define Matrix_Inverse arm_mat_inverse_f32

typedef struct kf_t
{
    float *FilteredValue;
    float *MeasuredVector;
    float *ControlVector;

    uint8_t xhatSize;
    uint8_t uSize;
    uint8_t zSize;

    uint8_t UseAutoAdjustment;
    uint8_t MeasurementValidNum;

    uint8_t *MeasurementMap;
    float *MeasurementDegree;
    float *MatR_DiagonalElements;
    float *StateMinVariance;
    uint8_t *temp;

    uint8_t SkipEq1, SkipEq2, SkipEq3, SkipEq4, SkipEq5;

    mat xhat;
    mat xhatminus;
    mat u;
    mat z;
    mat P;
    mat Pminus;
    mat F, FT, temp_F;
    mat B;
    mat H, HT;
    mat Q;
    mat R;
    mat K;
    mat S, temp_matrix, temp_matrix1, temp_vector, temp_vector1;

    int8_t MatStatus;

    void (*User_Func0_f)(struct kf_t *kf);
    void (*User_Func1_f)(struct kf_t *kf);
    void (*User_Func2_f)(struct kf_t *kf);
    void (*User_Func3_f)(struct kf_t *kf);
    void (*User_Func4_f)(struct kf_t *kf);
    void (*User_Func5_f)(struct kf_t *kf);
    void (*User_Func6_f)(struct kf_t *kf);

    float *xhat_data, *xhatminus_data;
    float *u_data;
    float *z_data;
    float *P_data, *Pminus_data;
    float *F_data, *FT_data, *temp_F_data;
    float *B_data;
    float *H_data, *HT_data;
    float *Q_data;
    float *R_data;
    float *K_data;
    float *S_data, *temp_matrix_data, *temp_matrix_data1, *temp_vector_data, *temp_vector_data1;
} KalmanFilter_t;

extern uint16_t sizeof_float, sizeof_double;

void Kalman_Filter_Init(KalmanFilter_t *kf, uint8_t xhatSize, uint8_t uSize, uint8_t zSize);
void Kalman_Filter_Measure(KalmanFilter_t *kf);
void Kalman_Filter_xhatMinusUpdate(KalmanFilter_t *kf);
void Kalman_Filter_PminusUpdate(KalmanFilter_t *kf);
void Kalman_Filter_SetK(KalmanFilter_t *kf);
void Kalman_Filter_xhatUpdate(KalmanFilter_t *kf);
void Kalman_Filter_P_Update(KalmanFilter_t *kf);
float *Kalman_Filter_Update(KalmanFilter_t *kf);

#endif /* __KALMAN_FILTER_H */
