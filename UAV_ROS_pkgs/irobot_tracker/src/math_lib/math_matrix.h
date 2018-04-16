/**
  ******************************************************************************
  * @file    math_matrix.h
  * @author  Leonardo Yu Yun
  * @version V1.1.0
  * @date    20-March-2014
  * @brief   This file contains all the functions prototypes for 
  *			 matrix operations
  *         
  ******************************************************************************  
  */ 


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MATH_MATRIX_H
#define __MATH_MATRIX_H


/* Includes ------------------------------------------------------------------*/
#include "math_basic.h"


/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/  




void matrix3f_zero(matrix3f m);
void matrix3f_set_value(matrix3f m, float a11, float a12, float a13,
									float a21, float a22, float a23,
									float a31, float a32, float a33);
void matrix3f_eye(matrix3f m);
void matrix3f_copy(matrix3f dest, matrix3f src);

void matrix3f_add(matrix3f m_out, matrix3f a_in,matrix3f b_in);
void matrix3f_sub(matrix3f m_out, matrix3f a_in,matrix3f b_in);

void matrix3f_add3(matrix3f m_out, matrix3f a_in, matrix3f b_in, matrix3f c_in);

void matrix3f_scale(matrix3f m, float scale);
void matrix3f_multi_scaler(matrix3f m_out, matrix3f m_in, float scaler);

void get_matrix3f_transpose(matrix3f dest, matrix3f src);
void take_matrix3f_transpose(matrix3f m);
float matrix3f_det(matrix3f m);

void get_matrix3f_inverse(matrix3f m_out, matrix3f m_in);

void matrix3f_multi_matrix3f(matrix3f m_out, matrix3f a_in,matrix3f b_in);
void matrix3f_multi_vector3f(vector3f v_out, matrix3f m, vector3f v);



#endif /*__MATH_MATRIX_H */



/************************ (C) COPYRIGHT DJI ********************END OF FILE****/

