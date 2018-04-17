/**
  ******************************************************************************
  * @file    math_vector.h
  * @author  Leonardo Yu Yun
  * @version V1.1.0
  * @date    20-March-2014
  * @brief   This file contains all the functions prototypes for 
  *			 vector operations
  *         
  ******************************************************************************  
  */ 


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MATH_VECTOR_H
#define __MATH_VECTOR_H


/* Includes ------------------------------------------------------------------*/
#include "math_basic.h"


/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/  


void vector2f_zero(vector2f v);
void vector3f_zero(vector3f v);
void vector4f_zero(vector4f v);

void vector2f_set_value(vector2f v, float v0, float v1);
void vector3f_set_value(vector3f v, float v0, float v1,float v2);
void vector4f_set_value(vector4f v, float v0, float v1, float v2, float v3);

void vector2f_copy(vector2f dest, vector2f src);
void vector3f_copy(vector3f dest, vector3f src);
void vector4f_copy(vector4f dest, vector4f src);

void vector2f_add(vector2f v_out, vector2f a_in,vector2f b_in);
void vector3f_add(vector3f v_out, vector3f a_in,vector3f b_in);
void vector4f_add(vector4f v_out, vector4f a_in,vector4f b_in);

void vector3f_add3(vector3f v_out, vector3f a_in, vector3f b_in, vector3f c_in);


void vector2f_sub(vector2f v_out, vector2f a_in,vector2f b_in);
void vector3f_sub(vector3f v_out, vector3f a_in,vector3f b_in);
void vector4f_sub(vector4f v_out, vector4f a_in,vector4f b_in);

void vector2f_scale(vector2f v, float scale);
void vector3f_scale(vector3f v, float scale);
void vector4f_scale(vector4f v, float scale);

void vector2f_multi_scaler(vector2f v_out, const vector2f v_in, const float scaler);
void vector3f_multi_scaler(vector3f v_out, const vector3f v_in, const float scaler);
void vector4f_multi_scaler(vector4f v_out, const vector4f v_in, const float scaler);


float vector2f_norm(const vector2f v);
float vector3f_norm(const vector3f x);
float vector4f_norm(const vector4f x);

float vector2f_inv_norm(const vector2f v);
float vector3f_inv_norm(const vector3f x);
float vector4f_inv_norm(const vector4f x);

void vector2f_normalize(vector2f v);
void vector3f_normalize(vector3f v);
void vector4f_normalize(vector4f v);

float vector2f_dot_product(const vector2f a_in, const vector2f b_in);
float vector3f_dot_product(const vector3f a_in, const vector3f b_in);

float vector2f_cross_product( const vector2f a_in, const vector2f b_in);
void vector3f_cross_product(vector3f v_out, const vector3f a_in, const vector3f b_in);
void normal_vector3f_between_vector3f(vector3f normal_vector, const vector3f a, const vector3f b);

float cosine_between_vector2f(const vector2f a,const vector2f b);
float cosine_between_vector3f(const vector3f a,const vector3f b);

float sine_between_vector2f(const vector2f a, const vector2f b);
float sine_between_vector3f(const vector3f a, const vector3f b);

float angle_between_vector2f(const vector2f a, const vector2f b);
float angle_between_vector3f(const vector3f a, const vector3f b);



#endif /*__MATH_VECTOR_H */


/************************ (C) COPYRIGHT DJI ********************END OF FILE****/

