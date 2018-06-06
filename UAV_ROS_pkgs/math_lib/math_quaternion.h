/**
  ******************************************************************************
  * @file    math_quaternion.h
  * @author  Leonardo Yu Yun
  * @version V1.1.0
  * @date    20-March-2014
  * @brief   This file contains all the functions prototypes for 
  *			 quaternion operations
  *         
  ******************************************************************************  
  */ 


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MATH_QUATERNION_H
#define __MATH_QUATERNION_H


/* Includes ------------------------------------------------------------------*/
#include "math_basic.h"


/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/  



void quat_zero(vector4f q);
void quat_set_value(vector4f q, float q0, float q1, float q2, float q3);
void quat_copy(vector4f dest, vector4f src);
void quat_eye(vector4f q);

void get_quat_conjugate(vector4f q_conju, const vector4f q);
void take_quat_conjugate(vector4f q);
void quat_normalize(vector4f q);

void quat_multi(vector4f q_out, const vector4f a, const vector4f b);
void unit_quat_multi(vector4f q_out, const vector4f a, const vector4f b);
void get_error_quat(vector4f q_err, const vector4f q_cur, const vector4f q_tgt);

void quat_rot_vector(vector3f v_out, const vector3f v_in, const vector4f q);


#endif /*__MATH_QUATERNION_H */


/************************ (C) COPYRIGHT DJI ********************END OF FILE****/
