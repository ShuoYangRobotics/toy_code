/**
  ******************************************************************************
  * @file    math_rotation.h
  * @author  Leonardo Yu Yun
  * @version V1.1.0
  * @date    20-March-2014
  * @brief   This file contains all the functions prototypes for 
  *			 rotation operations
  *         
  ******************************************************************************  
  */ 


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MATH_ROTATION_H
#define __MATH_ROTATION_H


/* Includes ------------------------------------------------------------------*/
#include "math_basic.h"


/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/  


void hat_operator(matrix3f m, vector3f a);

/* conversion between different representations */
void euler_to_DCM(matrix3f DCM, float yaw, float pitch, float roll);
void euler_to_quat(vector4f q, float yaw, float pitch, float roll);

void quat_to_DCM(matrix3f DCM, const vector4f q);
void quat_to_DCM_col_1(vector3f DCM, const vector4f q);
void quat_to_DCM_col_2(vector3f DCM, const vector4f q);
void quat_to_DCM_col_3(vector3f DCM, const vector4f q);
void quat_to_eular(float *yaw, float *pitch, float *roll, const vector4f q);

void DCM_to_quat(vector4f q, matrix3f DCM);
void DCM_to_euler(float *yaw, float *pitch, float *roll, matrix3f DCM);

void SORA_to_quat(vector4f q, float theta_x, float theta_y, float theta_z);
void quat_to_SORA(float * theta_x, float * theta_y, float * theta_z, const vector4f q);

void axis_angle_to_quat(vector4f q, vector3f axis, float angle);
void axis_angle_to_DCM(matrix3f DCM, vector3f axis, float angle);

/* quaternion decomposition */
void get_tilt_quat_n_angles(vector4f q_tilt, float *theta_x, float *theta_y, const vector4f q);
void get_tilt_quat(vector4f q_tilt, const vector4f q);

void quat_decompose_tilt_torsion(vector4f q_tilt, vector4f q_torsion, const vector4f q);
void quat_decompose_tilt_torsion_n_angles(vector4f q_tilt, vector4f q_torsion, 
										float * theta_x, float * theta_y, float * theta_z, 
										const vector4f q);

void quat_decompose_torsion_tilt(vector4f q_torsion, vector4f q_tilt, const vector4f q);

/* coordinate mapping function */
void get_level_quat(vector4f level_q, vector4f q);
void level_map_ground_to_body(float output[3], float input[3], matrix3f DCM);
void level_map_body_to_ground(float output[3], float input[3], matrix3f DCM);

/* auxiliary function */
float get_yaw_angle_from_quat(const vector4f q);
float get_yaw_angle_from_DCM(const matrix3f DCM);

float get_tilt_cosine_from_quat(const vector4f q);
float get_tilt_cosine_from_DCM(const matrix3f DCM);

float get_tilt_angle_from_quat(const vector4f q);
float get_tilt_angle_from_DCM(const matrix3f DCM);


#endif /*__MATH_ROTATION_H */


/************************ (C) COPYRIGHT DJI ********************END OF FILE****/

