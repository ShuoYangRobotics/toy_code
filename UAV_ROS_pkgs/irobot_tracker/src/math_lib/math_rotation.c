/**
  ******************************************************************************
  * @file    math_rotation.c
  * @author  Leonardo Yu Yun
  * @version V1.1.0
  * @date    20-March-2014
  * @brief   This file provides firmware functions to manage the following 
  *          functionalities of rotation operation:
  *           + basic operation: zero, set value, copy, add, sub, scale, norm
  *           + advanced operation: dot, cross, sine, cosine, angle between vectors
  * @note	 test case result of each func has been stated in remark section 
  *     
  @verbatim
 ===============================================================================
                     ##### convention used in this lib #####
 ===============================================================================
  yaw  -> y
  pitch-> p
  roll -> r

          | q0^2 + q1^2 - q2^2 - q3^2   2(q1q2 - q0q3)                      2(q1q3 + q0q2)                    |
    DCM = | 2(q1q2 + q0q3)              q0^2 - q1^2 + q2^2 - q3^2           2(q2q3 - q0q1)                    |
          | 2(q1q3 - q0q2)              2(q2q3 + q0q1)                      q0^2 - q1^2 - q2^2 + q3^2         |

          | cos(p)cos(y)                sin(r)sin(p)cos(y) - cos(r)sin(y)   cos(r)sin(p)cos(y) + sin(r)sin(y) |
    DCM = | cos(p)sin(y)                sin(r)sin(p)sin(y) + cos(r)cos(y)   cos(r)sin(p)sin(y) - sin(r)cos(y) |
          | -sin(p)                     sin(r)cos(p)                        cos(r)cos(p)                      |

  @endverbatim
  ******************************************************************************  
  */ 


/* Includes ------------------------------------------------------------------*/
#include "math_rotation.h"
#include "math_quaternion.h"
#include "math_matrix.h"
#include "math_vector.h"



/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/ 
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


/**
  * @brief			het operator for vector
  * @details		convert a vector to skew-symmetric
  *					equivalent to cross product
  *								|  0  -a3   a2 |
  *						a_hat = | a3    0  -a1 |
  *								|-a2   a1    0 |
  * @author			Yu Yun
  * @param[out]		m: output matrix
  * @param[in]		a: input vector
  * @retval			None 
  * @remark			benchmarked by: Yu Yun
  */
void hat_operator(matrix3f m, vector3f a)
{
	matrix3f_set_value( m, 0.0f, -a[2],  a[1],  
						   a[2],  0.0f, -a[0],
						  -a[1],  a[0],  0.0f);

}


/**
  * @brief			convert eular angle to DCM
  * @author			Yu Yun
  * @param[out]		DCM: output DCM
  * @param[in]		yaw:   input eular
  * @param[in]		pitch: input eular
  * @param[in]		roll:  input eular
  * @retval			None 
  * @remark			benchmarked by: Yu Yun
  */
void euler_to_DCM(matrix3f DCM, float yaw, float pitch, float roll)
{
	float cos_roll, cos_pitch, cos_yaw;
	float sin_roll, sin_pitch, sin_yaw;
	 
	cos_roll  = (float)cos(roll);
	cos_pitch = (float)cos(pitch);
	cos_yaw   = (float)cos(yaw);

	sin_roll  = (float)sin(roll);
	sin_pitch = (float)sin(pitch);
	sin_yaw   = (float)sin(yaw);

	/* 1st col */
	DCM[0][0] =  cos_pitch * cos_yaw;
	DCM[1][0] =  cos_pitch * sin_yaw;
	DCM[2][0] = -sin_pitch;

	/* 2nd col */
	DCM[0][1] = sin_roll * sin_pitch * cos_yaw - cos_roll * sin_yaw;
	DCM[1][1] = sin_roll * sin_pitch * sin_yaw + cos_roll * cos_yaw;
	DCM[2][1] = sin_roll * cos_pitch;

	/* 3rd col */
	DCM[0][2] = cos_roll * sin_pitch * cos_yaw + sin_roll * sin_yaw;
	DCM[1][2] = cos_roll * sin_pitch * sin_yaw - sin_roll * cos_yaw;
	DCM[2][2] = cos_roll * cos_pitch;

}


/**
  * @brief			convert eular angle to quaternion
  * @author			Yu Yun
  * @param[out]		q: output quaternion
  * @param[in]		yaw:   input eular
  * @param[in]		pitch: input eular
  * @param[in]		roll:  input eular
  * @retval			None 
  * @remark			benchmarked by: Yu Yun
  */
void euler_to_quat(vector4f q, float yaw, float pitch, float roll)
{
	float cos_half_yaw	 = (float)cos(yaw   * 0.5f);
	float sin_half_yaw   = (float)sin(yaw   * 0.5f);
	float cos_half_pitch = (float)cos(pitch * 0.5f);
	float sin_half_pitch = (float)sin(pitch * 0.5f);
	float cos_half_roll  = (float)cos(roll  * 0.5f);
	float sin_half_roll  = (float)sin(roll  * 0.5f);

	q[0] = cos_half_roll * cos_half_pitch * cos_half_yaw + sin_half_roll * sin_half_pitch * sin_half_yaw;
	q[1] = sin_half_roll * cos_half_pitch * cos_half_yaw - cos_half_roll * sin_half_pitch * sin_half_yaw;
	q[2] = cos_half_roll * sin_half_pitch * cos_half_yaw + sin_half_roll * cos_half_pitch * sin_half_yaw;
	q[3] = cos_half_roll * cos_half_pitch * sin_half_yaw - sin_half_roll * sin_half_pitch * cos_half_yaw;

}


/**
  * @brief			convert quaternion to DCM
  * @author			Yu Yun
  * @param[out]		DCM: output DCM
  * @param[in]		q:   input quaternion
  * @retval			None 
  * @remark			benchmarked by: Yu Yun
  */
void quat_to_DCM(matrix3f DCM, const vector4f q)
{
	/* each combination has been used twice, pre-process first */
	float q0q1 = q[0] * q[1];
	float q0q2 = q[0] * q[2]; 
	float q0q3 = q[0] * q[3]; 
	float q1q1 = q[1] * q[1];
	float q1q2 = q[1] * q[2];
	float q1q3 = q[1] * q[3];
	float q2q2 = q[2] * q[2];
	float q2q3 = q[2] * q[3];
	float q3q3 = q[3] * q[3];

	/* 1st col */
	DCM[0][0] = 1.0f - 2.0f * (q2q2 + q3q3);
	DCM[1][0] =        2.0f * (q1q2 + q0q3);
	DCM[2][0] =        2.0f * (q1q3 - q0q2);

	/* 2nd col */
	DCM[0][1] =        2.0f * (q1q2 - q0q3);
	DCM[1][1] = 1.0f - 2.0f * (q1q1 + q3q3); 
	DCM[2][1] =        2.0f * (q2q3 + q0q1);

	/* 3rd col */
	DCM[0][2] =        2.0f * (q1q3 + q0q2); 
	DCM[1][2] =        2.0f * (q2q3 - q0q1); 
	DCM[2][2] = 1.0f - 2.0f * (q1q1 + q2q2);

}


/**
  * @brief			convert quaternion to DCM col_1
  * @author			Yu Yun
  * @param[out]		DCM: output DCM col_1
  * @param[in]		q:   input quaternion
  * @retval			None 
  * @remark			benchmarked by: Yu Yun
  */
void quat_to_DCM_col_1(vector3f DCM, const vector4f q)
{
	/* 1st col */
	DCM[0] = 1.0f - 2.0f * (q[2]*q[2] + q[3]*q[3]);
	DCM[1] =        2.0f * (q[1]*q[2] + q[0]*q[3]);
	DCM[2] =        2.0f * (q[1]*q[3] - q[0]*q[2]);
}


/**
  * @brief			convert quaternion to DCM col_2
  * @author			Yu Yun
  * @param[out]		DCM: output DCM col_2
  * @param[in]		q:   input quaternion
  * @retval			None 
  * @remark			benchmarked by: Yu Yun
  */
void quat_to_DCM_col_2(vector3f DCM, const vector4f q)
{
	/* 2nd col */
	DCM[0] =        2.0f * (q[1]*q[2] - q[0]*q[3]);
	DCM[1] = 1.0f - 2.0f * (q[1]*q[1] + q[3]*q[3]); 
	DCM[2] =        2.0f * (q[2]*q[3] + q[0]*q[1]);
}


/**
  * @brief			convert quaternion to DCM col_3
  * @author			Yu Yun
  * @param[out]		DCM: output DCM col_3
  * @param[in]		q:   input quaternion
  * @retval			None 
  * @remark			benchmarked by: Yu Yun 
  */
void quat_to_DCM_col_3(vector3f DCM, const vector4f q)
{
	/* 3rd col */
	DCM[0] =        2.0f * (q[1]*q[3] + q[0]*q[2]); 
	DCM[1] =        2.0f * (q[2]*q[3] - q[0]*q[1]); 
	DCM[2] = 1.0f - 2.0f * (q[1]*q[1] + q[2]*q[2]);
}


/**
  * @brief			convert quaternion to euler angle
  * @author			Yu Yun
  * @param[out]		*yaw:   output euler angle
  * @param[out]		*pitch: output euler angle
  * @param[out]		*roll:  output euler angle
  * @param[in]		q:   input quaternion
  * @retval			None 
  * @remark			benchmarked by: Yu Yun 
  *					 + improve the accuracy by atan2
  */
void quat_to_eular(float *yaw, float *pitch, float *roll, const vector4f q)
{
	float sine_pitth = 0.0f;

	float cos_pitch_cos_yaw  = 1.0f - 2.0f * (q[2]*q[2] + q[3]*q[3]);   // +DCM[0][0]
	float cos_pitch_sin_yaw  =      + 2.0f * (q[1]*q[2] + q[0]*q[3]);   // +DCM[1][0]
	float sin_pitch	         =      - 2.0f * (q[1]*q[3] - q[0]*q[2]);   // -DCM[2][0]
	float cos_pitch			 = 0.0f;
	float sin_roll_cos_pitch =      + 2.0f * (q[2]*q[3] + q[0]*q[1]);   // +DCM[2][1]
	float cos_roll_cos_pitch = 1.0f - 2.0f * (q[1]*q[1] + q[2]*q[2]);   // +DCM[2][2]


	cos_pitch = (float)sqrt(cos_pitch_cos_yaw*cos_pitch_cos_yaw + cos_pitch_sin_yaw*cos_pitch_sin_yaw);

	*yaw   = (float)atan2(cos_pitch_sin_yaw, cos_pitch_cos_yaw);
	*pitch = (float)atan2(sin_pitch, cos_pitch);
	*roll  = (float)atan2(sin_roll_cos_pitch, cos_roll_cos_pitch);

}




/**
  * @brief			convert DCM to quaternion 
  * @details        to avoid division by zero and to improve 
  *					the numerical accuracy
  *					  + check the trace(sum of leading diagonal terms)
  *					  + if the trace is less or equal than zero, identify which
  *				        major diagonal element has the greatest value
  *					  + calculate the according quaternion element with the 
						greatest diagonal element
  * @author			Yu Yun
  * @param[out]		q:   output quaternion
  * @param[in]		DCM: input DCM
  * @retval			None 
  * @remark			benchmarked by: Yu Yun
  */
void DCM_to_quat(vector4f q, matrix3f DCM)
{
	float trace = DCM[0][0] + DCM[1][1] + DCM[2][2];
	float S = 0.0f;

	if (trace > 0)
	{
		S = (float)sqrt(trace + 1.0f) * 2.0f;								//S = 4*q0
		q[0] = 0.25f * S;
		q[1] = (DCM[2][1] - DCM[1][2]) / S;
		q[2] = (DCM[0][2] - DCM[2][0]) / S;
		q[3] = (DCM[1][0] - DCM[0][1]) / S;
	}
	else if ((DCM[0][0] > DCM[1][1]) && (DCM[0][0] > DCM[2][2]))
	{
		S = (float)sqrt(1.0f + DCM[0][0] - DCM[1][1] - DCM[2][2]) * 2.0f;	//S = 4*q1
		q[0] = (DCM[2][1] - DCM[1][2]) / S;
		q[1] = 0.25f * S;
		q[2] = (DCM[0][1] + DCM[1][0]) / S;
		q[3] = (DCM[0][2] + DCM[2][0]) / S;
	}
	else if (DCM[1][1] > DCM[2][2])
	{
		S = (float)sqrt(1.0f + DCM[1][1] - DCM[0][0] - DCM[2][2]) * 2.0f;	//S = 4*q2
		q[0] = (DCM[0][2] - DCM[2][0]) / S;
		q[1] = (DCM[0][1] + DCM[1][0]) / S;
		q[2] = 0.25f * S;
		q[3] = (DCM[1][2] + DCM[2][1]) / S;
	}
	else
	{
		S = (float)sqrt(1.0f + DCM[2][2] - DCM[0][0] - DCM[1][1]) * 2.0f;	//S = 4*q3
		q[0] = (DCM[1][0] - DCM[0][1]) / S;
		q[1] = (DCM[0][2] + DCM[2][0]) / S;
		q[2] = (DCM[1][2] + DCM[2][1]) / S;
		q[3] = 0.25f * S;
	}

}


/**
  * @brief			convert DCM to euler angle
  * @author			Yu Yun
  * @param[out]		*yaw:   output euler angle
  * @param[out]		*pitch: output euler angle
  * @param[out]		*roll:  output euler angle
  * @param[in]		DCM:   input DCM
  * @retval			None 
  * @remark			benchmarked by: Yu Yun 
  *					 + improve the accuracy by atan2
  */
void DCM_to_euler(float *yaw, float *pitch, float *roll, matrix3f DCM)
{
	float cos_pitch = (float)sqrt(DCM[0][0]*DCM[0][0] + DCM[1][0] * DCM[1][0]);

	*yaw   = (float)atan2(DCM[1][0], DCM[0][0]);
	*pitch = (float)atan2(-DCM[2][0], cos_pitch);
	*roll  = (float)atan2(DCM[2][1], DCM[2][2]);

}


/**
  * @brief			convert SORA to quaternion
  * @author			Yu Yun
  * @param[out]		q: output quaternion
  * @param[in]		theta_x: input angle
  * @param[in]		theta_y: input angle
  * @param[in]		theta_z: input angle
  * @retval			None 
  * @remark			benchmarked by: Yu Yun
  */
void SORA_to_quat(vector4f q, float theta_x, float theta_y, float theta_z)
{
	float theta = (float)sqrt(theta_x * theta_x + theta_y * theta_y + theta_z * theta_z);
	float half_theta;
	float cos_half_theta;
	float sin_half_theta;
	float inv_norm;

	if(theta == 0.0f)
	{
		q[0] = 1.0f;
		q[1] = 0.0f;
		q[2] = 0.0f;
		q[3] = 0.0f;
	}
	else
	{
		half_theta = theta * 0.5f;
		cos_half_theta = (float)cos(half_theta);
		sin_half_theta = (float)sin(half_theta);

		inv_norm = sin_half_theta / theta;

		q[0] = cos_half_theta;
		q[1] = theta_x * inv_norm;
		q[2] = theta_y * inv_norm;
		q[3] = theta_z * inv_norm;
	}
}


/**
  * @brief			convert quaternion to SORA
  * @author			Yu Yun
  * @param[out]		*theta_x: output angle
  * @param[out]		*theta_y: output angle
  * @param[out]		*theta_z: output angle
  * @param[in]		q: input quaternion
  * @retval			None 
  * @remark			benchmarked by: Yu Yun
  *					 + use atan2 to improve the accuracy
  */
void quat_to_SORA(float * theta_x, float * theta_y, float * theta_z, const vector4f q)
{
	float cos_half_theta, sin_half_theta;
	float half_theta, theta;

	cos_half_theta = q[0];
	sin_half_theta  = (float)sqrt(q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);

	half_theta = (float)atan2(sin_half_theta, cos_half_theta);
	theta = half_theta * 2.0f;

	if(sin_half_theta == 0.0f)
	{
		*theta_x = 0.0f;
		*theta_y = 0.0f;
		*theta_z = 0.0f;
	}
	else
	{
		*theta_x = theta * q[1]/sin_half_theta;
		*theta_y = theta * q[2]/sin_half_theta;
		*theta_z = theta * q[3]/sin_half_theta;
	}

}


/**
  * @brief			convert axis_angle to quaternion
  * @author			Yu Yun
  * @param[out]		q: output quaternion
  * @param[in]		axis: input rotation axis
  * @param[in]		angle: input rotation angle
  * @retval			None 
  * @remark			benchmarked by: Yu Yun
  */
void axis_angle_to_quat(vector4f q, vector3f axis, float angle)
{
	float cos_half_theta = (float)cos(angle * 0.5f);
	float sin_half_theta = (float)sin(angle * 0.5f);

	q[0] = cos_half_theta;
	q[1] = sin_half_theta * axis[0];
	q[2] = sin_half_theta * axis[1];
	q[3] = sin_half_theta * axis[2];

}


/**
  * @brief			convert axis_angle to quaternion
  * @details        Rodrigues' formula
  *
  *					DCM = I + omega_hat*sin(theta) + omega_hat^2*(1-cos(theta))
  *								|  0  -a3   a2 |
  *						a_hat = | a3    0  -a1 |
  *								|-a2   a1    0 |
  *					detailed derivation can be done by series expansion
  * @author			Yu Yun
  * @param[out]		DCM: output DCM
  * @param[in]		axis: input rotation axis
  * @param[in]		angle: input rotation angle
  * @retval			None 
  * @remark			benchmarked by: Yu Yun
  */
void axis_angle_to_DCM(matrix3f DCM, vector3f axis, float angle)
{
	float cos_theta = (float)cos(angle);
	float sin_theta = (float)sin(angle);

	matrix3f I;
	matrix3f omega_hat;
	matrix3f omega_hat_square;



	matrix3f omega_hat_sin;
	matrix3f omega_hat_sqr_1_cos;

	matrix3f_eye(I);
	hat_operator(omega_hat, axis);
	matrix3f_multi_matrix3f(omega_hat_square, omega_hat, omega_hat);

	matrix3f_multi_scaler(omega_hat_sin, omega_hat, sin_theta);
	matrix3f_multi_scaler(omega_hat_sqr_1_cos, omega_hat_square, (1-cos_theta));

	matrix3f_add3(DCM, I, omega_hat_sin, omega_hat_sqr_1_cos);

}





/**
  * @brief			get tilt quaternion from given quaternion
  * @author			Yu Yun
  * @param[out]		q_tilt: output q_tilt quaternion
  * @param[in]		q:		input quaternion 
  * @retval			None 
  * @remark			benchmarked by: Yu Yun
  *					 + the code is resilient to Identity input
  */
void get_tilt_quat_n_angles(vector4f q_tilt, float *theta_x, float *theta_y, const vector4f q)
{
	const vector3f ref_norm_vector = {0.0f, 0.0f, 1.0f};
	vector3f body_norm_vector;
	vector3f rotational_axis; 

	float inv_norm;

	float theta = 0.0f;

	quat_to_DCM_col_3(body_norm_vector, q);

	normal_vector3f_between_vector3f(rotational_axis, ref_norm_vector, body_norm_vector);

	theta = angle_between_vector3f(ref_norm_vector, body_norm_vector);

	*theta_x = theta * rotational_axis[0];
	*theta_y = theta * rotational_axis[1];

	axis_angle_to_quat(q_tilt, rotational_axis, theta);

}

/**
  * @brief			get tilt quaternion from given quaternion
  * @author			Yu Yun
  * @param[out]		q_tilt: output q_tilt quaternion
  * @param[in]		q:		input quaternion 
  * @retval			None 
  * @remark			benchmarked by: Yu Yun
  *					 + the code is resilient to Identity input
  */
void get_tilt_quat(vector4f q_tilt, const vector4f q)
{
	float null_x, null_y;

	get_tilt_quat_n_angles(q_tilt, &null_x, &null_y,  q);

}


/**
  * @brief			decompose the given quaternion to tilt quaternion & torsion quaternion
  * @detail			tilt first, which is the rotation of xy plane;
  *					torsion second, which is the rotation of z axis  
  * @author			Yu Yun
  * @param[out]		q_tilt:	   output q_tilt quaternion
  * @param[out]		q_torsion: output q_torsion quaternion
  * @param[in]		q:		   input quaternion 
  * @retval			None 
  * @remark			benchmarked by: Yu Yun
  *					 + the code is resilient to Identity input
  */
void quat_decompose_tilt_torsion(vector4f q_tilt, vector4f q_torsion, const vector4f q)
{
	get_tilt_quat(q_tilt, q);

	get_error_quat(q_torsion, q_tilt, q);
}


/**
  * @brief			decompose the given quaternion to tilt quaternion & torsion quaternion
  * @detail			tilt first, which is the rotation of xy plane;
  *					torsion second, which is the rotation of z axis  
  * @author			Yu Yun
  * @param[out]		q_tilt:	   output q_tilt quaternion
  * @param[out]		q_torsion: output q_torsion quaternion
  * @param[in]		q:		   input quaternion 
  * @retval			None 
  * @remark			benchmarked by: Yu Yun
  *					 + the code is resilient to Identity input
  */
void quat_decompose_tilt_torsion_n_angles(vector4f q_tilt, vector4f q_torsion, 
										float * theta_x, float * theta_y, float * theta_z, 
										const vector4f q)
{
	get_tilt_quat(q_tilt, q);
	get_tilt_quat_n_angles(q_tilt, theta_x, theta_y,  q);

	get_error_quat(q_torsion, q_tilt, q);
	*theta_z = get_yaw_angle_from_quat(q_torsion);	
}


/**
  * @brief			decompose the given quaternion to torsion quaternion & tilt quaternion 
  * @details		torsion first, which is the rotation of z axis  
  *					tilt second, which is the rotation of xy plane;	
  * @author			Yu Yun
  * @param[out]		q_torsion: output q_torsion quaternion
  * @param[out]		q_tilt:	   output q_tilt quaternion
  * @param[in]		q:		   input quaternion 
  * @retval			None 
  * @remark			benchmarked by: Yu Yun
  *					 + the code is resilient to Identity input
  */
void quat_decompose_torsion_tilt(vector4f q_torsion, vector4f q_tilt, const vector4f q)
{
	vector4f q_conju;

	get_quat_conjugate(q_conju, q);

	quat_decompose_tilt_torsion(q_tilt, q_torsion, q_conju);

	take_quat_conjugate(q_tilt);
	take_quat_conjugate(q_torsion);

}


/**
  * @brief			get the level quaternion(from ground to body), which is the torsion motion
  * @author			Yu Yun
  * @param[out]		level_q:   output level quaternion
  * @param[in]		q:		   input quaternion 
  * @retval			None 
  * @remark			benchmarked by: Yu Yun
  *					 + the code is resilient to Identity input
  */
void get_level_quat(vector4f level_q, vector4f q)
{
	vector4f q_null;

	quat_decompose_torsion_tilt(level_q, q_null, q);

}


/**
  * @brief			convert vector from ground frame to body frame
  * @author			Yu Yun
  * @param[out]		output: output vector defined on body frame
  * @param[in]		input:	input vector defined on ground frame
  * @param[in]		DCM:	input matrix, level DCM defined on groung frame
  * @retval			None 
  * @remark			benchmarked by: Yu Yun
  *					 + the code is resilient to Identity input
  */
void level_map_ground_to_body(float output[3], float input[3], matrix3f DCM)
{
	matrix3f DCM_T;
	vector3f IN;
	vector3f OUT;

	IN[0] = input[0];
	IN[1] = input[1];
	IN[2] = 0.0f;

	get_matrix3f_transpose(DCM_T, DCM);
	matrix3f_multi_vector3f(OUT, DCM_T, IN);

	output[0] = OUT[0];
	output[1] = OUT[1];
	output[2] = input[2];

}


/**
  * @brief			convert vector from body frame to ground frame
  * @author			Yu Yun
  * @param[out]		output: output vector defined on ground frame
  * @param[in]		input:	input vector defined on body frame
  * @param[in]		DCM:	input matrix, level DCM defined on groung frame
  * @retval			None 
  * @remark			benchmarked by: Yu Yun
  *					 + the code is resilient to Identity input
  */
void level_map_body_to_ground(float output[3], float input[3], matrix3f DCM)
{
	vector3f IN;
	vector3f OUT;

	IN[0] = input[0];
	IN[1] = input[1];
	IN[2] = 0.0f;

	matrix3f_multi_vector3f(OUT, DCM, IN);

	output[0] = OUT[0];
	output[1] = OUT[1];
	output[2] = input[2];
}


/**
  * @brief			convert quaternion to yaw angle
  * @author			Yu Yun
  * @param[in]		q:   input quaternion
  * @retval			yaw: output yaw angle
  * @remark			benchmarked by: Yu Yun 
  */
float get_yaw_angle_from_quat(const vector4f q)
{
	float yaw = 0.0f;
	float sine_pitth = 0.0f;

	float cos_pitch_cos_yaw  = 1.0f - 2.0f * (q[2]*q[2] + q[3]*q[3]);   // +DCM[0][0]
	float cos_pitch_sin_yaw  =      + 2.0f * (q[1]*q[2] + q[0]*q[3]);   // +DCM[1][0]
	
	yaw = (float)atan2(cos_pitch_sin_yaw, cos_pitch_cos_yaw);

	return yaw;
}


/**
  * @brief			convert quaternion to yaw angle
  * @author			Yu Yun
  * @param[in]		q:   input quaternion
  * @retval			yaw: output yaw angle
  * @remark			benchmarked by: Yu Yun 
  */
float get_yaw_angle_from_DCM(const matrix3f DCM)
{
	float yaw  = (float)atan2(DCM[1][0], DCM[0][0]);

	return yaw;
}


/**
  * @brief			get tilt cosine from quaternion
  * @author			Yu Yun
  * @param[in]		q:   input quaternion
  * @retval			cos: output cosine
  * @remark			benchmarked by: Yu Yun 
  */
float get_tilt_cosine_from_quat(const vector4f q)
{
	float cos = 1.0f - 2.0f * q[1]*q[1] - 2.0f * q[2]*q[2];

	return cos;
}

/**
  * @brief			get tilt cosine from DCM
  * @author			Yu Yun
  * @param[in]		DCM:   input DCM
  * @retval			cos: output cosine
  * @remark			benchmarked by: Yu Yun 
  */
float get_tilt_cosine_from_DCM(const matrix3f DCM)
{
	float cos = DCM[2][2];

	return cos;
}


/**
  * @brief			get tilt angle from quaternion
  * @author			Yu Yun
  * @param[in]		q:   input quaternion
  * @retval			angle: output tilt angle
  * @remark			benchmarked by: Yu Yun 
  */
float get_tilt_angle_from_quat(const vector4f q)
{
	float angle = 0.0f;
	const vector3f ref_norm_vector = {0.0f, 0.0f, 1.0f};
	vector3f body_norm_vector;

	quat_to_DCM_col_3(body_norm_vector, q);

	angle = angle_between_vector3f(ref_norm_vector, body_norm_vector);

	return angle;
}


/**
  * @brief			get tilt angle from quaternion
  * @author			Yu Yun
  * @param[in]		DCM:   input DCM
  * @retval			angle: output tilt angle
  * @remark			benchmarked by: Yu Yun 
  */
float get_tilt_angle_from_DCM(const matrix3f DCM)
{
	float angle = 0.0f;
	const vector3f ref_norm_vector = {0.0f, 0.0f, 1.0f};
	vector3f body_norm_vector;


	body_norm_vector[0] = DCM[2][0];
	body_norm_vector[1] = DCM[2][1];
	body_norm_vector[2] = DCM[2][2];

	angle = angle_between_vector3f(ref_norm_vector, body_norm_vector);

	return angle;
}


/**
  * @brief			quat update by given angular velocity and delta_t
  * @author			Yu Yun
  * @param[out]		q_out: updated quat
  * @param[in]		omega: current angular velocity
  * @param[in]		delta_t: 
  * @retval			angle: output tilt angle
  * @remark			benchmarked by: Yu Yun 
  */
void quat_update(vector4f q_out, const vector3f omega, const float delta_t, const vector4f q_in)
{
	vector3f theta;
	vector4f q_delta;

	vector3f_multi_scaler(theta, omega, delta_t);

	SORA_to_quat(q_delta, theta[0], theta[1], theta[2]);

	unit_quat_multi(q_out, q_in, q_delta);

}



/************************ (C) COPYRIGHT DJI ********************END OF FILE****/




