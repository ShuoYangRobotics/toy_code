/**
  ******************************************************************************
  * @file    math_quaternion.c
  * @author  Leonardo Yu Yun
  * @version V1.1.0
  * @date    20-March-2014
  * @brief   This file provides firmware functions to manage the following 
  *          functionalities of quaternion operation:
  *           + basic operation: zero, set value, copy, add, sub, scale, norm
  *           + advanced operation: dot, cross, sine, cosine, angle between vectors
  * @note	 test case result of each func has been stated in remark section 
  *           
  ******************************************************************************  
  */ 


/* Includes ------------------------------------------------------------------*/
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
  * @brief			set quaternion elements to zero
  * @author			Yu Yun
  * @param[in,out]	q: input&output quaternion
  * @retval			None 
  * @remark			benchmarked by: Yu Yun
  */
void quat_zero(vector4f q)
{
	vector4f_set_value(q, 0.0f, 0.0f, 0.0f, 0.0f);
}


/**
  * @brief			set quaternion elements to user defined values
  * @author			Yu Yun
  * @param[in,out]	q: input&output quaternion
  * @param[in]		q0: input value for element1
  * @param[in]		q1: input value for element2
  * @retval			None 
  * @remark			benchmarked by: Yu Yun
  */
void quat_set_value(vector4f q, float q0, float q1, float q2, float q3)
{
	vector4f_set_value(q, q0, q1, q2, q3);
}


/**
  * @brief			copy the value in quaternion src to quaternion dest
  * @author			Yu Yun
  * @param[out]		dest: output quaternion
  * @param[in]		src: input quaternion
  * @retval			None 
  * @remark			benchmarked by: Yu Yun
  */
void quat_copy(vector4f dest, vector4f src)
{
	vector4f_copy(dest, src);
}


/**
  * @brief			set identity quaternion 
  * @author			Yu Yun
  * @param[in,out]	q: input&output quaternion
  * @retval			None 
  * @remark			benchmarked by: Yu Yun
  */
void quat_eye(vector4f q)
{
	vector4f_set_value(q, 1.0f, 0.0f, 0.0f, 0.0f);
}


/**
  * @brief			get the conjugate of given quaternion
  * @author			Yu Yun
  * @param[out]		q_conju: output quaternion(conjugate)
  * @param[in]		q: input quaternion
  * @retval			None 
  * @remark			benchmarked by: Yu Yun
  */
void get_quat_conjugate(vector4f q_conju, const vector4f q)
{
	q_conju[0] = + q[0];
	q_conju[1] = - q[1];
	q_conju[2] = - q[2];
	q_conju[3] = - q[3];
}


/**
  * @brief			take the conjugate of given quaternion
  * @author			Yu Yun
  * @param[in,out]	q: input&output quaternion	
  * @retval			None 
  * @remark			benchmarked by: Yu Yun
  */
void take_quat_conjugate(vector4f q)
{
	q[1] = - q[1];
	q[2] = - q[2];
	q[3] = - q[3];
}


/**
  * @brief			normalize a quaternion
  * @author			Yu Yun
  * @param[in]		q: input quaternion
  * @retval			None 
  * @remark			benchmarked by: Yu Yun
  *					 + the code is resilient to zero input
  */
void quat_normalize(vector4f q)
{
	float inv_norm = vector4f_inv_norm(q);

	vector4f_scale(q, inv_norm);
}


/**
  * @brief			quaternion mutiplication
  * @author			Yu Yun
  * @param[out]		q_out: output quaternion
  * @param[in]		a_in: input quaternion a
  * @param[in]		b_in: input quaternion b
  * @retval			None 
  * @remark			benchmarked by: Yu Yun
  *					multi can be used for non unit quaternion(vector)
  */
void quat_multi(vector4f q_out, const vector4f a, const vector4f b)
{
	q_out[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
	q_out[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
	q_out[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
	q_out[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
	
}


/**
  * @brief			give the normalized quaternion muti result
  * @author			Yu Yun
  * @param[out]		q_out: output quaternion
  * @param[in]		a_in: input quaternion a
  * @param[in]		b_in: input quaternion b
  * @retval			None 
  * @remark			benchmarked by: Yu Yun
  *					result of muliplication has been normalized
  */
void unit_quat_multi(vector4f q_out, const vector4f a, const vector4f b)
{
	quat_multi(q_out, a, b);

	if(q_out[0] < 0.0f)
	{
		q_out[0] = -q_out[0];
		q_out[1] = -q_out[1];
		q_out[2] = -q_out[2];
		q_out[3] = -q_out[3];
	}

	quat_normalize(q_out);
}


/**
  * @brief			get error quaternion between two quaternions
  * @author			Yu Yun
  * @param[out]		q_err: output error quaternion
  * @param[in]		q_cur: input quaternion current
  * @param[in]		q_tgt: input quaternion target
  * @retval			None 
  * @remark			benchmarked by: Yu Yun
  */
void get_error_quat(vector4f q_err, const vector4f q_cur, const vector4f q_tgt)
{
	vector4f q_conju;

	get_quat_conjugate(q_conju, q_cur);

	unit_quat_multi(q_err, q_conju, q_tgt);

}

void quat_rot_vector(vector3f v_out, const vector3f v_in, const vector4f q)
{
	vector4f q_conju;
	vector4f V_IN;
	vector4f V_OUT;
	vector4f v_temp;

	V_IN[0] = 0.0f;
	V_IN[1] = v_in[0];
	V_IN[2] = v_in[1];
	V_IN[3] = v_in[2];

	get_quat_conjugate(q_conju, q);

	quat_multi(v_temp, q, V_IN);
	quat_multi(V_OUT, v_temp, q_conju);

	v_out[0] = V_OUT[1];
	v_out[1] = V_OUT[2];
	v_out[2] = V_OUT[3];

}



/************************ (C) COPYRIGHT DJI ********************END OF FILE****/



 
  

