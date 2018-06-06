/**
  ******************************************************************************
  * @file    math_vector.c
  * @author  Leonardo Yu Yun
  * @version V1.1.0
  * @date    20-March-2014
  * @brief   This file provides firmware functions to manage the following 
  *          functionalities of vector operation:
  *           + basic operation: zero, set value, copy, add, sub, scale, norm
  *           + advanced operation: dot, cross, sine, cosine, angle between vectors
  * @note	 test case result of each func has been stated in remark section 
  *           
  ******************************************************************************  
  */ 


/* Includes ------------------------------------------------------------------*/
#include "math_vector.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/ 
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


/**
  * @brief			set vector elements to zero
  * @author			Yu Yun
  * @param[in,out]	v: input&output vector
  * @retval			None 
  * @remark			benchmarked by: Yu Yun
  */
void vector2f_zero(vector2f v)
{
	v[0] = 0.0f;
	v[1] = 0.0f;
}


/**
  * @brief			set vector elements to zero
  * @author			Yu Yun
  * @param[in,out]	v: input&output vector
  * @retval			None 
  * @remark			benchmarked by: Yu Yun
  */
void vector3f_zero(vector3f v)
{
	v[0] = 0.0f;
	v[1] = 0.0f;
	v[2] = 0.0f;
}


/**
  * @brief			set vector elements to zero
  * @author			Yu Yun
  * @param[in,out]	v: input&output vector
  * @retval			None 
  * @remark			benchmarked by: Yu Yun
  */
void vector4f_zero(vector4f v)
{
	v[0] = 0.0f;
	v[1] = 0.0f;
	v[2] = 0.0f;
	v[3] = 0.0f;
}


/**
  * @brief			set vector elements to user defined values
  * @author			Yu Yun
  * @param[in,out]	v: input&output vector
  * @param[in]		v0: input value for element1
  * @param[in]		v1: input value for element2
  * @retval			None 
  * @remark			benchmarked by: Yu Yun
  */
void vector2f_set_value(vector2f v, float v0, float v1)
{
	v[0] = v0;
	v[1] = v1;
}


/**
  * @brief			set vector elements to user defined values
  * @author			Yu Yun
  * @param[in,out]	v: input&output vector
  * @param[in]		v0: input value for element1
  * @param[in]		v1: input value for element2
  * @param[in]		v2: input value for element3
  * @retval			None 
  * @remark			benchmarked by: Yu Yun
  */
void vector3f_set_value(vector3f v, float v0, float v1, float v2)
{
	v[0] = v0;
	v[1] = v1;
	v[2] = v2;
}


/**
  * @brief			set vector elements to user defined values
  * @author			Yu Yun
  * @param[in,out]	v: input&output vector
  * @param[in]		v0: input value for element1
  * @param[in]		v1: input value for element2
  * @param[in]		v2: input value for element3
  * @param[in]		v3: input value for element4
  * @retval			None 
  * @remark			benchmarked by: Yu Yun
  */
void vector4f_set_value(vector4f v, float v0, float v1, float v2, float v3)
{
	v[0] = v0;
	v[1] = v1;
	v[2] = v2;
	v[3] = v3;
}


/**
  * @brief			copy the value in vector src to vector dest
  * @author			Yu Yun
  * @param[out]		dest: output vector
  * @param[in]		src: input vector
  * @retval			None 
  * @remark			benchmarked by: Yu Yun
  */
void vector2f_copy(vector2f dest, vector2f src)
{
	dest[0] = src[0];
	dest[1] = src[1];
}


/**
  * @brief			copy the value in vector src to vector dest
  * @author			Yu Yun
  * @param[out]		dest: output vector
  * @param[in]		src: input vector
  * @retval			None 
  * @remark			benchmarked by: Yu Yun
  */
void vector3f_copy(vector3f dest, vector3f src)
{
	dest[0] = src[0];
	dest[1] = src[1];
	dest[2] = src[2];
}


/**
  * @brief			copy the value in vector src to vector dest
  * @author			Yu Yun
  * @param[out]		dest: output vector
  * @param[in]		src: input vector
  * @retval			None 
  * @remark			benchmarked by: Yu Yun
  */
void vector4f_copy(vector4f dest, vector4f src)
{
	dest[0] = src[0];
	dest[1] = src[1];
	dest[2] = src[2];
	dest[3] = src[3];
}


/**
  * @brief			vector addition
  * @author			Yu Yun
  * @param[out]		v_out: output vector
  * @param[in]		a_in: input vector a
  * @param[in]		b_in: input vector b
  * @retval			None 
  * @remark			benchmarked by: Yu Yun
  */
void vector2f_add(vector2f v_out, vector2f a_in,vector2f b_in)
{
	v_out[0] = a_in[0] + b_in[0];
	v_out[1] = a_in[1] + b_in[1];
}


/**
  * @brief			vector addition
  * @author			Yu Yun
  * @param[out]		v_out: output vector
  * @param[in]		a_in: input vector a
  * @param[in]		b_in: input vector b
  * @retval			None 
  * @remark			benchmarked by: Yu Yun
  */
void vector3f_add(vector3f v_out, vector3f a_in,vector3f b_in)
{
	v_out[0] = a_in[0] + b_in[0];
	v_out[1] = a_in[1] + b_in[1];
	v_out[2] = a_in[2] + b_in[2];
}


/**
  * @brief			vector addition
  * @author			Yu Yun
  * @param[out]		v_out: output vector
  * @param[in]		a_in: input vector a
  * @param[in]		b_in: input vector b
  * @retval			None 
  * @remark			benchmarked by: Yu Yun
  */
void vector3f_add3(vector3f v_out, vector3f a_in, vector3f b_in, vector3f c_in)
{
	v_out[0] = a_in[0] + b_in[0] + c_in[0];
	v_out[1] = a_in[1] + b_in[1] + c_in[1];
	v_out[2] = a_in[2] + b_in[2] + c_in[2];
}


/**
  * @brief			vector addition
  * @author			Yu Yun
  * @param[out]		v_out: output vector
  * @param[in]		a_in: input vector a
  * @param[in]		b_in: input vector b
  * @retval			None 
  * @remark			benchmarked by: Yu Yun
  */
void vector4f_add(vector4f v_out, vector4f a_in,vector4f b_in)
{
	v_out[0] = a_in[0] + b_in[0];
	v_out[1] = a_in[1] + b_in[1];
	v_out[2] = a_in[2] + b_in[2];
	v_out[3] = a_in[3] + b_in[3];
}


/**
  * @brief			vector subtraction
  * @author			Yu Yun
  * @param[out]		v_out: output vector
  * @param[in]		a_in: input vector a
  * @param[in]		b_in: input vector b
  * @retval			None 
  * @remark			benchmarked by: Yu Yun
  */
void vector2f_sub(vector2f v_out, vector2f a_in,vector2f b_in)
{
	v_out[0] = a_in[0] - b_in[0];
	v_out[1] = a_in[1] - b_in[1];
}


/**
  * @brief			vector subtraction
  * @author			Yu Yun
  * @param[out]		v_out: output vector
  * @param[in]		a_in: input vector a
  * @param[in]		b_in: input vector b
  * @retval			None 
  * @remark			benchmarked by: Yu Yun
  */
void vector3f_sub(vector3f v_out, vector3f a_in,vector3f b_in)
{
	v_out[0] = a_in[0] - b_in[0];
	v_out[1] = a_in[1] - b_in[1];
	v_out[2] = a_in[2] - b_in[2];
}


/**
  * @brief			vector subtraction
  * @author			Yu Yun
  * @param[out]		v_out: output vector
  * @param[in]		a_in: input vector a
  * @param[in]		b_in: input vector b
  * @retval			None 
  * @remark			benchmarked by: Yu Yun
  */
void vector4f_sub(vector4f v_out, vector4f a_in,vector4f b_in)
{
	v_out[0] = a_in[0] - b_in[0];
	v_out[1] = a_in[1] - b_in[1];
	v_out[2] = a_in[2] - b_in[2];
	v_out[3] = a_in[3] - b_in[3];
}


/**
  * @brief			scale the input vector with given scale
  * @author			Yu Yun
  * @param[in,out]	v: input&output vector
  * @param[in]		scale: input scale
  * @retval			None 
  * @remark			benchmarked by: Yu Yun
  */
void vector2f_scale(vector2f v, float scale)
{
	v[0] *= scale;
	v[1] *= scale;
}


/**
  * @brief			scale the input vector with given scale
  * @author			Yu Yun
  * @param[in,out]	v: input&output vector
  * @param[in]		scale: input scale
  * @retval			None 
  * @remark			benchmarked by: Yu Yun
  */
void vector3f_scale(vector3f v, float scale)
{
	v[0] *= scale;
	v[1] *= scale;
	v[2] *= scale;
}


/**
  * @brief			scale the input vector with given scale
  * @author			Yu Yun
  * @param[in,out]	v: input&output vector
  * @param[in]		scale: input scale
  * @retval			None 
  * @remark			benchmarked by: Yu Yun
  */
void vector4f_scale(vector4f v, float scale)
{
	v[0] *= scale;
	v[1] *= scale;
	v[2] *= scale;
	v[3] *= scale;
}


/**
  * @brief			multiply a vector with a scaler
  * @author			Yu Yun
  * @param[out]		v_out: input&output vector
  * @param[in]		v_in:  input scale
  * @param[in]		scaler: input scaler
  * @retval			None 
  * @remark			benchmarked by: Yu Yun
  */
void vector2f_multi_scaler(vector2f v_out, vector2f v_in, float scaler)
{
	v_out[0] = v_in[0] * scaler;
	v_out[1] = v_in[1] * scaler;
}


/**
  * @brief			multiply a vector with a scaler
  * @author			Yu Yun
  * @param[out]		v_out: input&output vector
  * @param[in]		v_in:  input scale
  * @param[in]		scaler: input scaler
  * @retval			None 
  * @remark			benchmarked by: Yu Yun
  */
void vector3f_multi_scaler(vector3f v_out, const vector3f v_in, const float scaler)
{
	v_out[0] = v_in[0] * scaler;
	v_out[1] = v_in[1] * scaler;
	v_out[2] = v_in[2] * scaler;
}


/**
  * @brief			multiply a vector with a scaler
  * @author			Yu Yun
  * @param[out]		v_out: input&output vector
  * @param[in]		v_in:  input scale
  * @param[in]		scaler: input scaler
  * @retval			None 
  * @remark			benchmarked by: Yu Yun
  */
void vector4f_multi_scaler(vector4f v_out, vector4f v_in, float scaler)
{
	v_out[0] = v_in[0] * scaler;
	v_out[1] = v_in[1] * scaler;
	v_out[2] = v_in[2] * scaler;
	v_out[3] = v_in[3] * scaler;
}

/**
  * @brief			get the norm of a vector
  * @author			Yu Yun
  * @param[in]		v: input vector
  * @retval			norm 
  * @remark			benchmarked by: Yu Yun
  */
float vector2f_norm(const vector2f v)
{
	float norm = (float)sqrt((double)(v[0] * v[0] + v[1] * v[1]));

	return norm;
}


/**
  * @brief			get the norm of a vector
  * @author			Yu Yun
  * @param[in]		v: input vector
  * @retval			norm 
  * @remark			benchmarked by: Yu Yun
  */
float vector3f_norm(const vector3f v)
{
	float norm = (float)sqrt((double)(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]));

	return norm;
}


/**
  * @brief			get the norm of a vector
  * @author			Yu Yun
  * @param[in]		v: input vector
  * @retval			norm 
  * @remark			benchmarked by: Yu Yun
  */
float vector4f_norm(const vector4f v)
{
	float norm = (float)sqrt((double)(v[0] * v[0] + v[1] * v[1] + v[2] * v[2] + v[3] * v[3]));

	return norm;
}


/**
  * @brief			get the inverse norm of a vector
  * @author			Yu Yun
  * @param[in]		v: input vector
  * @retval			norm 
  * @remark			benchmarked by: Yu Yun
  *					the code is resilient to zero input
  */
float vector2f_inv_norm(const vector2f v)
{
	float inv_norm = float_inv_sqrt(v[0] * v[0] + v[1] * v[1]);
//	float inv_norm = 1.0f/sqrt(v[0] * v[0] + v[1] * v[1]);

	return inv_norm;
}


/**
  * @brief			get the inverse norm of a vector
  * @author			Yu Yun
  * @param[in]		v: input vector
  * @retval			inv_norm 
  * @remark			benchmarked by: Yu Yun
  *					the code is resilient to zero input
  */
float vector3f_inv_norm(const vector3f v)
{
	float inv_norm = float_inv_sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
//	float inv_norm = 1.0f/sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);

	return inv_norm;
}


/**
  * @brief			get the inverse norm of a vector
  * @author			Yu Yun
  * @param[in]		v: input vector
  * @retval			inv_norm 
  * @remark			benchmarked by: Yu Yun
  *					the code is resilient to zero input
  */
float vector4f_inv_norm(const vector4f v)
{
	float inv_norm = float_inv_sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2] + v[3] * v[3]);
//	float inv_norm = 1.0f/sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2] + v[3] * v[3]);

	return inv_norm;
}


/**
  * @brief			normalize a vector
  * @author			Yu Yun
  * @param[in]		v: input vector
  * @retval			None 
  * @remark			benchmarked by: Yu Yun
  *					the code is resilient to zero input
  */
void vector2f_normalize(vector2f v)
{
	float inv_norm = vector2f_inv_norm(v);

	vector2f_scale(v, inv_norm);
}


/**
  * @brief			normalize a vector
  * @author			Yu Yun
  * @param[in]		v: input vector
  * @retval			None 
  * @remark			benchmarked by: Yu Yun
  *					the code is resilient to zero input
  */
void vector3f_normalize(vector3f v)
{
	float inv_norm = vector3f_inv_norm(v);

	vector3f_scale(v, inv_norm);
}


/**
  * @brief			normalize a vector
  * @author			Yu Yun
  * @param[in]		v: input vector
  * @retval			None 
  * @remark			benchmarked by: Yu Yun
  *					the code is resilient to zero input
  */
void vector4f_normalize(vector4f v)
{
	float inv_norm = vector4f_inv_norm(v);

	vector4f_scale(v, inv_norm);
}


/**
  * @brief			calculate the dot product of 2 vectors
  * @author			Yu Yun
  * @param[in]		a_in: input vector a
  * @param[in]		b_in: input vector b
  * @retval			dot_product 
  * @remark			benchmarked by: Yu Yun
  */
float vector2f_dot_product(const vector2f a_in, const vector2f b_in)
{
	float dot_product = a_in[0] * b_in[0] + a_in[1] * b_in[1];

	return dot_product;
}


/**
  * @brief			calculate the dot product of 2 vectors
  * @author			Yu Yun
  * @param[in]		a_in: input vector a
  * @param[in]		b_in: input vector b
  * @retval			dot_product 
  * @remark			benchmarked by: Yu Yun
  */
float vector3f_dot_product(const vector3f a_in, const vector3f b_in)
{
	float dot_product = a_in[0] * b_in[0] + a_in[1] * b_in[1] + a_in[2] * b_in[2];

	return dot_product;
}


/**
  * @brief			calculate the cross product of 2 vectors
  * @author			Yu Yun
  * @param[in]		a_in: input vector a
  * @param[in]		b_in: input vector b
  * @retval			cross_product 
  * @remark			benchmarked by: Yu Yun
  *					for same input vector,...
  */
float vector2f_cross_product( const vector2f a_in, const vector2f b_in)
{
	float cross_product = a_in[0] * b_in[1] - a_in[1] * b_in[0];

	return cross_product;
}


/**
  * @brief			calculate the cross product of 2 vectors
  * @author			Yu Yun
  * @param[out]		v_out: output vector(cross product)
  * @param[in]		a_in: input vector a
  * @param[in]		b_in: input vector b
  * @retval			None 
  * @remark			benchmarked by: Yu Yun
  *					for same input vector,...
  */
void vector3f_cross_product(vector3f v_out, const vector3f a_in, const vector3f b_in)
{
	v_out[0] = a_in[1] * b_in[2] - a_in[2] * b_in[1];
	v_out[1] = a_in[2] * b_in[0] - a_in[0] * b_in[2];
	v_out[2] = a_in[0] * b_in[1] - a_in[1] * b_in[0];
}


/**
  * @brief			calculate the normal vector of 2 vectors(normalized)
  * @author			Yu Yun
  * @param[out]		v_out: norm vector(normalized cross product)
  * @param[in]		a_in: input vector a, arbitrary, not necessary to be unit
  * @param[in]		b_in: input vector b, arbitrary, not necessary to be unit
  * @retval			None 
  * @remark			benchmarked by: Yu Yun
  *  				for same input vector,...
  */
void normal_vector3f_between_vector3f(vector3f normal_vector, const vector3f a, const vector3f b)
{
	vector3f_cross_product(normal_vector, a, b);
	vector3f_normalize(normal_vector);
}


/**
  * @brief			calculate the cosine of the angle between 2 vectors
  * @author			Yu Yun 
  * @param[in]		a_in: input vector a, arbitrary, not necessary to be unit
  * @param[in]		b_in: input vector b, arbitrary, not necessary to be unit
  * @retval			cosine: cosine of the angle between 2 vectors
  * @remark			benchmarked by: Yu Yun
  *					for same input vector,...
  */
float cosine_between_vector2f(const vector2f a,const vector2f b)
{
	float cosine;
	float inv_norm_a, inv_norm_b;

	inv_norm_a = vector2f_inv_norm(a);
	inv_norm_b = vector2f_inv_norm(b);

	cosine = vector2f_dot_product(a,b) * inv_norm_a * inv_norm_b;

	return cosine;

}


/**
  * @brief			calculate the cosine of the angle between 2 vectors
  * @author			Yu Yun 
  * @param[in]		a_in: input vector a, arbitrary, not necessary to be unit
  * @param[in]		b_in: input vector b, arbitrary, not necessary to be unit
  * @retval			cosine: cosine of the angle between 2 vectors
  * @remark			benchmarked by: Yu Yun
  *					for same input vector,...
  */
float cosine_between_vector3f(const vector3f a,const vector3f b)
{
	float cosine;
	float inv_norm_a, inv_norm_b;

	inv_norm_a = vector3f_inv_norm(a);
	inv_norm_b = vector3f_inv_norm(b);

	cosine = vector3f_dot_product(a,b) * inv_norm_a * inv_norm_b;

	return cosine;

}


/**
  * @brief			calculate the sine of the angle between 2 vectors
  * @author			Yu Yun 
  * @param[in]		a_in: input vector a, arbitrary, not necessary to be unit
  * @param[in]		b_in: input vector b, arbitrary, not necessary to be unit
  * @retval			sine: sine of the angle between 2 vectors
  * @remark			benchmarked by: Yu Yun
  *					for same input vector,...
  */
float sine_between_vector2f(const vector2f a, const vector2f b)
{
	float norm_vector;
	float inv_norm_a, inv_norm_b;
	float sine = 0.0f;

	inv_norm_a = vector2f_inv_norm(a);
	inv_norm_b = vector2f_inv_norm(b);
	norm_vector = vector2f_cross_product( a, b);
	sine = norm_vector * inv_norm_a * inv_norm_b;

	sine = float_limit(sine, 1.0f);

	return sine;
}

/**
  * @brief			calculate the sine of the angle between 2 vectors
  * @author			Yu Yun 
  * @param[in]		a_in: input vector a, arbitrary, not necessary to be unit
  * @param[in]		b_in: input vector b, arbitrary, not necessary to be unit
  * @retval			sine: sine of the angle between 2 vectors
  * @remark			benchmarked by: Yu Yun
  *					for same input vector,...
  */
float sine_between_vector3f(const vector3f a, const vector3f b)
{
	vector3f norm_vector;
	float inv_norm_a, inv_norm_b;
	float sine = 0.0f;

	inv_norm_a = vector3f_inv_norm(a);
	inv_norm_b = vector3f_inv_norm(b);
	vector3f_cross_product(norm_vector, a, b);
	sine = vector3f_norm(norm_vector) * inv_norm_a * inv_norm_b;

	sine = float_limit(sine, 1.0f);

	return sine;
}


/**
  * @brief			calculate the angle of the angle between 2 vectors
  * @author			Yu Yun 
  * @param[in]		a_in: input vector a, arbitrary, not necessary to be unit
  * @param[in]		b_in: input vector b, arbitrary, not necessary to be unit
  * @retval			theta: angle between 2 vectors
  * @remark			benchmarked by: Yu Yun
  *					for same input vector,...
  */
float angle_between_vector2f(const vector2f a, const vector2f b)
{
	float theta = 0.0f;
	float sine = 0.0f;
	float cosine = 0.0f;

	sine = sine_between_vector2f(a,b);
	cosine = cosine_between_vector2f(a,b);
	theta = (float)atan2(sine, cosine);

	return theta;
}


/**
  * @brief			calculate the angle of the angle between 2 vectors
  * @author			Yu Yun 
  * @param[in]		a_in: input vector a, arbitrary, not necessary to be unit
  * @param[in]		b_in: input vector b, arbitrary, not necessary to be unit
  * @retval			theta: angle between 2 vectors
  * @remark			benchmarked by: Yu Yun
  *					for same input vector,...
  */
float angle_between_vector3f(const vector3f a, const vector3f b)
{
	float theta = 0.0f;
	float sine = 0.0f;
	float cosine = 0.0f;

	sine = sine_between_vector3f(a,b);
	cosine = cosine_between_vector3f(a,b);
	theta = (float)atan2(sine, cosine);

	return theta;
}



/************************ (C) COPYRIGHT DJI ********************END OF FILE****/

