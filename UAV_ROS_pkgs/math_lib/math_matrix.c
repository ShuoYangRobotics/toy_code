/**
  ******************************************************************************
  * @file    math_matrix.c
  * @author  Leonardo Yu Yun
  * @version V1.1.0
  * @date    20-March-2014
  * @brief   This file provides firmware functions to manage the following 
  *          functionalities of matrix operation:
  *           + basic operation: zero, set value, copy, add, sub, scale, norm
  *           + advanced operation: dot, cross, sine, cosine, angle between vectors
  * @note	 test case result of each func has been stated in remark section 
  *           
  ******************************************************************************  
  */ 


/* Includes ------------------------------------------------------------------*/
#include "math_matrix.h"
#include "math_vector.h"



/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/ 
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


/**
  * @brief			set matrix elements to zero
  * @author			Yu Yun
  * @param[in,out]	m: input&output matrix
  * @retval			None 
  * @remark			benchmarked by: Yu Yun
  */
void matrix3f_zero(matrix3f m)
{
	vector3f_set_value(m[0], 0.0f, 0.0f, 0.0f);
	vector3f_set_value(m[1], 0.0f, 0.0f, 0.0f);
	vector3f_set_value(m[2], 0.0f, 0.0f, 0.0f);
}


/**
  * @brief			set matrix elements to user defined values
  * @author			Yu Yun
  * @param[in,out]	matrix: input&output matrix
  * @param[in]		a11: input value for element a11
  * @param[in]		a12: input value for element a12
  * @retval			None 
  * @remark			benchmarked by: Yu Yun
  */
void matrix3f_set_value(matrix3f m, float a11, float a12, float a13,
									float a21, float a22, float a23,
									float a31, float a32, float a33)
{
	vector3f_set_value(m[0], a11, a12, a13);
	vector3f_set_value(m[1], a21, a22, a23);
	vector3f_set_value(m[2], a31, a32, a33);
}


/**
  * @brief			set identity matrix 
  * @author			Yu Yun
  * @param[in,out]	m: input&output matrix
  * @retval			None 
  * @remark			benchmarked by: Yu Yun
  */
void matrix3f_eye(matrix3f m)
{
	matrix3f_set_value(m, 1.0f, 0.0f, 0.0f,
						  0.0f, 1.0f, 0.0f,
						  0.0f, 0.0f, 1.0f);
}


/**
  * @brief			copy the value in matrix src to matrix dest
  * @author			Yu Yun
  * @param[out]		dest: output matrix
  * @param[in]		src: input matrix
  * @retval			None 
  * @remark			benchmarked by: Yu Yun
  */
void matrix3f_copy(matrix3f dest, matrix3f src)
{
	vector3f_copy(dest[0], src[0]);
	vector3f_copy(dest[1], src[1]);
	vector3f_copy(dest[2], src[2]);
}


/**
  * @brief			matrix addition
  * @author			Yu Yun
  * @param[out]		m_out: output matrix
  * @param[in]		a_in: input matrix a
  * @param[in]		b_in: input matrix b
  * @retval			None 
  * @remark			benchmarked by: Yu Yun
  */
void matrix3f_add(matrix3f m_out, matrix3f a_in,matrix3f b_in)
{
	vector3f_add(m_out[0], a_in[0], b_in[0]);
	vector3f_add(m_out[1], a_in[1], b_in[1]);	
	vector3f_add(m_out[2], a_in[2], b_in[2]);
}

/**
  * @brief			matrix addition
  * @author			Yu Yun
  * @param[out]		m_out: output matrix
  * @param[in]		a_in: input matrix a
  * @param[in]		b_in: input matrix b
  * @retval			None 
  * @remark			benchmarked by: Yu Yun
  */
void matrix3f_add3(matrix3f m_out, matrix3f a_in, matrix3f b_in, matrix3f c_in)
{
	vector3f_add3(m_out[0], a_in[0], b_in[0], c_in[0]);
	vector3f_add3(m_out[1], a_in[1], b_in[1], c_in[1]);	
	vector3f_add3(m_out[2], a_in[2], b_in[2], c_in[2]);
}


/**
  * @brief			matrix subtraction
  * @author			Yu Yun
  * @param[out]		m_out: output matrix
  * @param[in]		a_in: input matrix a
  * @param[in]		b_in: input matrix b
  * @retval			None 
  * @remark			benchmarked by: Yu Yun
  */
void matrix3f_sub(matrix3f m_out, matrix3f a_in,matrix3f b_in)
{
	vector3f_sub(m_out[0], a_in[0], b_in[0]);
	vector3f_sub(m_out[1], a_in[1], b_in[1]);	
	vector3f_sub(m_out[2], a_in[2], b_in[2]);
}


/**
  * @brief			scale the input matrix with given scale
  * @author			Yu Yun
  * @param[in,out]	m: input&output matrix
  * @param[in]		scale: input scale
  * @retval			None 
  * @remark			benchmarked by: Yu Yun
  */
void matrix3f_scale(matrix3f m, float scale)
{
	vector3f_scale(m[0], scale);
	vector3f_scale(m[1], scale);
	vector3f_scale(m[2], scale);
}


/**
  * @brief			multiply a matrix with a scaler
  * @author			Yu Yun
  * @param[out]		m_out: output matrix
  * @param[in]		m_in:  input matrix
  * @param[in]		scale: input scaler
  * @retval			None 
  * @remark			benchmarked by: Yu Yun
  */
void matrix3f_multi_scaler(matrix3f m_out, matrix3f m_in, float scaler)
{
	vector3f_multi_scaler(m_out[0], m_in[0], scaler);
	vector3f_multi_scaler(m_out[1], m_in[1], scaler);
	vector3f_multi_scaler(m_out[2], m_in[2], scaler);
}


/**
  * @brief			get the transpose of given matrix
  * @author			Yu Yun
  * @param[out]		dest: output matrix(transpose)
  * @param[in]		src: input matrix
  * @retval			None 
  * @remark			benchmarked by: Yu Yun
  */
void get_matrix3f_transpose(matrix3f dest, matrix3f src)
{
	uint8_t m, n;

	for(m=0; m<3; m++)
	{
		for(n=m; n<3; n++)
		{
			dest[m][n] = src[n][m];
			dest[n][m] = src[m][n];
		}
	}
}


/**
  * @brief			take the transpose of given matrix
  * @author			Yu Yun
  * @param[in,out]	m: input&output matrix
  * @retval			None 
  * @remark			benchmarked by: Yu Yun
  */
void take_matrix3f_transpose(matrix3f m)
{
	uint8_t row, column;
	float temp;

	for(row=0; row<3; row++)
	{
		for(column=row+1; column<3; column++)
		{
			temp = m[row][column];
			m[row][column] = m[column][row];
			m[column][row] = temp;
		}
	}
}



/**
  * @brief			get the determinent of given matrix
  * @author			Yu Yun
  * @param[in]		m: input matrix
  * @retval			det: determinent of matrix
  * @remark			benchmarked by: Yu Yun
  */
float matrix3f_det(matrix3f m)
{
	float det = 1.0f;

	det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
		- m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
		+ m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);

	return det;
}


/**
  * @brief			get the inverse of given matrix
  * @author			Yu Yun
  * @param[out]		dest: output matrix(inverse)
  * @param[in]		src: input matrix
  * @retval			None 
  * @remark			benchmarked by: Yu Yun
  */
void get_matrix3f_inverse(matrix3f m_out, matrix3f m_in)
{
	m_out[0][0] = m_in[1][1] * m_in[2][2] - m_in[1][2]  * m_in[2][1];
	m_out[0][1] = m_in[0][2] * m_in[2][1] - m_in[0][1]  * m_in[2][2];
	m_out[0][2] = m_in[0][1] * m_in[1][2] - m_in[0][2]  * m_in[1][1];

	m_out[1][0] = m_in[1][2] * m_in[2][0] - m_in[1][0]  * m_in[2][2];
	m_out[1][1] = m_in[0][0] * m_in[2][2] - m_in[0][2]  * m_in[2][0];
	m_out[1][2] = m_in[0][2] * m_in[1][0] - m_in[0][0]  * m_in[1][2];

	m_out[2][0] = m_in[1][0] * m_in[2][1] - m_in[1][1]  * m_in[2][0];
	m_out[2][1] = m_in[0][1] * m_in[2][0] - m_in[0][0]  * m_in[2][1];
	m_out[2][2] = m_in[0][0] * m_in[1][1] - m_in[0][1]  * m_in[1][0];

	matrix3f_scale(m_out, 1.0f/matrix3f_det(m_in));

}



/**
  * @brief			matrix mutiplication
  * @author			Yu Yun
  * @param[out]		m_out: output matrix
  * @param[in]		a_in: input matrix a
  * @param[in]		b_in: input matrix b
  * @retval			None 
  * @remark			benchmarked by: Yu Yun
  */
void matrix3f_multi_matrix3f(matrix3f m_out, matrix3f a_in,matrix3f b_in)
{
	uint8_t row, column, k;
	float sum = 0.0f;

	for(row=0; row<3; row++)
	{
		for(column=0; column<3; column++)
		{
			sum = 0.0f;
			for(k=0; k<3; k++)
			{
				sum += a_in[row][k] * b_in[k][column];
			}
			m_out[row][column] = sum;
		}
	}
}


/**
  * @brief			matrix mutiply vector
  * @author			Yu Yun
  * @param[out]		v_out: output vector
  * @param[in]		m: input matrix 
  * @param[in]		v: input vector
  * @retval			None 
  * @remark			benchmarked by: Yu Yun
  */
void matrix3f_multi_vector3f(vector3f v_out, matrix3f m, vector3f v)
{
	uint8_t row, k;
	float sum = 0.0f;

	for(row=0; row<3; row++)
	{
		sum = 0.0f;

		for(k=0; k<3; k++)
		{
			sum += m[row][k] * v[k];
			v_out[row]= sum;
		}
	}
}





/************************ (C) COPYRIGHT DJI ********************END OF FILE****/



 
  

