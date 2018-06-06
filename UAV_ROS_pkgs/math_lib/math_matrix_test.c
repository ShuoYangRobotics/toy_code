/**
******************************************************************************
* @file    math_basic.c
* @author  Leonardo Yu Yun
* @version V1.1.0
* @date    20-March-2014
* @brief   This file provides firmware functions to manage the following 
*          functionalities basic math operation:
*           + basic math calculation
*         
******************************************************************************  
*/ 


/* Includes ------------------------------------------------------------------*/
#include "math_matrix_test.h"
#include "math_matrix.h"
#include <stdio.h>



/* Private typedef -----------------------------------------------------------*/
typedef enum
{
	matrix3f_zero_test,
	matrix3f_copy_test,
	matrix3f_add_test,
	matrix3f_scale_test,
	get_matrix3f_transpose_test,
	set_matrix3f_transpose_test,
	matrix3f_determinent_test,
	matrix3f_matrix3f_multi_test,
	matrix3f_inverse_test,

}math_matrix_test_case;

/* Private define ------------------------------------------------------------*/ 
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

void print_matrix3f(matrix3f a )
{
	printf("row1: %f, %f, %f \r\n", a[0][0], a[0][1], a[0][2]);
	printf("row2: %f, %f, %f \r\n", a[1][0], a[1][1], a[1][2]);
	printf("row3: %f, %f, %f \r\n", a[2][0], a[2][1], a[2][2]);

}

void matrix3f_zero_test_case(void)
{
	matrix3f a;

	matrix3f_zero(a);

	print_matrix3f(a );
}

void matrix3f_copy_test_case(void)
{
	matrix3f a;
	matrix3f b;

	matrix3f_zero(a);

	print_matrix3f(b);
	matrix3f_copy(b,a);
	print_matrix3f(b);
}

void matrix3f_add_test_case(void)
{
	matrix3f a;
	matrix3f b;
	matrix3f c;

	matrix3f_zero(a);
	matrix3f_zero(b);

	matrix3f_add(c, a, b);
	matrix3f_sub(c, a, b);
	print_matrix3f(c);
}

void matrix3f_matrix3f_multi_test_case(void)
{
	matrix3f a;
	matrix3f b;
	matrix3f c;

	matrix3f_set_value(a, 1.0f, 2.0f, 3.0f,
						  1.0f, 2.0f, 3.0f,
						  1.0f, 2.0f, 3.0f);
	matrix3f_set_value(b, 1.0f, 1.0f, 1.0f,
						  2.0f, 2.0f, 2.0f,
						  3.0f, 3.0f, 3.0f);

	matrix3f_multi_matrix3f(c, a, b);
	print_matrix3f(c);
}


void matrix3f_scale_test_case(void)
{
	matrix3f a;

	matrix3f_set_value(a, 1.0f, 2.0f, 3.0f,
						  1.0f, 2.0f, 3.0f,
						  1.0f, 2.0f, 3.0f);

	matrix3f_scale(a, 0.5f);
	print_matrix3f(a);
}

void get_matrix3f_transpose_test_case(void)
{
	matrix3f a;
	matrix3f b;

	matrix3f_set_value(a, 1.1f, 1.2f, 1.3f,
						  2.1f, 2.2f, 2.3f,
						  3.1f, 3.2f, 3.3f);

	get_matrix3f_transpose(b, a);
	print_matrix3f(a);
	print_matrix3f(b);
}

void set_matrix3f_transpose_test_case(void)
{
	matrix3f a;

	matrix3f_set_value(a, 1.1f, 1.2f, 1.3f,
						  2.1f, 2.2f, 2.3f,
						  3.1f, 3.2f, 3.3f);
	print_matrix3f(a);

	take_matrix3f_transpose(a);

	print_matrix3f(a);
}


void matrix3f_determinent_test_case(void)
{
	matrix3f a;
	float det;


	//matrix3f_set_value(a, 1.1f, 1.2f, 1.3f,
	//					  2.1f, 2.2f, 2.3f,
	//					  3.1f, 3.2f, 3.3f);
	matrix3f_eye(a);

	print_matrix3f(a);

	det = matrix3f_det(a);


	printf("det : %f", det);
}

void matrix3f_inverse_test_case(void)
{
	matrix3f a;
	matrix3f b;
	matrix3f c;



	matrix3f_set_value(a, 1.1f, 3.2f, 1.3f,
						  2.1f, 2.2f, 2.3f,
						  5.1f, 3.2f, 3.3f);

	//matrix3f_eye(a);

	get_matrix3f_inverse(b,a);

	matrix3f_multi_matrix3f(c, a, b);

	print_matrix3f(a);
	print_matrix3f(b);
	print_matrix3f(c);

}

/**
* @brief  limit the input value to a given range
* @param  value: input value
* @param  min: lower bound
* @param  max: upper bound
* @retval None
*/


void math_matrix_debug_test_case(void)
{
	uint8_t test_case = matrix3f_inverse_test;

	switch(test_case)
	{
	case matrix3f_zero_test:			matrix3f_zero_test_case();				break;
	case matrix3f_copy_test:			matrix3f_copy_test_case();				break;
	case matrix3f_add_test:				matrix3f_add_test_case();				break;
	case matrix3f_scale_test:			matrix3f_scale_test_case();				break;	
	case get_matrix3f_transpose_test:	get_matrix3f_transpose_test_case();		break;	
	case set_matrix3f_transpose_test:	set_matrix3f_transpose_test_case();		break;		
	case matrix3f_determinent_test:		matrix3f_determinent_test_case();		break;		
	case matrix3f_matrix3f_multi_test:  matrix3f_matrix3f_multi_test_case();	break;
	case matrix3f_inverse_test:			matrix3f_inverse_test_case();			break;		
	
	}

}

/************************ (C) COPYRIGHT DJI ********************END OF FILE****/

