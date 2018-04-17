/**
  ******************************************************************************
  * @file    math_basic.c
  * @author  Leonardo Yu Yun
  * @version V1.1.0
  * @date    20-March-2014
  * @brief   This file provides firmware functions to manage the following 
  *          functionalities of basic math operation:
  *           + range limit for int, float...
  *           + basic math calculation, square, cube, inv_sqrt
  * @note	 test case result of each func has been stated in remark section 
  *         
  ******************************************************************************  
  */ 


/* Includes ------------------------------------------------------------------*/
#include "math_basic.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/ 

#define HIGH_PRECISION

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/




/**
  * @brief			limit the input value to a given range, for int8_t only
  * @author			Yu Yun
  * @param[in]		value: input value
  * @param[in]		min: lower bound
  * @param[in]		max: upper bound
  * @retval			value: output of the constrained output   
  * @remark			benchmarked by: Yu Yun
  */
int8_t int8_constrain(int8_t value, int8_t min, int8_t max)
{
	if(value > max)
	{
		value = max;
	}
	else if(value < min)
	{
		value = min;
	}

	return value;
}


/**
  * @brief		limit the input value to a given range, for int16_t only
  * @author     Yu Yun
  * @param[in]	value: input value
  * @param[in]	min: lower bound
  * @param[in]	max: upper bound
  * @retval		value: output of the constrained output   
  * @remark		benchmarked by: Yu Yun
  */
int16_t int16_constrain(int16_t value, int16_t min, int16_t max)
{
	if(value > max)
	{
		value = max;
	}
	else if(value < min)
	{
		value = min;
	}

	return value;
}


/**
  * @brief		limit the input value to a given range, for int32_t only
  * @author     Yu Yun
  * @param[in]	value: input value
  * @param[in]	min: lower bound
  * @param[in]	max: upper bound
  * @retval		value: output of the constrained output   
  * @remark		benchmarked by: Yu Yun
  */
int32_t int32_constrain(int32_t value, int32_t min, int32_t max)
{
	if(value > max)
	{
		value = max;
	}
	else if(value < min)
	{
		value = min;
	}

	return value;
}


/**
  * @brief		limit the input value to a given range, for uint16_t only
  * @author     Yu Yun
  * @param[in]	value: input value
  * @param[in]	min: lower bound
  * @param[in]	max: upper bound
  * @retval		value: output of the constrained output   
  * @remark		benchmarked by: Yu Yun
  */
uint8_t uint8_constrain(uint8_t value, uint8_t min, uint8_t max)
{
	if(value > max)
	{
		value = max;
	}
	else if(value < min)
	{
		value = min;
	}

	return value;
}


/**
  * @brief		limit the input value to a given range, for uint16_t only
  * @author     Yu Yun
  * @param[in]	value: input value
  * @param[in]	min: lower bound
  * @param[in]	max: upper bound
  * @retval		value: output of the constrained output   
  * @remark		benchmarked by: Yu Yun
  */
uint16_t uint16_constrain(uint16_t value, uint16_t min, uint16_t max)
{
	if(value > max)
	{
		value = max;
	}
	else if(value < min)
	{
		value = min;
	}

	return value;
}


/**
  * @brief		limit the input value to a given range, for uint32_t only
  * @author     Yu Yun
  * @param[in]	value: input value
  * @param[in]	min: lower bound
  * @param[in]	max: upper bound
  * @retval		value: output of the constrained output   
  * @remark		benchmarked by: Yu Yun
  */
uint32_t uint32_constrain(uint32_t value, uint32_t min, uint32_t max)
{
	if(value > max)
	{
		value = max;
	}
	else if(value < min)
	{
		value = min;
	}

	return value;
}


/**
  * @brief		limit the input value to a given range, for float only
  * @author     Yu Yun
  * @param[in]	value: input value
  * @param[in]	min: lower bound
  * @param[in]	max: upper bound
  * @retval		value: output of the constrained output   
  * @remark		benchmarked by: Yu Yun
  */
float float_constrain(float value, float min, float max)
{
	if(value > max)
	{
		value = max;
	}
	else if(value < min)
	{
		value = min;
	}

	return value;
}


/**
  * @brief		limit the input value to a given range, for double only
  * @author     Yu Yun
  * @param[in]	value: input value
  * @param[in]	min: lower bound
  * @param[in]	max: upper bound
  * @retval		value: output of the constrained output   
  * @remark		benchmarked by: Yu Yun
  */
double double_constrain(double value, double min, double max)
{
	if(value > max)
	{
		value = max;
	}
	else if(value < min)
	{
		value = min;
	}

	return value;
}


/**
  * @brief		limit the input value to a given range, for float only
  * @author     Yu Yun
  * @param[in]	value: input value
  * @param[in]	range: range limit
  * @retval		value: output of the constrained output   
  * @remark		benchmarked by: Yu Yun
  */
float float_limit(float value, float range)
{
	value = float_constrain(value, -range, range);
	return value;
}


/**
  * @brief		limit the input value to a given range, for float only
  * @author     Yu Yun
  * @param[in]	value: input value
  * @param[in]	range: range limit
  * @retval		value: output of the constrained output   
  * @remark		benchmarked by: Yu Yun
  */
double double_limit(double value, double range)
{
	value = double_constrain(value, -range, range);
	return value;
}


/**
  * @brief		calculate the square of a given number, float only
  * @author     Yu Yun
  * @param[in]	x: input value
  * @retval		square: square of the input 
  * @remark		benchmarked by: Yu Yun
  */
float float_square(float x)
{
	float square = x * x;

	return square;
}


/**
  * @brief		calculate the square of a given number, double only
  * @author     Yu Yun
  * @param[in]	x: input value
  * @retval		square: square of the input 
  * @remark		benchmarked by: Yu Yun
  */
double double_square(double x)
{
	double square = x * x;

	return square;
}


/**
  * @brief		calculate the cube of a given number, float only
  * @author     Yu Yun
  * @param[in]	x: input value
  * @retval		cube: cube of the input 
  * @remark		benchmarked by: Yu Yun
  */
float float_cube(float x)
{
	float cube = x * x * x;
	
	return cube;
}


/**
  * @brief		calculate the cube of a given number, double only
  * @author     Yu Yun
  * @param[in]	x: input value
  * @retval		cube: cube of the input 
  * @remark		benchmarked by: Yu Yun
  */
double double_cube(double x)
{
	double cube = x * x * x;
	
	return cube;
}


/**
  * @brief		calculate the cube root of a given number, float only
  * @author     Yu Yun
  * @param[in]	x: input value
  * @retval		cube_root: cube_root of the input 
  * @remark		benchmarked by: Yu Yun
  */
float float_cube_root(float x)
{
	float cube_root = (float)pow(x, (1.0f/3.0f));

	return cube_root;
}



/**
  * @brief		calculate the cube root of a given number, double only
  * @author     Yu Yun
  * @param[in]	x: input value
  * @retval		cube_root: cube_root of the input 
  * @remark		benchmarked by: Yu Yun
  */
double double_cube_root(double x)
{
	double cube_root = pow(x, (1.0/3.0f));

	return cube_root;
}


/**
  * @brief		fast calculate inverse square root of a given number
  * @author     Yu Yun
  * @param[in]	x: input value
  *				 This parameter can be one of the following values:
  *                @arg 1.0e-14: -> 9.985661e+06
  *                @arg 0.0e+00: -> 1.981803e+19 (near infinite)
  *                @arg 1.0e+14: -> 9.989504e-08
  * @retval		x: inverse square root of the input 
  * @remark		benchmarked by: Yu Yun
  *				4 times faster than standard lib
  *				less than 0.2% relative error
  *				the code is resilient to zero input
  */
__INLINE float fast_inv_sqrt(float x)
{
   float xhalf = 0.5f*x;
   int i = *(int*)&x; 			// get bits for floating VALUE 
   i = 0x5f375a86- (i>>1); 		// gives initial guess y0
   x = *(float*)&i; 			// convert bits BACK to float
   x = x*(1.5f-xhalf*x*x); 		// Newton step, repeating increases accuracy
   return x;
}


/**
  * @brief		calculate the inverse square root of a given number, float only
  * @details    return high precision result, but take more time
  *				or return relatively high precision result, faster
  * @author     Yu Yun
  * @param[in]	x:		  input value
  * @retval		inv_sqrt: inverse square root of the input 
  * @remark		benchmarked by: Yu Yun
  *				the code is resilient to zero input
  */
#ifdef HIGH_PRECISION

float float_inv_sqrt(float x)
{
  float inv_sqrt;

	if(x == 0.0)
	{
		inv_sqrt = fast_inv_sqrt((float)x);
	}
	else
	{
		inv_sqrt = (float)(1.0/sqrt(x));
	}

   return inv_sqrt;
}

#else

float float_inv_sqrt(float x)
{
	float inv_sqrt = fast_inv_sqrt(x);

	return inv_sqrt;
}

#endif


/**
  * @brief		calculate the inverse square root of a given number, double only
  * @details    return high precision result, but take more time
  *				or return relatively high precision result, faster
  * @author     Yu Yun
  * @param[in]	x:	      input value
  * @retval		inv_sqrt: inverse square root of the input 
  * @remark		benchmarked by: Yu Yun
  *				the code is resilient to zero input
  */
#ifdef HIGH_PRECISION

double double_inv_sqrt(double x)
{
	double inv_sqrt;

	if(x == 0.0)
	{
		inv_sqrt = (double)fast_inv_sqrt((float)x);
	}
	else
	{
		inv_sqrt = 1.0/sqrt(x);
	}

   return inv_sqrt;
}

#else

double double_inv_sqrt(double x)
{
	double inv_sqrt = (double)fast_inv_sqrt((float)x);

	return inv_sqrt;
}

#endif



/************************ (C) COPYRIGHT DJI ********************END OF FILE****/

