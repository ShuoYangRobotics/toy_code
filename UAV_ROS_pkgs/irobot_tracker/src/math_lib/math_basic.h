/**
  ******************************************************************************
  * @file    math_basic.h
  * @author  Leonardo Yu Yun
  * @version V1.1.0
  * @date    20-March-2014
  * @brief   This file contains all the functions prototypes for basic math lib
  *         
  ******************************************************************************  
  */ 


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MATH_BASIC_H
#define __MATH_BASIC_H

/* Includes ------------------------------------------------------------------*/
#include <math.h>
#include <stdint.h>
#include "constants.h"


/* Exported types ------------------------------------------------------------*/

/** 
  * @brief   frequent used vector and matrix typedef 
  */
typedef float			vector2f[2];
typedef float			vector3f[3];
typedef float			vector4f[4];

typedef vector2f		matrix2f[2];
typedef vector3f		matrix3f[3];
typedef vector4f		matrix4f[4];


/* Exported constants --------------------------------------------------------*/

#define __INLINE         __inline

/* Exported macro ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/  


/* range limit ****************************************************************/
int8_t int8_constrain(int8_t value, int8_t min, int8_t max);
int16_t int16_constrain(int16_t value, int16_t min, int16_t max);
int32_t int32_constrain(int32_t value, int32_t min, int32_t max);

uint8_t uint8_constrain(uint8_t value, uint8_t min, uint8_t max);
uint16_t uint16_constrain(uint16_t value, uint16_t min, uint16_t max);
uint32_t uint32_constrain(uint32_t value, uint32_t min, uint32_t max);

float float_constrain(float value, float min, float max);
double double_constrain(double value, double min, double max);

float float_limit(float value, float range);
double double_limit(double value, double range);


/* basic operation ************************************************************/
float float_square(float x);
double double_square(double x);

float float_cube(float x);
double double_cube(double x);

float float_cube_root(float x);
double double_cube_root(double x);

float float_inv_sqrt(float x);
double double_inv_sqrt(double x);




#endif /*__MATH_BASIC_H */



/************************ (C) COPYRIGHT DJI ********************END OF FILE****/

