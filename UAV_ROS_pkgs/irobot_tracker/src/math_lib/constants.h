/**
  ******************************************************************************
  * @file    constants.h
  * @author  Leonardo Yu Yun
  * @version V1.1.0
  * @date    29-March-2014
  * @brief   This file contains all the constants used in mathmatic calculation 
  *			 and unit conversion
  *           + costants: pi, e, G,  
  *           + unit conversion:   m,inche, yard,miles, kg, lb, Farenheite, Celsius..
  ******************************************************************************  
  */ 


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CONSTANTS_H
#define __CONSTANTS_H



/* Includes ------------------------------------------------------------------*/
#include <math.h>
#include <stdint.h>


/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/


/** @defgroup Math_constants 
  * @{
  */ 
#define C_PI			(double) 3.141592653589793
#define C_PI_F			(float)  3.1415927f

#define C_DEG2RAD		(double) 0.01745329251994330
#define C_DEG2RAD_F		(float)  0.017453292f

#define C_RAD2DEG		(double) 57.29577951308232
#define C_RAD2DEG_F		(float)  57.295780f

#define C_PIOTWO		(double) 1.570796326794897
#define C_PIOTWO_F		(float)  1.5707964f

#define C_TWOPI			(double) 6.283185307179586
#define C_TWOPI_F		(float)  6.2831855f

#define C_ROOT2			(double) 1.414213562373095
#define C_ROOT2_F		(float)  1.4142135f

#define C_ROOT3			(double) 1.732050807568877
#define C_ROOT3_F		(float)  1.7320508f

#define C_E				(double) 2.718281828459046
#define C_E_F			(float)  2.7182817f

/* the above parameters have been varified in matlab, only preserves the exact precision */

/**
  * @}
  */ 


/** @defgroup Physical_constants 
  * @{
  */ 
#define C_G0MERSEC2		(double) 9.80665				/* g0 in m/s^2 */
#define C_G0MERSEC2_F	(float)  9.80665f				/* g0 in m/s^2 */

#define C_G				(double) 9.7873					/* g! */
#define C_G_F			(float)  9.7873f				/* g! */

#define C_G0			(double) 32.17404856			/* (C_G0MERSEC2/C_FT2M), standard gravity */
#define C_G0_F			(float)  32.17404856f			/* (C_G0MERSEC2/C_FT2M), standard gravity */

#define C_P0N_M2		(double) 1.01325e5				/* p0 in N/m^2 */
#define C_P0N_M2_F		(float)  1.01325e5				/* p0 in N/m^2 */

#define C_P0			(double) 14.692125				/* (C_P0N_M2*1.450e-4), standard atomospheric pressure */
#define C_P0_F			(float)  14.692125f				/* (C_P0N_M2*1.450e-4), standard atomospheric pressure */

#define C_DEGF0_R		(double) 459.67					/* absolute zero on Fahrenheit scale */
#define C_DEGC0_T		(double) 273.16					/* absolute zero on Celsius scale */

#define C_DEGC0_DEGF	(double) 32.0					/* Farenheit to Celsius : (F - 32) / 1.8 = C */
#define C_DEGF_PER_DEGC	(double)  1.8					/* Celsius to Farenheit : (C x 1.8) + 32 = F */


#define C_G_CONST		(double) 1.068944098e-09		/* 6.6732e-11*CUBE(C_M2FT)/C_KG2LBM */
#define C_G_CONST_F		(float)  1.068944098e-09f		/* 6.6732e-11*CUBE(C_M2FT)/C_KG2LBM */

#define C_EARTH_MASS	(double) 1.317041554e25			/* 5.974e24 * C_KG2LBM */
#define C_EARTH_MASS_F	(float)  1.317041554e25f		/* 5.974e24 * C_KG2LBM */

#define C_N0_AVOGADRO	(double) 6.02205e23				
#define C_N0_AVOGADRO_F	(float)  6.02205e23f				

#define C_R_IDEAL_SU	(double) 8.31434
#define C_R_IDEAL_SU_F	(float)  8.31434f

#define C_K_BOLTZMANN	(double) 1.380622e-23
#define C_K_BOLTZMANN_F	(float)  1.380622e-23f

#define C_C_LIGHT		(double) 983571194.2			/* 2.9979250e+8 * C_M2FT */
#define C_C_LIGHT_F		(float) 983571194.2f			/* 2.9979250e+8 * C_M2FT */

#define C_ECHARGE		(double) 1.6021917e-19		
#define C_ECHARGE_F		(float) 1.6021917e-19f		 

#define C_DOFA			(double) 0.080719353			/* 1.293 * C_KG2LBM/CUBE(C_M2FT) */
#define C_DOFA_F		(float)  0.080719353f			/* 1.293 * C_KG2LBM/CUBE(C_M2FT) */

#define C_DOFH2O		(double) 62.427960576			/* 1.000e3 * C_KG2LBM/CUBE(C_M2FT) */
#define C_DOFH2O_F		(float)  62.427960576f			/* 1.000e3 * C_KG2LBM/CUBE(C_M2FT) */

#define C_STOFH2O		(double) 75.6
#define C_STOFH2O_F		(float)  75.6f

#define C_VOFH2O		(double) 1.787e-3
#define C_VOFH2O_F		(float)  1.787e-3f

#define C_SOUND0VEL		(double) 1087.598425			/* 331.5 * C_M2FT */
#define C_SOUND0VEL_F	(float)  1087.598425f			/* 331.5 * C_M2FT */

#define C_SOUND20VEL	(double) 1126.64042				/* 343.4 * C_M2FT */
#define C_SOUND20VEL_F	(float)  1126.64042f			/* 343.4 * C_M2FT */

#define C_WGS84_a		(double) 6378137.0				/* WGS-84 semimajor axis(m) */
#define C_WGS84_a_F		(float)  6378137.0f				/* WGS-84 semimajor axis(m) */

#define C_WGS84_b		(double) 6356752.3142			/* WGS-84 semiminor axis(m) */
#define C_WGS84_b_F		(float)  6356752.3142f			/* WGS-84 semiminor axis(m) */

#define C_WIE			(double) 7.2321151467e-05		/* WGS-84 earth rotation rate (rad/s) */
#define C_WIE_F			(float)  7.2321151467e-05f		/* WGS-84 earth rotation rate (rad/s) */

#define C_EARTH			(double) 6378137.0
/**
  * @}
  */ 


/** @defgroup Unit_conversion
  * @{
  */ 
#define C_IN2M			(double) 0.0254					/* inches to meters */
#define C_IN2M_F		(float)  0.0254f				/* inches to meters */

#define C_FT2M			(double) 0.3048					/* (C_IN2M*12) feet to meters */
#define C_FT2M_F		(float)  0.3048f				/* (C_IN2M*12) feet to meters */

#define C_M2FT			(double) 3.280839895			/* (1.0/C_FT2M) */
#define C_M2FT_F		(float)  3.280839895f			/* (1.0/C_FT2M) */

#define C_YD2M			(double) 0.9144					/* (C_FT2M*3) yards to meters */
#define C_YD2M_F		(float)  0.9144f				/* (C_FT2M*3) yards to meters */

#define C_NMI2M			(double) 1852.0					/* nautical miles to meters */
#define C_NMI2M_F		(float)  1852.0f				/* nautical miles to meters */

#define C_MI2M			(double) 1609.344				/* (C_FT2M*5280) miles to meters */
#define C_MI2M_F		(float)  1609.344f				/* (C_FT2M*5280) miles to meters */

#define C_LBM2KG		(double) 0.45359237				/* lb mass */
#define C_LBM2KG_F		(float)  0.45359237f			/* lb mass */
#define C_KG2LBM		(double) 2.204622622			/* (1/C_LBM2KG) */
#define C_KG2LBM_F		(float)  2.204622622f			/* (1/C_LBM2KG) */

#define C_MM2M			(double) 0.001		
#define C_MM2M_F		(float)  0.001f
#define C_M2MM			(double) 1000.0
#define C_M2MM_F		(float)  1000.0f

#define C_IN2M			(double) 0.0254
#define c_IN2M_F		(float)  0.0254f
#define C_M2IN			(double) 39.37007874
#define C_M2IN_F		(float)  39.37007874f

#define C_IN2MM			(double) 25.4
#define C_IN2MM_F		(float)  25.4f
#define C_MM2IN			(double) 0.03937007874
#define C_MM2IN_F		(float)  0.03937007874f

#define C_FPS2KT		(double) 0.5924838
#define C_FPS2KT_F		(float)  0.5924838f
#define C_KT2FPS		(double) 1.68780986
#define C_KT2FPS_F		(float)  1.68780986f

#define C_SQIN2SQET		(double) 0.006944444444444444
#define C_SQIN2SQET_F	(float)  0.0069444445f
#define C_SQFT2SQIN		(double) 144.0
#define C_SQFT2SQIN_F	(float)  144.0f

#define C_GPM2CFS		(double) 0.0022280093
#define C_GPM2CFS_F		(float)  0.0022280093f
#define C_CFS2GPM		(double) 448.83117
#define C_CFS2GPM_F		(float)  448.83117f

#define C_C2K			(double) 273.16					
#define C_C2K_F			(float)  273.16f	
#define C_F2R			(double) 459.69
#define C_F2R_F			(float)  459.69f
/**
  * @}
  */ 



/* Exported macro ------------------------------------------------------------*/

/* macros for basic math ******************************************************/
#define MAX( a, b )				((a) > (b) ? (a) : (b))
#define MIN( a, b )				((a) < (b) ? (a) : (b))

#define LIMIT ( x, low, hi)		(((x)<=(hi)) ? (((x)>=(low)) ? (x) : (low) : (hi))

#define FABS( x )				((x) < 0.0f ? -(x) : (x))
#define ABS( x )				((x) < 0 ? -(x) : (x))
#define SQR( x )				((x)*(x))
#define FTYPE_ABS FABS


/* macros for temperature conversions *****************************************/
#define DEGF_TO_DEGR( f )		((f) + C_DEGF0_R)
#define DEGR_TO_DEGF( r )		((r) - C_DEGF0_R)
#define DEGC_TO_DEGK( c )		((c) + C_DEGC0_T)
#define DEGK_TO_DEGC( k )		((k) - C_DEGC0_T)
#define DEGC_TO_DEGF( c )		((c)*C_DEGF_PER_DEGC + C_DEGC0_DEGF)
#define DEGF_TO_DEGC( f )		(((f) - C_DEGC0_DEGF)/C_DEGF_PER_DEGC)


/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/  



#endif /*__CONSTANTS_H */



/************************ (C) COPYRIGHT DJI ********************END OF FILE****/





