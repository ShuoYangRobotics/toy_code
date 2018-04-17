#include <urg_node/uav_math.h>
float fConstrain(float value, float min, float max)
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



/***************Function Implementations****************************
* Function prototype :		float invSqrt(float x)
* Function:					calculate 1/Sqrt(x)
* Note:
* 1. 4 times faster than 1/Sqrt(x)
* 2. the maximum relative error over all floating point numbers was 0.00175228
* 3. from Quake-III source code
********************************************************************/
float invSqrt(float x)
{
   float xhalf = 0.5f*x;
   int i = *(int*)&x; 			// get bits for floating VALUE 
   i = 0x5f375a86- (i>>1); 		// gives initial guess y0
   x = *(float*)&i; 			// convert bits BACK to float
   x = x*(1.5f-xhalf*x*x); 		// Newton step, repeating increases accuracy
   return x;
}



float Dot_Product(float A[3], float B[3])
{
    float Product;
    Product = A[0] * B[0] +  A[1] * B[1] +  A[2] * B[2];
    return Product;
}


/*----------------------------------------------
    C = ||A||*||B||sin(theta)* n
-----------------------------------------------*/
void Cross_Product(float A[3], float B[3], float C[3])
{
    C[0] = A[1] * B[2] -  A[2] * B[1];
    C[1] = A[2] * B[0] -  A[0] * B[2];
    C[2] = A[0] * B[1] -  A[1] * B[0];
}

// A,B should be unit vector
float Get_Cos_Between_Vector(float A[3], float B[3])
{
    float Cos;
    Cos = Dot_Product(A, B);
    Cos = fConstrain(Cos, -1.0f, 1.0f);
    return Cos;
}

double Get_Angle_Between_Vector(float A[3], float B[3])
{
    double Angle;
    Angle = acos(Get_Cos_Between_Vector(A, B));
    return Angle;
}

/*---------------------------------------------------------------
 *-- rotation defined in spacial frame
 *-- from spacial to body 
      [ 1 - 2q2^2 - 2q3^3  2(q1q2 + q0q3)     2(q1q3 - q0q2)    ]
  R = | 2(q1q2 - q0q3)     1 - 2q1^2 - 2q3^2  2(q2q3 + q0q1)    |
      [ 2(q1q3 + q0q2)     2(q2q3 - q0q1)     1 - 2q1^2 - 2q2^2 ]  
   
  *-- from body to spacial   
           [ 1 - 2q2^2 - 2q3^3  2(q1q2 - q0q3)     2(q1q3 + q0q2)    ]
  R^(-1) = | 2(q1q2 + q0q3)     1 - 2q1^2 - 2q3^2  2(q2q3 - q0q1)    |
           [ 2(q1q3 - q0q2)     2(q2q3 + q0q1)     1 - 2q1^2 - 2q2^2 ]      
----------------------------------------------------------------*/

   
void Get_Normal_Vector(float N[3], float Q[4])
{   
    float norm;
    N[0] = 2.0f * (Q[1] * Q[3] + Q[0] * Q[2]);
    N[1] = 2.0f * (Q[2] * Q[3] - Q[0] * Q[1]);
    N[2] = 1.0f - 2.0f * (Q[1] * Q[1] + Q[2] * Q[2]);
    norm = invSqrt(N[0] * N[0] + N[1]* N[1] + N[2] * N[2]);
	N[0] *= norm;
	N[1] *= norm;
	N[2] *= norm;
}

//note that the Yaw was negated because PC software make a mistake, there should be no "-" in front of atan2
void Quaternoin_to_Eular(const float Q[4], float *yaw, float *pitch, float *roll)
{
   *yaw  = -atan2(2 * Q[1] * Q[2] + 2 * Q[0]* Q[3], -2 * Q[2]*Q[2] - 2 * Q[3] * Q[3] + 1)* 180.0f/M_PI;
   *pitch = -asin(-2 * Q[1] * Q[3] + 2 * Q[0] * Q[2])* 180.0f/M_PI;
   *roll = atan2(2 * Q[2] * Q[3] + 2 * Q[0] * Q[1], -2 * Q[1] * Q[1] - 2 * Q[2] * Q[2] + 1)* 180.0f/M_PI;
}

/*-- the angles are in RAD --*/
void Quaternoin_to_Eular_RAD(const float Q[4], float *yaw, float *pitch, float *roll)
{   
    float tasin = 0.0f;
   *yaw  = atan2(2 * Q[1] * Q[2] + 2 * Q[0]* Q[3], -2 * Q[2]*Q[2] - 2 * Q[3] * Q[3] + 1);
   tasin = 2 * Q[1] * Q[3] - 2 * Q[0] * Q[2];
   if(tasin >= 1.0f) tasin = 0.9999999f;
   if(tasin <= -1.0f) tasin = -0.9999999f;
   *pitch = -asin(tasin);
   *roll = atan2(2 * Q[2] * Q[3] + 2 * Q[0] * Q[1], -2 * Q[1] * Q[1] - 2 * Q[2] * Q[2] + 1);
}

void Eular_to_Quaternion(float Q[4], float yaw, float pitch, float roll)
{   
    float cos_half_yaw, sin_half_yaw, cos_half_pitch, sin_half_pitch, cos_half_roll, sin_half_roll;
    
    cos_half_yaw = cos(yaw/2.0f);
    sin_half_yaw = sin(yaw/2.0f);
    cos_half_pitch = cos(pitch/2.0f);
    sin_half_pitch = sin(pitch/2.0f);
    cos_half_roll = cos(roll/2.0f);
    sin_half_roll = sin(roll/2.0f);
    
    Q[0] = cos_half_roll * cos_half_pitch * cos_half_yaw + sin_half_roll * sin_half_pitch * sin_half_yaw;
    Q[1] = sin_half_roll * cos_half_pitch * cos_half_yaw - cos_half_roll * sin_half_pitch * sin_half_yaw;
    Q[2] = cos_half_roll * sin_half_pitch * cos_half_yaw + sin_half_roll * cos_half_pitch * sin_half_yaw;
    Q[3] = cos_half_roll * cos_half_pitch * sin_half_yaw - sin_half_roll * sin_half_pitch * cos_half_yaw;
        
}

// note that the result is in RAD form
void Quaternion_to_Yaw(float Q[4], float *Yaw)
{
    *Yaw  = atan2(2 * Q[1] * Q[2] + 2 * Q[0]* Q[3], -2 * Q[2]*Q[2] - 2 * Q[3] * Q[3] + 1);
}


//Quaternion Multiplication: C=A*B
//rotation C = rotation B followed by rotation A
void QuaternionA_Multi_B(const float A[4],const float B[4],float C[4])	//C=A*B
{
  float norm;
  
  C[0]=A[0]*B[0]-A[1]*B[1]-A[2]*B[2]-A[3]*B[3];
  C[1]=A[0]*B[1]+A[1]*B[0]+A[2]*B[3]-A[3]*B[2];
  C[2]=A[0]*B[2]-A[1]*B[3]+A[2]*B[0]+A[3]*B[1];
  C[3]=A[0]*B[3]+A[1]*B[2]-A[2]*B[1]+A[3]*B[0];
  
  norm = invSqrt(C[0] * C[0] + C[1]* C[1] + C[2] * C[2] + C[3] * C[3]);
  C[0] *= norm;
  C[1] *= norm;
  C[2] *= norm;
  C[3] *= norm;
    
  /*-- make sure that the quaternion follows the convention --*/
  if(C[0] < 0.0f)
  {
    C[0] = -C[0];
	C[1] = -C[1];
	C[2] = -C[2];
	C[3] = -C[3];   
  }
}


//the Q_Error is defined on body frame
void Get_Error_Quaternion(const float Q_Cur[4], const float Q_Tar[4], float Q_Error[4])
{
    float Q_temp[4];
	Q_temp[0]=Q_Cur[0];
	Q_temp[1]=-Q_Cur[1];
	Q_temp[2]=-Q_Cur[2];
	Q_temp[3]=-Q_Cur[3];
    //rotation from Cur_Q to Tar_Q
	QuaternionA_Multi_B(Q_temp,Q_Tar,Q_Error);
}


/****************Function Implementations***********************************
 * Func prototype 	: void Measure_Acc_2_Body_Acc(float_num Measure_Acc[3],float Body_Acc[3], float Q[4])
 * brief    		: convert the measured Acc to body Acc
 * param          	: Measure_Acc[3], Body_Acc[3], Q[4]
 * Return         	: None
 *--------------------------------------------------------------------------
 * Note
 * 1. convert the measured Acc to body Acc
***************************************************************************/
void Measure_Acc_2_Body_Acc(float Measure_Acc[3],float Body_Acc[3], float Q[4])
{
	Body_Acc[0] = Measure_Acc[0] - (2.0f * Q[1] * Q[3] - 2.0f * Q[0] *Q[2]);
	Body_Acc[1] = Measure_Acc[1] - (2.0f * Q[0] * Q[1] + 2.0f * Q[2] *Q[3]);
	Body_Acc[2] = Measure_Acc[2] - (1.0f- 2.0f * Q[1] * Q[1] - 2.0f * Q[2] *Q[2]);
}

/*-- get the rotation angle denoted by Quaternion --*/
//notice that the Q[0] should represents for the scaler part 
// recall that cos(2x) = 2*cos(x) - 1 
float Get_Q_Cos_Theta(float Q0)
{
    float Cos_Theta;
 
    
    Cos_Theta = 2.0f * Q0* Q0 - 1.0f;
    
    Cos_Theta = fConstrain(Cos_Theta, -1.0f, 1.0f);
    
    
    return Cos_Theta;
   // Angle = 9;

}


float Get_Cos_G_Over_XY(float Q[4])
{

    float Cos_Theta;
    float Z_projection;
    
    Z_projection = 1.0f - 2.0f * Q[1]*Q[1] - 2.0f * Q[2]* Q[2];
    
    Cos_Theta =  Z_projection/1.0f;
    
    return Cos_Theta;

}


//convert the Simultaneous Orthogonal Rotations Angle(SORA) to Quaternion
// note that the input Angle should be in Rad form
void SORA_to_Quaternion(float Q[4],float ThetaX, float ThetaY, float ThetaZ)   //unit: Rad/s
{
	float Alpha,Cos,Sin,SinOverAlpha,norm;

	if(ThetaX==0&&ThetaY==0&& ThetaZ==0)	  // if no rotation, direct return [1,0,0,0]
	{
	Q[0] = 1.0f;
	Q[1] = 0.0f;
	Q[2] = 0.0f;
	Q[3] = 0.0f;
	}
	else
	{
	Alpha=sqrt(ThetaX*ThetaX + ThetaY*ThetaY + ThetaZ*ThetaZ);
    Cos = cos(Alpha/2.0f);
    Sin = sin(Alpha/2.0f);
//	SinOverAlpha=sin(Alpha/2)/Alpha;
	Q[0]=Cos;			
	Q[1]=Sin * ThetaX / Alpha;
	Q[2]=Sin * ThetaY / Alpha;
	Q[3]=Sin * ThetaZ / Alpha;
					 
	norm = invSqrt(Q[0] * Q[0] + Q[1]* Q[1] + Q[2] * Q[2] + Q[3] * Q[3]);
	Q[0] *= norm;
	Q[1] *= norm;
	Q[2] *= norm;
	Q[3] *= norm;
	}
}


// convert Quaternion to the Simultaneous Orthogonal Rotations Angle(SORA)
void Quaternion_to_SORA(float Q[4], float *AngleX, float *AngleY, float *AngleZ)  ////unit: Rad/s
{
	float Alpha,norm;
	Alpha=2.0f*acos(Q[0]);

	norm= invSqrt(Q[1]*Q[1]+Q[2]*Q[2]+Q[3]*Q[3]);
	*AngleX=Q[1]*norm*Alpha;
	*AngleY=Q[2]*norm*Alpha;
	*AngleZ=Q[3]*norm*Alpha;
 
}




