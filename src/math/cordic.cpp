//******************************************************************************
//   https://github.com/the0b/MSP430-CORDIC-sine-cosine
//   Description: This file contains a function to caluculate sine and cosine
//                using CORDIC rotation on a 16 bit MSP430. The function takes
//                any angle in degrees and outputs the answers in Q.15 fixed 
//                point format[(floating point) = (fixed point)/ 2^15]. 
//
//                In creating this file, I referenced the 2004 presentation of
//                fixed point two's complement CORDIC arithmetic presentation
//                by Titi Trandafir of Microtrend Systems which contained an 
//                assembly language program utilizing CORDIC. I also referenced
//                the CORDIC wikipedia page.
//
//   Theo Brower
//   Version    1.01
//   Feb 2011
//     IAR Embedded Workbench Kickstart (Version: 5.20.1)
//
//Copyright (c) 2011 Theo Brower
//
//Permission is hereby granted, free of charge, to any person obtaining a 
//copy of this software and associated documentation files (the "Software"), 
//to deal in the Software without restriction, including without limitation 
//the rights to use, copy, modify, merge, publish, distribute, sublicense, 
//and/or sell copies of the Software, and to permit persons to whom the 
//Software is furnished to do so, subject to the following conditions:
//The above copyright notice and this permission notice shall be included 
//in all copies or substantial portions of the Software.
//
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS 
//OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL 
//THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
//FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS 
//IN THE SOFTWARE.
//******************************************************************************
# include "cordic.h"
# include "math.h"
// Table of arctan's for use with CORDIC algorithm
// Store in decimal representation N = ((2^16)*angle_deg) / 180
Trignometric atantable[ATAN_TAB_N]={  Trignometric(0.78539816),   //atan(2^0) =  0.78539816 rad
                                Trignometric(0.46364761),   //atan(2^-1) = 0.46364761 rad
                                Trignometric(0.24497866),   //atan(2^-2) = 0.24497866 rad
                                Trignometric(0.12435499),   //0.12435499 rad
                                Trignometric(0.06241881),   //0.06241881 rad
                                Trignometric(0.03123983),   //0.03123983 rad
                                Trignometric(0.01562373),   //0.01562373 rad
                                Trignometric(0.00781234),   //0.00781234 rad
                                Trignometric(0.00390623),   //0.00390623 rad
                                Trignometric(0.00195312),   //0.00195312 rad
                                Trignometric(0.00097656),   //0.00097656 rad
                                Trignometric(0.00048828),   //0.00048828 rad
                                Trignometric(0.00024414),   //0.00024414 rad
                                Trignometric(0.00012207),   //0.00012207 rad
                                Trignometric(0.000061035),
                                Trignometric(0.000030518),
                                Trignometric(0.000015259),
                                Trignometric(0.000007629),
                                Trignometric(0.000003815),
                                Trignometric(0.000001907),
                                Trignometric(0.000000954),
                                Trignometric(0.000000477),
                                Trignometric(0.000000238),
                                Trignometric(0.000000119),
                                Trignometric(0.00000006),
                                Trignometric(0.00000003),
                                Trignometric(0.000000015),
                                Trignometric(0.0000000075),
                                Trignometric(0.000000004),
                                Trignometric(0.000000002),
                                Trignometric(0.000000001)                        
};

Trignometric normilize_yaw(Trignometric theta)
{
  if(theta < Trignometric(-PI))
  {
    theta += ((-theta+Trignometric(PI))/Trignometric(2*PI))*Trignometric(2*PI);
  }
  if(theta > Trignometric(PI))
  {
    theta -= ((theta+Trignometric(PI))/Trignometric(2*PI))*Trignometric(2*PI);
  }
  return theta;
}

Trignometric cordic_cos(Trignometric theta)
{
  return cordic_cos_normalized(normilize_yaw(theta));
}

Trignometric cordic_sin(Trignometric theta)
{
  return cordic_sin_normalized(normilize_yaw(theta));
}

Trignometric cordic_sin_normalized(Trignometric theta){
  short sigma,quadAdj,i,shift;
  Trignometric s, x1, x2, y;
  Trignometric *atanptr = atantable;
  Trignometric ans=0;
  char iterations=Trigonometric_Precision;

  //Limit iterations to number of atan values in our table
  iterations = (iterations > ATAN_TAB_N) ? ATAN_TAB_N : iterations;

  //Shift angle to be in range -180 to 180
  while(theta < -PI) theta += 2.0*PI;
  while(theta > PI) theta -= 2.0*PI;
  
  //Shift angle to be in range -90 to 90
  if (theta < -PI_2){
    theta = theta + PI;
    quadAdj = -1;
  } else if (theta > PI_2){
    theta = theta - PI;
    quadAdj = -1;
  } else{
    quadAdj = 1;
  }

  //Shift angle to be in range -45 to 45
  if (theta < -PI_4){
    theta = theta + PI_2;
    shift = -1;
  } else if (theta > PI_4){
    theta = theta - PI_2;
    shift = 1;
  } else{
    shift = 0;
  }

  //Initial values
  x1 = magic_number;    //this will be the cosine result, 
                  //initially the magic number 0.60725293
  y = 0.0;          //y will contain the sine result
  s = 0.0;          //s will contain the final angle
  sigma = 1;      //direction from target angle
  
  for (i=0; i<iterations; i++){
    sigma = (theta - s) > 0 ? 1 : -1;
    if(sigma < 0){
      x2 = x1 + (y >> i);
      y = y - (x1 >> i);
      x1 = x2;
      s -= *atanptr++;
    } else{
      x2 = x1 - (y >> i);
      y = y + (x1 >> i);
      x1 = x2;
      s += *atanptr++;
    }
  }
  
  //Correct for possible overflow in cosine result
  if(x1 < 0) x1 = -x1;
  
  //Push final values to appropriate registers
  if(shift > 0){
    ans = x1;
  } else if (shift < 0){
    ans = -x1;
  } else {
    ans = y;
  }
  //Adjust for sign change if angle was in quadrant 3 or 4
  ans=quadAdj * ans;
  return ans;
}

Trignometric cordic_cos_normalized(Trignometric theta){
  short sigma,quadAdj,i,shift;
  Trignometric s, x1, x2, y;
  Trignometric *atanptr = atantable;
  Trignometric ans=0;
  char iterations=Trigonometric_Precision;
  
  //Limit iterations to number of atan values in our table
  iterations = (iterations > ATAN_TAB_N) ? ATAN_TAB_N : iterations;

  //Shift angle to be in range -180 to 180
  while(theta < -PI) theta += 2.0*PI;
  while(theta > PI) theta -= 2.0*PI;
  
  //Shift angle to be in range -90 to 90
  if (theta < -PI_2){
    theta = theta + PI;
    quadAdj = -1;
  } else if (theta > PI_2){
    theta = theta - PI;
    quadAdj = -1;
  } else{
    quadAdj = 1;
  }
  
  //Shift angle to be in range -45 to 45
  if (theta < -PI_4){
    theta = theta + PI_2;
    shift = -1;
  } else if (theta > PI_4){
    theta = theta - PI_2;
    shift = 1;
  } else{
    shift = 0;
  }
  
  //Initial values
  x1 = magic_number;    //this will be the cosine result, 
                  //initially the magic number 0.60725293
  y = 0;          //y will contain the sine result
  s = 0;          //s will contain the final angle
  sigma = 1;      //direction from target angle
  
  for (i=0; i<iterations; i++){
    sigma = (theta - s) > 0 ? 1 : -1;
    if(sigma < 0){
      x2 = x1 + (y >> i);
      y = y - (x1 >> i);
      x1 = x2;
      s -= *atanptr++;
    } else{
      x2 = x1 - (y >> i);
      y = y + (x1 >> i);
      x1 = x2;
      s += *atanptr++;
    }
  }
  
  //Correct for possible overflow in cosine result
  if(x1 < 0) x1 = -x1;
  
  //Push final values to appropriate registers
  if(shift > 0){
    ans = -y;
  } else if (shift < 0){
    ans = y;
  } else {
    ans = x1;
  }
  
  //Adjust for sign change if angle was in quadrant 3 or 4
  ans = quadAdj * ans;
  return ans;
}