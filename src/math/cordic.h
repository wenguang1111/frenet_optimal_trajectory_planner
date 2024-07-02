//******************************************************************************
//
//   Description: Header file for CORDIC sine/cosine function, see .c file for
//                more detailed description.
//
//   T. Brower
//   Version    1.00
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


#ifndef CORDIC_H
#define CORDIC_H

#define int_2_13 cnl::scaled_integer<int16_t, cnl::power<-13>> 
#define int_8_7 cnl::scaled_integer<int16_t, cnl::power<-7>>

# include "cnl/all.h"
// Function to find the sine and cosine of an angle
// using CORDIC
void cordic_sincos(int, 
                   char, 
                   int *,
                   int *);

int_2_13  cordic_sin(int_2_13 theta);
int_2_13  cordic_cos(int_2_13  theta);
template<typename T>
int_2_13 cordic_atan(T y, T x);
template<typename T>
T cordic_sqrt(T input);
// cnl::scaled_integer<int16_t, cnl::power<-13>> sin_int_1_15(cnl::scaled_integer<int16_t, cnl::power<-13>> angle);
// cnl::scaled_integer<int16_t, cnl::power<-13>> cos_int_1_15(cnl::scaled_integer<int16_t, cnl::power<-13>> angle);

#endif /* CORDIC_H */