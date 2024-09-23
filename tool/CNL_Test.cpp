/*
How to compile this file:
clang++ -isystem /home/wenguang/installed/cnl/include -std=c++20 CNL_Test.cpp -o CNL_Test
clang++ -isystem /home/wenguang/installed/cnl_Debug/include -std=c++20 CNL_Test.cpp ../src/math/cordic.cpp -o CNL_Test
*/
#include "cnl/all.h"
// #include "/home/wenguang/workplace/frenet_optimal_trajectory_planner/src/math/cordic.h"
// #include <cmath>
#include <iostream>
// #include <limits>
// using cnl::power;
using cnl::scaled_integer;
typedef cnl::scaled_integer<int16_t, cnl::power<-3>> int_12_3;
typedef cnl::scaled_integer<int32_t, cnl::power<-20>> int_11_20;
typedef cnl::scaled_integer<int64_t, cnl::power<-20>> int_21_10;
typedef cnl::scaled_integer<int16_t, cnl::power<-13>> int_2_13;
// typedef cnl::scaled_integer<uint64_t, cnl::power<-30>> uint_64_30;
// typedef cnl::rounding_integer<uint16_t, cnl::nearest_rounding_tag> rounding_7_9;
template<int IntegerDigits, int FractionalDigits, class Narrowest>
using tie_to_pos_inf = cnl::static_number<
        IntegerDigits + FractionalDigits, -FractionalDigits, cnl::tie_to_pos_inf_rounding_tag,
        cnl::saturated_overflow_tag>;
template<int IntegerDigits, int FractionalDigits, class Narrowest>
using native_rounding = cnl::static_number<
        IntegerDigits + FractionalDigits, -FractionalDigits, cnl::native_rounding_tag,
        cnl::saturated_overflow_tag>;
template<int IntegerDigits, int FractionalDigits, class Narrowest>
using nearest_rounding = cnl::static_number<
        IntegerDigits + FractionalDigits, -FractionalDigits, cnl::nearest_rounding_tag,
        cnl::saturated_overflow_tag>;
template<int IntegerDigits, int FractionalDigits, class Narrowest>
using neg_inf_rounding = cnl::static_number<
        IntegerDigits + FractionalDigits, -FractionalDigits, cnl::neg_inf_rounding_tag,
        cnl::saturated_overflow_tag>;

using int_23_8_tie_to_pos_inf = tie_to_pos_inf<23, 8, int32_t>;
using int_23_8_native_rounding = native_rounding<23,8,int32_t>;
using int_23_8_nearest_rounding = nearest_rounding<23,8,int32_t>;
using int_23_8_neg_inf_rounding = neg_inf_rounding<23,8,int32_t>;

// void calculate(float number)
// {
//   int_23_8_tie_to_pos_inf b_ceil=number;
//   int_23_8_native_rounding b_floor=number;
//   int_23_8_nearest_rounding b_nearst=number;
//   int_23_8_neg_inf_rounding b_negative=number;
//   std::cout << "number = "<< number*256<< std::endl;
//   std::cout << "tie_to_pos_inf = "<< to_rep(b_ceil)<< std::endl;
//   std::cout << "native_rounding = "<<to_rep(b_floor)<< std::endl;
//   std::cout << "nearest_rounding_tag = "<<to_rep(b_nearst)<< std::endl;
//   std::cout << "neg_inf_rounding_tag = "<<to_rep(b_negative)<< std::endl;
// }

// std::vector<float> calc_Quintic(int32_t a0, int32_t a1, int32_t a2, int32_t a3, int32_t a4, int32_t a5, int32_t t)
// {
//   float 
//   int32_t third_derivation = 24*a4+60*a5*t;
//   third_derivation = 6*a3+t*third_derivation;
//   int32_t second_derivation = 12*a4+20*a5*t;
//   second_derivation = 6*a3+t*second_derivation;
//   second_derivation = 2*a2+t*ans;
//   int32_t first_derivation = 4*a4+5*a5*t;
//   first_derivation = 3*a3+t*first_derivation;
//   first_derivation = 2*a2+t*first_derivation;
//   first_derivation = a1+t*first_derivation;
//   int32_t position = a4+t*a5;
//   position = a3+t*position;
//   position = a2+t*position;
//   position = a1+t*position;
//   position = a0+t*position;
//   std::vector<float> output;
//   float first,second,third, pos;
//   third =std::round(static_cast<float>(third_derivation)/256.0)
//   output.push_back(third);
//   if(third_derviation>=0)
//   {
//     second = std::ceil(second_derivation/256.0);
//   }
//   else
//   (
//     second = std::floor(second_derivation/256.0);
//   )
//   if(second_derivation>=0)
//   {
//     first = std::ceil(first_derivation/256.0);
//   }
//   else
//   (
//     first = std::floor(first_derivation/256.0);
//   )
//   if(first_derivation>=0)
//   {
//     pos = std::ceil(position/256.0);
//   }
//   else
//   (
//     pos = std::floor(position/256.0);
//   )
//   output.push_back(second);
//   output.push_back(first);
//   output.push_back(pos);
// }

#include <iostream>
int main()
{
  // float t=1.125;
  // calculate(a);
  // calculate(1.78);
  // calculate(1.43);
  // calculate(1.45);
  // calculate(1.5);
  // calculate(1.6);
  // calculate(1.7);
  // calculate(-56.43);
  // calculate(-56.89);
  // calculate(-675.135);

  // std::cout << y << std::endl;
    // int_11_20 y = a0*x;
  // std::cout << to_rep(y)<<std::endl;
  // y=y*x;
  // std::cout << to_rep(y)<<std::endl;

//   int_12_3 x=1.125;
//   std::cout << "x=1.125, int = "<<to_rep(x)<<std::endl;
//   int_21_10 a0=23.3;
//   std::cout << "23.3, int = "<< to_rep(a0)<<std::endl;
//   int_21_10 y0=a0*x*x*x*x*x;
//   std::cout << "a0*x*x*x*x*x=23.3*1.125, int = "<< to_rep(y0)<<std::endl;
//   int_21_10 y1=a0*x;
//   y1*=x;
//   y1*=x;
//   y1*=x;
//   y1*=x;
//   std::cout << "a0*x*x*x*x*x=23.3*1.125, int = "<< to_rep(y1)<<std::endl;
    float a=-3.2;
    std::cout << a << " = round::"<< std::round(a)<< std::endl;
    a=-3.9;
    std::cout << a << " = round::"<< std::round(a)<< std::endl;
    a=3.2;
    std::cout << a << " = round::"<< std::round(a)<< std::endl;
    a=3.9;
    std::cout << a << " = round::"<< std::round(a)<< std::endl;
}