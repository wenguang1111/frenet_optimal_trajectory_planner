/*
How to compile this file:
clang++ -isystem /home/wenguang/installed/cnl/include -std=c++20 CNL_Test.cpp -o CNL_Test
clang++ -isystem /home/wenguang/installed/cnl_Debug/include -std=c++20 CNL_Test.cpp ../src/math/cordic.cpp -o CNL_Test
*/
#include "cnl/all.h"
// #include "/home/wenguang/workplace/frenet_optimal_trajectory_planner/src/math/cordic.h"
// #include <cmath>
// #include <iostream>
// #include <limits>
// using cnl::power;
using cnl::scaled_integer;
// typedef cnl::scaled_integer<uint16_t, cnl::power<-14>> uint_2_14;
// typedef cnl::scaled_integer<ini16_t, cnl::power<-13>> int_2_13;
typedef cnl::scaled_integer<uint64_t, cnl::power<-4>> uint_9_7;

extern "C"
{
  uint_9_7 calculate(uint_9_7 xs, uint_9_7 vxs, uint_9_7 axs,
        uint_9_7 xe, uint_9_7 vxe, uint_9_7 axe, uint_9_7 t)
  { 
    uint_9_7 b3 = (-7*axs+2*axe+(-6*vxs-4*vxe+10*(xe-xs)/t)/t)/t;
    return b3;
  }
}