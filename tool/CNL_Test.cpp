/*
How to compile this file:
clang++ -isystem /home/wenguang/installed/cnl/include -std=c++20 CNL_Test.cpp -o CNL_Test
clang++ -isystem /home/wenguang/installed/cnl_Debug/include -std=c++20 CNL_Test.cpp ../src/math/cordic.cpp -o CNL_Test
*/
#include "cnl/all.h"
// #include "/home/wenguang/workplace/frenet_optimal_trajectory_planner/src/math/cordic.h"
#include <cmath>
#include <iostream>
#include <limits>
using cnl::power;
using cnl::scaled_integer;

int main() {
    // cnl::scaled_integer<int16_t, cnl::power<-13>> a;
    // cnl::scaled_integer<int16_t, cnl::power<-13>> b;
    // cnl::scaled_integer<int16_t, cnl::power<-13>> ans;
    // std::cout << std::numeric_limits<
    //                  cnl::scaled_integer<int16_t, cnl::power<-12>>>::max()
    //           << std::endl;
    // a = 2.512;
    cnl::scaled_integer<int16_t, cnl::power<-10>> a=6;
    cnl::scaled_integer<int16_t, cnl::power<-10>> b=2*3.14;
    // cnl::scaled_integer<int16_t, cnl::power<-11>> c=cnl::quotient(a,b);
    cnl::scaled_integer<int16_t, cnl::power<-11>> d = a/b;
    cnl::scaled_integer<int16_t, cnl::power<-11>> e = a-d;
    // a = NAN;
    // std::cout << c << std::endl;
    std::cout << d << std::endl;
    std::cout << e << std::endl;
    
    // std::cout << e << std::endl;
    // a=30.0/180.0*M_PI;
    // ans=cordic_sin(a);
    // std::cout << "sin(" <<  a<< ") = " << ans << std::endl;
    // ans=cordic_cos(a);
    // std::cout << "cos(" <<  a<< ") = " << ans << std::endl;
    // a=45.0/180.0*M_PI;
    // ans=cordic_sin(a);
    // std::cout << "sin(" <<  a<< ") = " << ans << std::endl;
    // ans=cordic_cos(a);
    // std::cout << "cos(" <<  a<< ") = " << ans << std::endl;
    // a=90.0/180.0*M_PI;
    // ans=cordic_sin(a);
    // std::cout << "sin(" <<  a<< ") = " << ans << std::endl;
    // ans=cordic_cos(a);
    // std::cout << "cos(" <<  a<< ") = " << ans << std::endl;
}