/*
How to compile this file:
clang++ -isystem /home/wenguang/installed/cnl/include -std=c++20 CNL_Test.cpp -o
CNL_Test
*/
#include "cnl/all.h"
#include <cmath>
#include <iostream>
#include <limits>
using cnl::power;
using cnl::scaled_integer;

int main() {
    cnl::scaled_integer<int16_t, cnl::power<-12>> a,b,c,d;
    std::cout << std::numeric_limits<
                     cnl::scaled_integer<int16_t, cnl::power<-12>>>::max()
              << std::endl;
    
    a = 2.3;
    std::cout << "size of a="<< sizeof(a)<< std::endl;
    b=4;
    c = b/a;
    d=b%a;
    std::cout << "c=" <<c<< std::endl;
    std::cout << "size of c="<< sizeof(c)<<std::endl;
        std::cout << "d=" <<d<< std::endl;
    std::cout << "size of d="<< sizeof(d)<<std::endl;

    // auto b = cnl::scaled_integer<uint16_t, cnl::power<-3>>{0};
    // b = 23.5212;

    // std::vector<cnl::scaled_integer<int16_t, cnl::power<-8>>> c;
    // c.push_back(a);
    // c.push_back(b);
    // // std::cout << c[0] << " represented by " << to_rep(c[0]) << std::endl;
    // // std::cout << c[1] << " represented by " << to_rep(c[1]) << std::endl;
    // // std::cout << (double)a;

    // cnl::scaled_integer<int16_t, cnl::power<-12>> x;
    // x = 1.001;

}