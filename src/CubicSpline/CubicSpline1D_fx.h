#ifndef FRENET_OPTIMAL_TRAJECTORY_CUBICSPLINE1D_FX_H
#define FRENET_OPTIMAL_TRAJECTORY_CUBICSPLINE1D_FX_H

#include "cnl/all.h"
#include <vector>

#define fp_type int_64_25
typedef cnl::scaled_integer<int64_t, cnl::power<-25>>  int_64_25; 
typedef cnl::scaled_integer<int64_t, cnl::power<-22>>  int_64_22; 
typedef cnl::scaled_integer<int64_t, cnl::power<-21>>  int_64_21; 
typedef cnl::scaled_integer<int64_t, cnl::power<-20>>  int_64_20; 
typedef cnl::scaled_integer<int64_t, cnl::power<-16>>  int_64_16;
typedef cnl::scaled_integer<int64_t, cnl::power<-15>>  int_64_15; 
typedef cnl::scaled_integer<int64_t, cnl::power<-14>>  int_64_14;
typedef cnl::scaled_integer<int64_t, cnl::power<-13>>  int_64_13;
typedef cnl::scaled_integer<int64_t, cnl::power<-12>>  int_64_12;
typedef cnl::scaled_integer<int64_t, cnl::power<-11>>  int_64_11;
typedef cnl::scaled_integer<int64_t, cnl::power<-10>>  int_64_10;
typedef cnl::scaled_integer<int64_t, cnl::power<-9>>  int_64_9;
typedef cnl::scaled_integer<int64_t, cnl::power<-8>>  int_64_8;
typedef cnl::scaled_integer<int64_t, cnl::power<-7>>  int_64_7;
typedef cnl::scaled_integer<int64_t, cnl::power<-6>>  int_64_6;
typedef cnl::scaled_integer<int16_t, cnl::power<-13>> int_2_13;

// 1-dimensional cubic spline class.
// For technical details see: http://mathworld.wolfram.com/CubicSpline.html
class CubicSpline1D_fx {
public:
    int nx;
    CubicSpline1D_fx();
    CubicSpline1D_fx(const std::vector<fp_type>& v1, const std::vector<fp_type>& v2);
    fp_type calc_der0(fp_type t);
    fp_type calc_der1(fp_type t);
    bool isValidPath();
private:
    std::vector<fp_type> x;
    std::vector<fp_type> y;
    std::vector<fp_type> a;
    std::vector<fp_type> b;
    std::vector<fp_type> c;
    std::vector<fp_type> d;
    std::vector<fp_type> tridionalmatrix_a;
    std::vector<fp_type> tridionalmatrix_b;
    std::vector<fp_type> tridionalmatrix_c;
    std::vector<fp_type> tridionalmatrix_d;
    int search_index(fp_type t);
    void assignValue(std::vector<fp_type> &TM_a, std::vector<fp_type> &TM_b, 
            std::vector<fp_type> &TM_c, std::vector<fp_type> &TM_d, std::vector<fp_type> &deltas);
    bool validPath;
};

#endif //FRENET_OPTIMAL_TRAJECTORY_CUBICSPLINE1D_FX_H
