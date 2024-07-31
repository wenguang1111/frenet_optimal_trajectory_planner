#ifndef FRENET_OPTIMAL_TRAJECTORY_QUINTICPOLYNOMIAL_FX_H
#define FRENET_OPTIMAL_TRAJECTORY_QUINTICPOLYNOMIAL_FX_H

#include "cnl/all.h"

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

class QuinticPolynomial_fx {
public:
    QuinticPolynomial_fx() = default;
    QuinticPolynomial_fx(fp_type xs, fp_type vxs, fp_type axs, fp_type xe,
                      fp_type vxe, fp_type axe, fp_type t);
    fp_type calc_point_fx(fp_type t);
    fp_type calc_first_derivative_fx(fp_type t);
    fp_type calc_second_derivative_fx(fp_type t);
    fp_type calc_third_derivative_fx(fp_type t);
    fp_type getA0();
    fp_type getA1();
    fp_type getA2();
    fp_type getA3();
    fp_type getA4();
    fp_type getA5();
private:
    fp_type a0, a1, a2, a3, a4, a5;
};

#endif //FRENET_OPTIMAL_TRAJECTORY_QUINTICPOLYNOMIAL_FX_H
