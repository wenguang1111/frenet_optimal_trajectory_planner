#include "QuinticPolynomial_fx.h"
#include <cmath>

QuinticPolynomial_fx::QuinticPolynomial_fx(fp_type xs, fp_type vxs, fp_type axs,
        fp_type xe, fp_type vxe, fp_type axe, fp_type t):
        a0(xs), a1(vxs) {
     a2 = axs>>1;

    //Gaussian elimination
    fp_type inverse = cnl::quotient(1,t);
    a3 = 10*(xe-xs);
    a3*=inverse;
    a3+=-6*vxs-4*vxe;
    a3*=inverse;
    a3+=-3*axs+2*axe;
    a3*=inverse;
    a4=15*(xs-xe);
    a4*=inverse;
    a4+=7*vxe+8*vxs;
    a4*=inverse;
    a4+=4.5*axs-4*axe;
    a4*=inverse;
    a4*=inverse;
    a5=6*(xe-xs);
    a5*=inverse;
    a5+=-3*vxe-3*vxs;
    a5*=inverse;
    a5+=2*axe-2*axs;
    a5*=inverse;
    a5*=inverse;
    a5*=inverse;
}

fp_type QuinticPolynomial_fx::calc_point_fx(fp_type t) {
    fp_type ans = a4+t*a5;
    ans = a3+t*ans;
    ans = a2+t*ans;
    ans = a1+t*ans;
    ans = a0+t*ans;
    return ans;
}

fp_type QuinticPolynomial_fx::calc_first_derivative_fx(fp_type t) {
    fp_type ans = 4*a4+5*a5*t;
    ans = 3*a3+t*ans;
    ans = 2*a2+t*ans;
    ans = a1+t*ans;
    return ans;
}

fp_type QuinticPolynomial_fx::calc_second_derivative_fx(fp_type t) {
    fp_type ans = 12*a4+20*a5*t;
    ans = 6*a3+t*ans;
    ans = 2*a2+t*ans;
    return ans;
}

fp_type QuinticPolynomial_fx::calc_third_derivative_fx(fp_type t) {
    fp_type ans = 24*a4+60*a5*t;
    ans = 6*a3+t*ans;
    return ans;
}

fp_type QuinticPolynomial_fx::getA0()
{
    return a0;
}

fp_type QuinticPolynomial_fx::getA1()
{
    return a1;
}

fp_type QuinticPolynomial_fx::getA2()
{
    return a2;
}

fp_type QuinticPolynomial_fx::getA3()
{
    return a3;
}

fp_type QuinticPolynomial_fx::getA4()
{
    return a4;
}

fp_type QuinticPolynomial_fx::getA5()
{
    return a5;
}