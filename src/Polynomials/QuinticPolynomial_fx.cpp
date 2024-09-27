#include "QuinticPolynomial_fx.h"
#ifdef USE_RECORDER
    #include "tool/recorder.h"
#endif

#include <cmath>

QuinticPolynomial_fx::QuinticPolynomial_fx(fp_type xs, fp_type vxs, fp_type axs,
        fp_type xe, fp_type vxe, fp_type axe, fp_type t):
        a0(xs), a1(vxs) {
     a2 = axs>>1;

    //Gaussian elimination
    fp_type inverse = cnl::quotient(1,t);
    #ifdef USE_RECORDER
        Recorder::getInstance()->saveData<float>("QuinticPolynomial_fx::inverse", static_cast<float>(inverse));
    #endif
    a3 = 10*(xe-xs);
    #ifdef USE_RECORDER
        Recorder::getInstance()->saveData<float>("QuinticPolynomial_fx::a3", static_cast<float>(a3));
    #endif
    a3*=inverse;
    #ifdef USE_RECORDER
        Recorder::getInstance()->saveData<float>("QuinticPolynomial_fx::a3", static_cast<float>(a3));
    #endif
    a3+=-6*vxs-4*vxe;
    #ifdef USE_RECORDER
        Recorder::getInstance()->saveData<float>("QuinticPolynomial_fx::a3", static_cast<float>(a3));
    #endif
    a3*=inverse;
    #ifdef USE_RECORDER
        Recorder::getInstance()->saveData<float>("QuinticPolynomial_fx::a3", static_cast<float>(a3));
    #endif
    a3+=-3*axs+2*axe;
    #ifdef USE_RECORDER
        Recorder::getInstance()->saveData<float>("QuinticPolynomial_fx::a3", static_cast<float>(a3));
    #endif
    a3*=inverse;
    #ifdef USE_RECORDER
        Recorder::getInstance()->saveData<float>("QuinticPolynomial_fx::a3", static_cast<float>(a3));
    #endif
    a4=15*(xs-xe);
    #ifdef USE_RECORDER
        Recorder::getInstance()->saveData<float>("QuinticPolynomial_fx::a4", static_cast<float>(a4));
    #endif
    a4*=inverse;
    #ifdef USE_RECORDER
        Recorder::getInstance()->saveData<float>("QuinticPolynomial_fx::a4", static_cast<float>(a4));
    #endif
    a4+=7*vxe+8*vxs;
    #ifdef USE_RECORDER
        Recorder::getInstance()->saveData<float>("QuinticPolynomial_fx::a4", static_cast<float>(a4));
    #endif
    a4*=inverse;
    #ifdef USE_RECORDER
        Recorder::getInstance()->saveData<float>("QuinticPolynomial_fx::a4", static_cast<float>(a4));
    #endif
    a4+=4.5*axs-4*axe;
    #ifdef USE_RECORDER
        Recorder::getInstance()->saveData<float>("QuinticPolynomial_fx::a4", static_cast<float>(a4));
    #endif
    a4*=inverse;
    #ifdef USE_RECORDER
        Recorder::getInstance()->saveData<float>("QuinticPolynomial_fx::a4", static_cast<float>(a4));
    #endif
    a4*=inverse;
    #ifdef USE_RECORDER
        Recorder::getInstance()->saveData<float>("QuinticPolynomial_fx::a4", static_cast<float>(a4));
    #endif
    a5=6*(xe-xs);
    #ifdef USE_RECORDER
        Recorder::getInstance()->saveData<float>("QuinticPolynomial_fx::a5", static_cast<float>(a5));
    #endif
    a5*=inverse;
    #ifdef USE_RECORDER
        Recorder::getInstance()->saveData<float>("QuinticPolynomial_fx::a5", static_cast<float>(a5));
    #endif
    a5+=-3*vxe-3*vxs;
    #ifdef USE_RECORDER
        Recorder::getInstance()->saveData<float>("QuinticPolynomial_fx::a5", static_cast<float>(a5));
    #endif
    a5*=inverse;
    #ifdef USE_RECORDER
        Recorder::getInstance()->saveData<float>("QuinticPolynomial_fx::a5", static_cast<float>(a5));
    #endif
    a5+=2*axe-2*axs;
    #ifdef USE_RECORDER
        Recorder::getInstance()->saveData<float>("QuinticPolynomial_fx::a5", static_cast<float>(a5));
    #endif
    a5*=inverse;
    #ifdef USE_RECORDER
        Recorder::getInstance()->saveData<float>("QuinticPolynomial_fx::a5", static_cast<float>(a5));
    #endif
    a5*=inverse;
    #ifdef USE_RECORDER
        Recorder::getInstance()->saveData<float>("QuinticPolynomial_fx::a5", static_cast<float>(a5));
    #endif
    a5*=inverse;
    #ifdef USE_RECORDER
        Recorder::getInstance()->saveData<float>("QuinticPolynomial_fx::a5", static_cast<float>(a5));
    #endif
    // #ifdef USE_RECORDER
    //     Recorder::getInstance()->saveData<float>("k",k);
    // #endif 
}

fp_type QuinticPolynomial_fx::calc_point_fx(fp_type t) {
    fp_type ans = a4+t*a5;
    #ifdef USE_RECORDER
        Recorder::getInstance()->saveData<float>("QuinticPolynomial_fx::calc_point_fx::ans", static_cast<float>(ans));
    #endif
    ans = a3+t*ans;
    #ifdef USE_RECORDER
        Recorder::getInstance()->saveData<float>("QuinticPolynomial_fx::calc_point_fx::ans", static_cast<float>(ans));
    #endif
    ans = a2+t*ans;
    #ifdef USE_RECORDER
        Recorder::getInstance()->saveData<float>("QuinticPolynomial_fx::calc_point_fx::ans", static_cast<float>(ans));
    #endif
    ans = a1+t*ans;
    #ifdef USE_RECORDER
        Recorder::getInstance()->saveData<float>("QuinticPolynomial_fx::calc_point_fx::ans", static_cast<float>(ans));
    #endif
    ans = a0+t*ans;
    #ifdef USE_RECORDER
        Recorder::getInstance()->saveData<float>("QuinticPolynomial_fx::calc_point_fx::ans", static_cast<float>(ans));
    #endif
    return ans;
}

fp_type QuinticPolynomial_fx::calc_first_derivative_fx(fp_type t) {
    fp_type ans = 4*a4+5*a5*t;
    #ifdef USE_RECORDER
        Recorder::getInstance()->saveData<float>("QuinticPolynomial_fx::calc_first_derivative_fx::ans", static_cast<float>(ans));
    #endif
    ans = 3*a3+t*ans;
    #ifdef USE_RECORDER
        Recorder::getInstance()->saveData<float>("QuinticPolynomial_fx::calc_first_derivative_fx::ans", static_cast<float>(ans));
    #endif
    ans = 2*a2+t*ans;
    #ifdef USE_RECORDER
        Recorder::getInstance()->saveData<float>("QuinticPolynomial_fx::calc_first_derivative_fx::ans", static_cast<float>(ans));
    #endif
    ans = a1+t*ans;
    #ifdef USE_RECORDER
        Recorder::getInstance()->saveData<float>("QuinticPolynomial_fx::calc_first_derivative_fx::ans", static_cast<float>(ans));
    #endif
    return ans;
}

fp_type QuinticPolynomial_fx::calc_second_derivative_fx(fp_type t) {
    fp_type ans = 12*a4+20*a5*t;
    #ifdef USE_RECORDER
        Recorder::getInstance()->saveData<float>("QuinticPolynomial_fx::calc_second_derivative_fx::ans", static_cast<float>(ans));
    #endif
    ans = 6*a3+t*ans;
    #ifdef USE_RECORDER
        Recorder::getInstance()->saveData<float>("QuinticPolynomial_fx::calc_second_derivative_fx::ans", static_cast<float>(ans));
    #endif
    ans = 2*a2+t*ans;
    #ifdef USE_RECORDER
        Recorder::getInstance()->saveData<float>("QuinticPolynomial_fx::calc_second_derivative_fx::ans", static_cast<float>(ans));
    #endif
    return ans;
}

fp_type QuinticPolynomial_fx::calc_third_derivative_fx(fp_type t) {
    fp_type ans = 24*a4+60*a5*t;
    #ifdef USE_RECORDER
        Recorder::getInstance()->saveData<float>("QuinticPolynomial_fx::calc_third_derivative_fx::ans", static_cast<float>(ans));
    #endif
    ans = 6*a3+t*ans;
    #ifdef USE_RECORDER
        Recorder::getInstance()->saveData<float>("QuinticPolynomial_fx::calc_third_derivative_fx::ans", static_cast<float>(ans));
    #endif
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