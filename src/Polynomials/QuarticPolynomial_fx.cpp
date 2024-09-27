#include "QuarticPolynomial_fx.h"
#include "utils.h"
#ifdef USE_RECORDER
    #include "tool/recorder.h"
#endif

#include <cmath>

QuarticPolynomial_fx::QuarticPolynomial_fx(fp_type xs, fp_type vxs, fp_type axs,
        fp_type vxe, fp_type axe, fp_type t):
        a0(xs), a1(vxs) {
    a2 = axs>>1;
    #ifdef USE_RECORDER
        Recorder::getInstance()->saveData<float>("QuarticPolynomial::a2", static_cast<float>(a2));
    #endif
    fp_type inverse = cnl::quotient(1,t);
    #ifdef USE_RECORDER
        Recorder::getInstance()->saveData<float>("QuarticPolynomial::a2", static_cast<float>(inverse));
    #endif
    a3 = (vxe-vxs)*inverse;
    #ifdef USE_RECORDER
        Recorder::getInstance()->saveData<float>("QuarticPolynomial::a2", static_cast<float>(a3));
    #endif
    a3 = a3 + cnl::quotient(-axe-2*axs,3);
    #ifdef USE_RECORDER
        Recorder::getInstance()->saveData<float>("QuarticPolynomial::a2", static_cast<float>(a3));
    #endif
    a3 *= inverse;
    #ifdef USE_RECORDER
        Recorder::getInstance()->saveData<float>("QuarticPolynomial::a2", static_cast<float>(a3));
    #endif
    a4 = cnl::quotient(vxs-vxe,2);
    #ifdef USE_RECORDER
        Recorder::getInstance()->saveData<float>("QuarticPolynomial::a2", static_cast<float>(a4));
    #endif
    a4 *= inverse;
    #ifdef USE_RECORDER
        Recorder::getInstance()->saveData<float>("QuarticPolynomial::a2", static_cast<float>(a4));
    #endif
    a4 +=  cnl::quotient(axe+axs,4);
    #ifdef USE_RECORDER
        Recorder::getInstance()->saveData<float>("QuarticPolynomial::a2", static_cast<float>(a4));
    #endif
    a4*=inverse;
    #ifdef USE_RECORDER
        Recorder::getInstance()->saveData<float>("QuarticPolynomial::a2", static_cast<float>(a4));
    #endif
    a4*=inverse;
    #ifdef USE_RECORDER
        Recorder::getInstance()->saveData<float>("QuarticPolynomial::a2", static_cast<float>(a4));
    #endif
}

fp_type QuarticPolynomial_fx::calc_point(fp_type t) {
    fp_type ans=a3+t*a4;
    #ifdef USE_RECORDER
        Recorder::getInstance()->saveData<float>("QuarticPolynomial::calc_point::ans", static_cast<float>(ans));
    #endif
    ans=ans*t+a2;
    #ifdef USE_RECORDER
        Recorder::getInstance()->saveData<float>("QuarticPolynomial::calc_point::ans", static_cast<float>(ans));
    #endif
    ans=ans*t+a1;
    #ifdef USE_RECORDER
        Recorder::getInstance()->saveData<float>("QuarticPolynomial::calc_point::ans", static_cast<float>(ans));
    #endif
    ans=ans*t+a0;
    #ifdef USE_RECORDER
        Recorder::getInstance()->saveData<float>("QuarticPolynomial::calc_point::ans", static_cast<float>(ans));
    #endif
    return ans;
}

fp_type QuarticPolynomial_fx::calc_first_derivative(fp_type t) {
    fp_type ans = 3*a3+4*a4*t;
    #ifdef USE_RECORDER
        Recorder::getInstance()->saveData<float>("QuarticPolynomial::calc_first_derivative::ans", static_cast<float>(ans));
    #endif
    ans = 2*a2+t*ans;
    #ifdef USE_RECORDER
        Recorder::getInstance()->saveData<float>("QuarticPolynomial::calc_first_derivative::ans", static_cast<float>(ans));
    #endif
    ans = a1+t*ans;
    #ifdef USE_RECORDER
        Recorder::getInstance()->saveData<float>("QuarticPolynomial::calc_first_derivative::ans", static_cast<float>(ans));
    #endif
    return ans;
}

fp_type QuarticPolynomial_fx::calc_second_derivative(fp_type t) {
    fp_type ans = 6*a3+12*a4*t;
    #ifdef USE_RECORDER
        Recorder::getInstance()->saveData<float>("QuarticPolynomial::calc_second_derivative::ans", static_cast<float>(ans));
    #endif
    ans = 2*a2+t*ans;
    #ifdef USE_RECORDER
        Recorder::getInstance()->saveData<float>("QuarticPolynomial::calc_second_derivative::ans", static_cast<float>(ans));
    #endif
    return ans;
}

fp_type QuarticPolynomial_fx::calc_third_derivative(fp_type t) {
    return 6 * a3 + 24 * a4 * t;
}