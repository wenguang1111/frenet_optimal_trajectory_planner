#include "QuarticPolynomial.h"
#include "utils.h"
#ifdef USE_RECORDER
    #include "tool/recorder.h"
#endif

#include <cmath>

QuarticPolynomial::QuarticPolynomial(fixp_s xs, fixp_s_d vxs, fixp_s_dd axs,
        fixp_s_d vxe, fixp_s_dd axe, fixp_maxt t):
        a0(xs), a1(vxs) {
        a2 = axs>>1;
    #ifdef USE_RECORDER
        Recorder::getInstance()->saveData<float>("QuarticPolynomial::a2", static_cast<float>(a2));
    #endif
    fixp_30_33 inverse = cnl::quotient(1,t);
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

fixp_s QuarticPolynomial::calc_point(fixp_maxt t) {
    // a0 + a1 * t + a2 * pow_2<fixp_time_2>(t) + a3 * pow_3<fixp_time_3>(t) + a4 * pow_4<fixp_time_4>(t);
    // a0+t*(a1+t*(a2+t*(a3+t*a4)));
    fixp_s ans=a3+t*a4;
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

fixp_s_d QuarticPolynomial::calc_first_derivative(fixp_maxt t) {
    // return a1 + 2 * a2 * t + 3 * a3 * pow_2<fixp_time_2>(t) + 4 * a4 * pow_3<fixp_time_3>(t);
    // return a1+t*(2*a2+t*(3*a3+4*a4*t));
    fixp_maxt ans = 3*a3+4*a4*t;
    ans = 2*a2+t*ans;
    ans = a1+t*ans;
    return ans;
}

fixp_s_dd QuarticPolynomial::calc_second_derivative(fixp_maxt t) {
    // return 2 * a2 + 6 * a3 * t + 12 * a4 * pow_2<fixp_time_2>(t);
    fixp_maxt ans = 6*a3+12*a4*t;
    ans = 2*a2+t*ans;
    return ans;
}

fixp_s_ddd QuarticPolynomial::calc_third_derivative(fixp_maxt t) {
    return 6 * a3 + 24 * a4 * t;
}