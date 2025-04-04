#include "QuinticPolynomial.h"
#include "utils.h"
#ifdef USE_RECORDER
    #include "tool/recorder.h"
#endif

#include <cmath>

QuinticPolynomial::QuinticPolynomial(fixp_c_d xs, fixp_c_d vxs, fixp_c_d axs,
        fixp_c_d xe, fixp_c_d vxe, fixp_c_d axe, fixp_t t):
        a0(xs), a1(vxs) {
          a2 = axs>>1;

    //Gaussian elimination
    fixp_t inverse = cnl::quotient(1,t);
    // #ifdef USE_RECORDER
    //     Recorder::getInstance()->saveData<float>("QuinticPolynomial_fx::inverse", static_cast<float>(inverse));
    // #endif
    a3 = 10*(xe-xs);
    // #ifdef USE_RECORDER
    //     Recorder::getInstance()->saveData<float>("QuinticPolynomial_fx::a3", static_cast<float>(a3));
    // #endif
    a3*=inverse;
    // #ifdef USE_RECORDER
    //     Recorder::getInstance()->saveData<float>("QuinticPolynomial_fx::a3", static_cast<float>(a3));
    // #endif
    a3+=-6*vxs-4*vxe;
    // #ifdef USE_RECORDER
    //     Recorder::getInstance()->saveData<float>("QuinticPolynomial_fx::a3", static_cast<float>(a3));
    // #endif
    a3*=inverse;
    // #ifdef USE_RECORDER
    //     Recorder::getInstance()->saveData<float>("QuinticPolynomial_fx::a3", static_cast<float>(a3));
    // #endif
    a3+=-3*axs+2*axe;
    // #ifdef USE_RECORDER
    //     Recorder::getInstance()->saveData<float>("QuinticPolynomial_fx::a3", static_cast<float>(a3));
    // #endif
    a3*=inverse;
    // #ifdef USE_RECORDER
    //     Recorder::getInstance()->saveData<float>("QuinticPolynomial_fx::a3", static_cast<float>(a3));
    // #endif
    a4=15*(xs-xe);
    // #ifdef USE_RECORDER
    //     Recorder::getInstance()->saveData<float>("QuinticPolynomial_fx::a4", static_cast<float>(a4));
    // #endif
    a4*=inverse;
    // #ifdef USE_RECORDER
    //     Recorder::getInstance()->saveData<float>("QuinticPolynomial_fx::a4", static_cast<float>(a4));
    // #endif
    a4+=7*vxe+8*vxs;
    // #ifdef USE_RECORDER
    //     Recorder::getInstance()->saveData<float>("QuinticPolynomial_fx::a4", static_cast<float>(a4));
    // #endif
    a4*=inverse;
    // #ifdef USE_RECORDER
    //     Recorder::getInstance()->saveData<float>("QuinticPolynomial_fx::a4", static_cast<float>(a4));
    // #endif
    a4+=4.5*axs-4*axe;
    // #ifdef USE_RECORDER
    //     Recorder::getInstance()->saveData<float>("QuinticPolynomial_fx::a4", static_cast<float>(a4));
    // #endif
    a4*=inverse;
    // #ifdef USE_RECORDER
    //     Recorder::getInstance()->saveData<float>("QuinticPolynomial_fx::a4", static_cast<float>(a4));
    // #endif
    a4*=inverse;
    // #ifdef USE_RECORDER
    //     Recorder::getInstance()->saveData<float>("QuinticPolynomial_fx::a4", static_cast<float>(a4));
    // #endif
    a5=6*(xe-xs);
    // #ifdef USE_RECORDER
    //     Recorder::getInstance()->saveData<float>("QuinticPolynomial_fx::a5", static_cast<float>(a5));
    // #endif
    a5*=inverse;
    // #ifdef USE_RECORDER
    //     Recorder::getInstance()->saveData<float>("QuinticPolynomial_fx::a5", static_cast<float>(a5));
    // #endif
    a5+=-3*vxe-3*vxs;
    // #ifdef USE_RECORDER
    //     Recorder::getInstance()->saveData<float>("QuinticPolynomial_fx::a5", static_cast<float>(a5));
    // #endif
    a5*=inverse;
    // #ifdef USE_RECORDER
    //     Recorder::getInstance()->saveData<float>("QuinticPolynomial_fx::a5", static_cast<float>(a5));
    // #endif
    a5+=2*axe-2*axs;
    // #ifdef USE_RECORDER
    //     Recorder::getInstance()->saveData<float>("QuinticPolynomial_fx::a5", static_cast<float>(a5));
    // #endif
    a5*=inverse;
    // #ifdef USE_RECORDER
    //     Recorder::getInstance()->saveData<float>("QuinticPolynomial_fx::a5", static_cast<float>(a5));
    // #endif
    a5*=inverse;
    // #ifdef USE_RECORDER
    //     Recorder::getInstance()->saveData<float>("QuinticPolynomial_fx::a5", static_cast<float>(a5));
    // #endif
    a5*=inverse;
    // #ifdef USE_RECORDER
    //     Recorder::getInstance()->saveData<float>("QuinticPolynomial_fx::a5", static_cast<float>(a5));
    // #endif
}

fixp_d QuinticPolynomial::calc_point(fixp_t t) {
    // return a0 + a1 * t + a2 * pow_2<fixp_time_2>(t) + a3 * pow_3<fixp_time_3>(t) +
    // a4 * pow_4<fixp_time_4>(t) + a5 * pow_5<fixp_time_5>(t);
    // return a0+t*(a1+t*(a2+t*(a3+t*(a4+t*a5))));
    fixp_intermediate ans = a4+t*a5;
    ans = a3+t*ans;
    ans = a2+t*ans;
    ans = a1+t*ans;
    ans = a0+t*ans;
    return ans;
}

fixp_d_d QuinticPolynomial::calc_first_derivative(fixp_t t) {
    // return a1 + 2 * a2 * t + 3 * a3 * pow_2<fixp_time_2>(t) + 4 * a4 * pow_3<fixp_time_3>(t) +
    // 5 * a5 * pow_4<fixp_time_4>(t);
    // return a1+t*(2*a2+t*(3*a3+t*(4*a4+5*a5*t)));
    fixp_intermediate ans = 4.0*a4+5.0*a5*t;
    ans = 3*a3+t*ans;
    ans = 2*a2+t*ans;
    ans = a1+t*ans;
    return ans;
}

fixp_d_dd QuinticPolynomial::calc_second_derivative(fixp_t t) {
    // return 2 * a2 + 6 * a3 * t + 12 * a4 * pow_2<fixp_time_2>(t) + 20 * a5 * pow_3<fixp_time_3>(t);
    fixp_intermediate ans = 12.0*a4+20.0*a5*t;
    ans = 6*a3+t*ans;
    ans = 2*a2+t*ans;
    return ans;
}

fixp_d_ddd QuinticPolynomial::calc_third_derivative(fixp_t t) {
    // return 6 * a3 + 24 * a4 * t + 60 * a5 * pow_2<fixp_time_2>(t);
    fixp_intermediate ans = 24.0*a4+60.0*a5*t;
    ans = 6*a3+t*ans;
    return ans;
}