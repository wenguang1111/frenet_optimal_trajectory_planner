#include "QuinticPolynomial.h"
#include "utils.h"
#ifdef USE_RECORDER
    #include "tool/recorder.h"
#endif

#include <cmath>

QuinticPolynomial::QuinticPolynomial(fixp_c_d xs, fixp_c_d vxs, fixp_c_d axs,
        fixp_c_d xe, fixp_c_d vxe, fixp_c_d axe, fixp_maxt t):
        a0(xs), a1(vxs) {
     a2 = axs>>1;

    //Gaussian elimination
    fixp_quinticpolynomial_K K;
    fixp_quinticpolynomial_K0 K0;
    fixp_quinticpolynomial_K1 K1;
    fixp_quinticpolynomial_K2 K2;

    K=a0+a1*t+a2*pow_2<fixp_quinticpolynomial_K>(t);
    K0=xe-K;
    K1=vxe-2*a2*t-a1;
    K2=axe-2*a2;
    fixp_maxt inverse = cnl::quotient(1,t);
    // static_assert(is_same<decltype(asd), elastic_integer<13, int8_t>>::value);

    // a3=fixp_quinticpolynomial_A0_A5(10*cnl::quotient(K0,pow_3<fixp_time_3>(t))) - 
    //     fixp_quinticpolynomial_A0_A5(4*cnl::quotient(K1,pow_2<fixp_time_2>(t))) + 
    //     fixp_quinticpolynomial_A0_A5(2*cnl::quotient(K2,t));
    // a4=fixp_quinticpolynomial_A0_A5(7*cnl::quotient(K1,pow_3<fixp_time_3>(t))) - 
    //     fixp_quinticpolynomial_A0_A5(15*cnl::quotient(K0,pow_4<fixp_time_4>(t))) - 
    //     fixp_quinticpolynomial_A0_A5(4*cnl::quotient(K2,pow_2<fixp_time_2>(t)));
    // a5=fixp_quinticpolynomial_A0_A5(2*cnl::quotient(K2,pow_3<fixp_time_3>(t))) + 
    //     fixp_quinticpolynomial_A0_A5(6*cnl::quotient(K0,pow_5<fixp_time_5>(t))) -
    //     fixp_quinticpolynomial_A0_A5(3*cnl::quotient(K1,pow_4<fixp_time_4>(t))); 
    a3 = -4*K1+10*K0*inverse;
    a3 = inverse*a3+2*K2;
    a3 = inverse*a3;
    a4 = 7*K1-15*K0*inverse;
    a4 = inverse*a4 - 4*K2;
    a4 = a4*inverse*inverse;
    a5 = -3*K1 + 6*K0*inverse;
    a5 = inverse*a5 + 2*K2;
    a5 = a5*inverse*inverse*inverse;

    // #ifdef USE_RECORDER
    //     Recorder::getInstance()->saveData<float>("K", static_cast<float>(K));
    //     Recorder::getInstance()->saveData<float>("K0", static_cast<float>(K0));
    //     Recorder::getInstance()->saveData<float>("K1", static_cast<float>(K1));
    //     Recorder::getInstance()->saveData<float>("K2", static_cast<float>(K2));
    //     Recorder::getInstance()->saveData<float>("a0", static_cast<float>(a0));
    //     Recorder::getInstance()->saveData<float>("a1", static_cast<float>(a1));
    //     Recorder::getInstance()->saveData<float>("a2", static_cast<float>(a2));
    //     Recorder::getInstance()->saveData<float>("a3", static_cast<float>(a3));
    //     Recorder::getInstance()->saveData<float>("a4", static_cast<float>(a4));
    //     Recorder::getInstance()->saveData<float>("a5", static_cast<float>(a5));
    //     Recorder::getInstance()->saveData<float>("xs", static_cast<float>(xs));
    //     Recorder::getInstance()->saveData<float>("vxs", static_cast<float>(vxs));
    //     Recorder::getInstance()->saveData<float>("axs", static_cast<float>(axs));
    //     Recorder::getInstance()->saveData<float>("xe", static_cast<float>(xe));
    //     Recorder::getInstance()->saveData<float>("vxe", static_cast<float>(vxe));
    //     Recorder::getInstance()->saveData<float>("axe", static_cast<float>(axe));
    //     Recorder::getInstance()->saveData<float>("t", static_cast<float>(t));
    // #endif
}

fixp_d QuinticPolynomial::calc_point(fixp_maxt t) {
    // return a0 + a1 * t + a2 * pow_2<fixp_time_2>(t) + a3 * pow_3<fixp_time_3>(t) +
    // a4 * pow_4<fixp_time_4>(t) + a5 * pow_5<fixp_time_5>(t);
    // return a0+t*(a1+t*(a2+t*(a3+t*(a4+t*a5))));
    fixp_maxt ans = a4+t*a5;
    ans = a3+t*ans;
    ans = a2+t*ans;
    ans = a1+t*ans;
    ans = a0+t*ans;
    return ans;
}

fixp_d_d QuinticPolynomial::calc_first_derivative(fixp_maxt t) {
    // return a1 + 2 * a2 * t + 3 * a3 * pow_2<fixp_time_2>(t) + 4 * a4 * pow_3<fixp_time_3>(t) +
    // 5 * a5 * pow_4<fixp_time_4>(t);
    // return a1+t*(2*a2+t*(3*a3+t*(4*a4+5*a5*t)));
    fixp_maxt ans = 4*a4+5*a5*t;
    ans = 3*a3+t*ans;
    ans = 2*a2+t*ans;
    ans = a1+t*ans;
    return ans;
}

fixp_d_dd QuinticPolynomial::calc_second_derivative(fixp_maxt t) {
    // return 2 * a2 + 6 * a3 * t + 12 * a4 * pow_2<fixp_time_2>(t) + 20 * a5 * pow_3<fixp_time_3>(t);
    fixp_maxt ans = 12*a4+20*a5*t;
    ans = 6*a3+t*ans;
    ans = 2*a2+t*ans;
    return ans;
}

fixp_d_ddd QuinticPolynomial::calc_third_derivative(fixp_maxt t) {
    // return 6 * a3 + 24 * a4 * t + 60 * a5 * pow_2<fixp_time_2>(t);
    fixp_maxt ans = 24*a4+60*a5*t;
    ans = 6*a3+t*ans;
    return ans;
}