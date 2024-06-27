#include "QuinticPolynomial.h"
#include "utils.h"
#ifdef USE_RECORDER
    #include "tool/recorder.h"
#endif

#include <cmath>

QuinticPolynomial::QuinticPolynomial(fixp_c_d xs, fixp_c_d vxs, fixp_c_d axs,
        fixp_c_d xe, fixp_c_d vxe, fixp_c_d axe, fixp_maxt t):
        a0(xs), a1(vxs) {
     a2 = cnl::quotient(axs, fixp_c_d(2.0));

    //Gaussian elimination
    fixp_quinticpolynomial_K K;
    fixp_quinticpolynomial_K0 K0;
    fixp_quinticpolynomial_K1 K1;
    fixp_quinticpolynomial_K2 K2;

    K=a0+a1*t+a2*pow_2<fixp_quinticpolynomial_K>(t);
    K0=xe-K;
    K1=vxe-2*a2*t-a1;
    K2=axe-2*a2;
    // static_assert(is_same<decltype(asd), elastic_integer<13, int8_t>>::value);

    a3=fixp_quinticpolynomial_A0_A5(10*cnl::quotient(K0,pow_3<fixp_time_3>(t))) - 
        fixp_quinticpolynomial_A0_A5(4*cnl::quotient(K1,pow_2<fixp_time_2>(t))) + 
        fixp_quinticpolynomial_A0_A5(2*cnl::quotient(K2,t));
    a4=fixp_quinticpolynomial_A0_A5(7*cnl::quotient(K1,pow_3<fixp_time_3>(t))) - 
        fixp_quinticpolynomial_A0_A5(15*cnl::quotient(K0,pow_4<fixp_time_4>(t))) - 
        fixp_quinticpolynomial_A0_A5(4*cnl::quotient(K2,pow_2<fixp_time_2>(t)));
    a5=fixp_quinticpolynomial_A0_A5(2*cnl::quotient(K2,pow_3<fixp_time_3>(t))) + 
        fixp_quinticpolynomial_A0_A5(6*cnl::quotient(K0,pow_5<fixp_time_5>(t))) -
        fixp_quinticpolynomial_A0_A5(3*cnl::quotient(K1,pow_4<fixp_time_4>(t))); 
    // #ifdef USE_RECORDER
    //     Recorder::getInstance()->saveData<double>("QuinticPolynomial::K", K);
    //     Recorder::getInstance()->saveData<double>("QuinticPolynomial::K0", K0);
    //     Recorder::getInstance()->saveData<double>("QuinticPolynomial::K1", K1);
    //     Recorder::getInstance()->saveData<double>("QuinticPolynomial::K2", K2);
    //     Recorder::getInstance()->saveData<double>("QuinticPolynomial::t2", t2);
    //     Recorder::getInstance()->saveData<double>("QuinticPolynomial::t3", t3);
    //     Recorder::getInstance()->saveData<double>("QuinticPolynomial::t4", t4);
    //     Recorder::getInstance()->saveData<double>("QuinticPolynomial::t5", t5);
    // #endif
}

fixp_d QuinticPolynomial::calc_point(fixp_maxt t) {
    return a0 + a1 * t + a2 * pow_2<fixp_time_2>(t) + a3 * pow_3<fixp_time_3>(t) +
    a4 * pow_4<fixp_time_4>(t) + a5 * pow_5<fixp_time_5>(t);
}

fixp_d_d QuinticPolynomial::calc_first_derivative(fixp_maxt t) {
    return a1 + 2 * a2 * t + 3 * a3 * pow_2<fixp_time_2>(t) + 4 * a4 * pow_3<fixp_time_3>(t) +
    5 * a5 * pow_4<fixp_time_4>(t);
}

fixp_d_dd QuinticPolynomial::calc_second_derivative(fixp_maxt t) {
    return 2 * a2 + 6 * a3 * t + 12 * a4 * pow_2<fixp_time_2>(t) + 20 * a5 * pow_3<fixp_time_3>(t);
}

fixp_d_ddd QuinticPolynomial::calc_third_derivative(fixp_maxt t) {
    return 6 * a3 + 24 * a4 * t + 60 * a5 * pow_2<fixp_time_2>(t);
}