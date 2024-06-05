#include "QuinticPolynomial.h"
#include "tool/recorder.h"

#include <cmath>

QuinticPolynomial::QuinticPolynomial(fixp_c_d xs, fixp_c_d vxs, fixp_c_d axs,
        fixp_c_d xe, fixp_c_d vxe, fixp_c_d axe, fixp_maxt t):
        a0(xs), a1(vxs) {
     a2 = axs / 2.0;

    //Gaussian elimination
    fixp_quinticpolynomial_K K;
    fixp_quinticpolynomial_K0 K0;
    fixp_quinticpolynomial_K1 K1;
    fixp_quinticpolynomial_K2 K2;
    // fixp_quinticpolynomial_t2 t2;
    // fixp_quinticpolynomial_t3 t3;
    // fixp_quinticpolynomial_t4 t4;
    // fixp_quinticpolynomial_t5 t5;
    // t2=pow_2(t);
    // t3=pow_3(t);
    // t4=pow_4(t);
    // t5=pow_5(t);
    K=a0+a1*t+a2*pow_2<fixp_quinticpolynomial_K>(t);
    K0=xe-K;
    K1=vxe-2*a2*t-a1;
    K2=axe-2*a2;
    a3=10*K0/pow_3<fixp_quinticpolynomial_A0_A5>(t)-4*K1/pow_2<fixp_quinticpolynomial_A0_A5>(t)+2*K2/t;
    a4=7*K1/pow_3<fixp_quinticpolynomial_A0_A5>(t)-15*K0/pow_4<fixp_quinticpolynomial_A0_A5>(t)-4*K2/pow_2<fixp_quinticpolynomial_A0_A5>(t);
    a5=2*K2/pow_3<fixp_quinticpolynomial_A0_A5>(t)+6*K0/pow_5<fixp_quinticpolynomial_A0_A5>(t)-3*K1/pow_4<fixp_quinticpolynomial_A0_A5>(t); 

    #ifdef USE_RECORDER
        Recorder::getInstance()->saveData<double>("QuinticPolynomial::K", K);
        Recorder::getInstance()->saveData<double>("QuinticPolynomial::K0", K0);
        Recorder::getInstance()->saveData<double>("QuinticPolynomial::K1", K1);
        Recorder::getInstance()->saveData<double>("QuinticPolynomial::K2", K2);
        Recorder::getInstance()->saveData<double>("QuinticPolynomial::t2", t2);
        Recorder::getInstance()->saveData<double>("QuinticPolynomial::t3", t3);
        Recorder::getInstance()->saveData<double>("QuinticPolynomial::t4", t4);
        Recorder::getInstance()->saveData<double>("QuinticPolynomial::t5", t5);
    #endif
}

fixp_d QuinticPolynomial::calc_point(fixp_maxt t) {
    return a0 + a1 * t + a2 * pow_2<fixp_d>(t) + a3 * pow_3<fixp_d>(t) +
    a4 * pow_4<fixp_d>(t) + a5 * pow_5<fixp_d>(t);
}

fixp_d_d QuinticPolynomial::calc_first_derivative(fixp_maxt t) {
    return a1 + 2 * a2 * t + 3 * a3 * pow_2<fixp_d_d>(t) + 4 * a4 * pow_3<fixp_d_d>(t) +
    5 * a5 * pow_4<fixp_d_d>(t);
}

fixp_d_dd QuinticPolynomial::calc_second_derivative(fixp_maxt t) {
    return 2 * a2 + 6 * a3 * t + 12 * a4 * pow_2<fixp_d_dd>(t) + 20 * a5 * pow_3<fixp_d_dd>(t);
}

fixp_d_ddd QuinticPolynomial::calc_third_derivative(fixp_maxt t) {
    return 6 * a3 + 24 * a4 * t + 60 * a5 * pow_2<fixp_d_ddd>(t);
}