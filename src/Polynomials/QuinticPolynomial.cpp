#include "QuinticPolynomial.h"
#ifdef USE_RECORDER
    #include "tool/recorder.h"
#endif

#include <cmath>

QuinticPolynomial::QuinticPolynomial(float xs, float vxs, float axs,
        float xe, float vxe, float axe, float t):
        a0(xs), a1(vxs) {
    a2 = axs / 2.0;

    //Gaussian elimination
    fixp_quinticpolynomial_K K;
    fixp_quinticpolynomial_K0 K0;
    fixp_quinticpolynomial_K1 K1;
    fixp_quinticpolynomial_K2 K2;
    fixp_quinticpolynomial_t2 t2;
    fixp_quinticpolynomial_t3 t3;
    fixp_quinticpolynomial_t4 t4;
    fixp_quinticpolynomial_t5 t5;
    t2=pow(t,2);
    t3=pow(t,3);
    t4=pow(t,4);
    t5=pow(t,5);
    K=a0+a1*t+a2*t2;
    K0=xe-K;
    K1=vxe-2*a2*t-a1;
    K2=axe-2*a2;
    a3=10*K0/t3-4*K1/t2+2*K2/t;
    a4=7*K1/t3-15*K0/t4-4*K2/t2;
    a5=2*K2/t3+6*K0/t5-3*K1/t4; //TODO: The t2-t4 can be removed by using pow() of fp. version.
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

fixp_d QuinticPolynomial::calc_point(float t) {
    return a0 + a1 * t + a2 * pow(t, 2) + a3 * pow(t, 3) +
    a4 * pow(t, 4) + a5 * pow(t, 5);
}

fixp_d_d QuinticPolynomial::calc_first_derivative(float t) {
    return a1 + 2 * a2 * t + 3 * a3 * pow(t, 2) + 4 * a4 * pow(t, 3) +
    5 * a5 * pow(t, 4);
}

fixp_d_dd QuinticPolynomial::calc_second_derivative(float t) {
    return 2 * a2 + 6 * a3 * t + 12 * a4 * pow(t, 2) + 20 * a5 * pow(t, 3);
}

fixp_d_ddd QuinticPolynomial::calc_third_derivative(float t) {
    return 6 * a3 + 24 * a4 * t + 60 * a5 * pow(t, 2);
}