#include "QuarticPolynomial.h"
#ifdef USE_RECORDER
    #include "tool/recorder.h"
#endif

#include <cmath>

QuarticPolynomial::QuarticPolynomial(float xs, float vxs, float axs,
        float vxe, float axe, float t):
        a0(xs), a1(vxs) {
    a2 = axs / 2.0;
    fixp_quarticpolynomial_K1 K1;
    fixp_quarticpolynomial_K2 K2;
    K1=vxe-a1-2*a2*t;
    K2=axe-2*a2;
    a3 = K1/pow(t,2) - K2/(3*t);
    a4 = K2/(4*pow(t,2))-K1/(2*pow(t,3));
    #ifdef USE_RECORDER
        Recorder::getInstance()->saveData<double>("QuarticPolynomial::K1", K1);
        Recorder::getInstance()->saveData<double>("QuarticPolynomial::K2", K2);
    #endif
}

fixp_s QuarticPolynomial::calc_point(float t) {
    return a0 + a1 * t + a2 * pow(t, 2) + a3 * pow(t, 3) + a4 * pow(t, 4);
}

fixp_s_d QuarticPolynomial::calc_first_derivative(float t) {
    return a1 + 2 * a2 * t + 3 * a3 * pow(t, 2) + 4 * a4 * pow(t, 3);
}

fixp_s_dd QuarticPolynomial::calc_second_derivative(float t) {
    return 2 * a2 + 6 * a3 * t + 12 * a4 * pow(t, 2);
}

fixp_s_ddd QuarticPolynomial::calc_third_derivative(float t) {
    return 6 * a3 + 24 * a4 * t;
}