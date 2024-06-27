#include "QuarticPolynomial.h"
#include "utils.h"
#ifdef USE_RECORDER
    #include "tool/recorder.h"
#endif

#include <cmath>

QuarticPolynomial::QuarticPolynomial(fixp_s xs, fixp_s_d vxs, fixp_s_dd axs,
        fixp_s_d vxe, fixp_s_dd axe, fixp_maxt t):
        a0(xs), a1(vxs) {
    a2 = cnl::quotient(axs, fixp_s_dd(2.0));
    fixp_quarticpolynomial_K1 K1;
    fixp_quarticpolynomial_K2 K2;
    K1=vxe-a1-2*a2*t;
    K2=axe-2*a2;
    a3 = fixp_quarticpolynomial_A1_A4(cnl::quotient(K1, pow_2<fixp_time_2>(t))) - 
            fixp_quarticpolynomial_A1_A4(cnl::quotient(K2,(3*t)));
    a4 = fixp_quarticpolynomial_A1_A4(cnl::quotient(K2,(4*pow_2<fixp_time_2>(t)))) - 
            fixp_quarticpolynomial_A1_A4(cnl::quotient(K1,(2*pow_3<fixp_time_3>(t))));
    // #ifdef USE_RECORDER
    //     Recorder::getInstance()->saveData<double>("QuarticPolynomial::K1", K1);
    //     Recorder::getInstance()->saveData<double>("QuarticPolynomial::K2", K2);
    // #endif
}

fixp_s QuarticPolynomial::calc_point(fixp_maxt t) {
    return a0 + a1 * t + a2 * pow_2<fixp_time_2>(t) + a3 * pow_3<fixp_time_3>(t) + a4 * pow_4<fixp_time_4>(t);
}

fixp_s_d QuarticPolynomial::calc_first_derivative(fixp_maxt t) {
    return a1 + 2 * a2 * t + 3 * a3 * pow_2<fixp_time_2>(t) + 4 * a4 * pow_3<fixp_time_3>(t);
}

fixp_s_dd QuarticPolynomial::calc_second_derivative(fixp_maxt t) {
    return 2 * a2 + 6 * a3 * t + 12 * a4 * pow_2<fixp_time_2>(t);
}

fixp_s_ddd QuarticPolynomial::calc_third_derivative(fixp_maxt t) {
    return 6 * a3 + 24 * a4 * t;
}