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
    fixp_maxt inverse = cnl::quotient(1,t);
    // a3 = fixp_quarticpolynomial_A1_A4(cnl::quotient(K1, pow_2<fixp_time_2>(t))) - 
    //         fixp_quarticpolynomial_A1_A4(cnl::quotient(K2,(3*t)));
    a3 = inverse*(K1*inverse-K2/3);
    a4 = K2/4-(K1/2)*inverse;
    a4 = inverse*a4;
    a4 = inverse*a4;
    // #ifdef USE_RECORDER
    //     Recorder::getInstance()->saveData<float>("xs", static_cast<float>(xs));
    //     Recorder::getInstance()->saveData<float>("vxs", static_cast<float>(vxs));
    //     Recorder::getInstance()->saveData<float>("axs", static_cast<float>(axs));
    //     Recorder::getInstance()->saveData<float>("vxe", static_cast<float>(vxe));
    //     Recorder::getInstance()->saveData<float>("axe", static_cast<float>(axe));
    //     Recorder::getInstance()->saveData<float>("t", static_cast<float>(t));
    //     Recorder::getInstance()->saveData<float>("K1", static_cast<float>(K1));
    //     Recorder::getInstance()->saveData<float>("K2", static_cast<float>(K2));
    //     Recorder::getInstance()->saveData<float>("a0", static_cast<float>(a0));
    //     Recorder::getInstance()->saveData<float>("a1", static_cast<float>(a1));
    //     Recorder::getInstance()->saveData<float>("a2", static_cast<float>(a2));
    //     Recorder::getInstance()->saveData<float>("a3", static_cast<float>(a3));
    //     Recorder::getInstance()->saveData<float>("a4", static_cast<float>(a4));
    // #endif
}

fixp_s QuarticPolynomial::calc_point(fixp_maxt t) {
    // return a0 + a1 * t + a2 * pow_2<fixp_time_2>(t) + a3 * pow_3<fixp_time_3>(t) + a4 * pow_4<fixp_time_4>(t);
    // return a0+t*(a1+t*(a2+t*(a3+t*a4)));
    fixp_maxt ans=a3+t*a4;
    ans=ans*t+a2;
    ans=ans*t+a1;
    ans=ans*t+a0;
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