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

    // fixp_quarticpolynomial_K1 K1;
    // fixp_quarticpolynomial_K2 K2;
    // K1=vxe-a1-2*a2*t;
    // K2=axe-2*a2;
    // fixp_maxt inverse = cnl::quotient(1,t);
    // a3 = inverse*(K1*inverse-K2/3);
    // a4 = K2/4-(K1/2)*inverse;
    // a4 = inverse*a4;
    // a4 = inverse*a4;
    fp_type inverse = cnl::quotient(1,t);
    a3 = (vxe-vxs)*inverse;
    a3 = a3 + cnl::quotient(-axe-2*axs,3);
    a3 *= inverse;
    a4 = cnl::quotient(vxs-vxe,2);
    a4 *= inverse;
    a4 +=  cnl::quotient(axe+axs,4);
    a4*=inverse;
    a4*=inverse;
}

fp_type QuarticPolynomial_fx::calc_point(fp_type t) {
    fp_type ans=a3+t*a4;
    ans=ans*t+a2;
    ans=ans*t+a1;
    ans=ans*t+a0;
    return ans;
}

fp_type QuarticPolynomial_fx::calc_first_derivative(fp_type t) {
    fp_type ans = 3*a3+4*a4*t;
    ans = 2*a2+t*ans;
    ans = a1+t*ans;
    return ans;
}

fp_type QuarticPolynomial_fx::calc_second_derivative(fp_type t) {
    fp_type ans = 6*a3+12*a4*t;
    ans = 2*a2+t*ans;
    return ans;
}

fp_type QuarticPolynomial_fx::calc_third_derivative(fp_type t) {
    return 6 * a3 + 24 * a4 * t;
}