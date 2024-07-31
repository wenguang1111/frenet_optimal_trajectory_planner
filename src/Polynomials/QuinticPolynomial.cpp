#include "QuinticPolynomial.h"
#ifdef USE_RECORDER
    #include "tool/recorder.h"
#endif

#include <cmath>
#include <iostream>

QuinticPolynomial::QuinticPolynomial(float xs, float vxs, float axs,
        float xe, float vxe, float axe, float t):
        a0(xs), a1(vxs) {
    a2 = axs / 2.0;
    a3 = (-3*axs+2*axe+(-6*vxs-4*vxe+10*(xe-xs)/t)/t)/t;
    a4 = (9.0/2.0*axs-4*axe+(7*vxe+8*vxs+15*(xs-xe)/t)/t)/(t*t);
    a5 = (2*axe-2*axs+(-3*vxe-3*vxs+6*(xe-xs)/t)/t)/(t*t*t);
}

float QuinticPolynomial::calc_point(float t) {
    return a0 + a1 * t + a2 * pow(t, 2) + a3 * pow(t, 3) +
    a4 * pow(t, 4) + a5 * pow(t, 5);
}

float QuinticPolynomial::calc_first_derivative(float t) {
    return a1 + 2 * a2 * t + 3 * a3 * pow(t, 2) + 4 * a4 * pow(t, 3) +
    5 * a5 * pow(t, 4);
}

float QuinticPolynomial::calc_second_derivative(float t) {
    return 2 * a2 + 6 * a3 * t + 12 * a4 * pow(t, 2) + 20 * a5 * pow(t, 3);
}

float QuinticPolynomial::calc_third_derivative(float t) {
    return 6 * a3 + 24 * a4 * t + 60 * a5 * pow(t, 2);
}

float QuinticPolynomial::getA0()
{
    return a0;
}

float QuinticPolynomial::getA1()
{
    return a1;
}

float QuinticPolynomial::getA2()
{
    return a2;
}

float QuinticPolynomial::getA3()
{
    return a3;
}

float QuinticPolynomial::getA4()
{
    return a4;
}

float QuinticPolynomial::getA5()
{
    return a5;
}