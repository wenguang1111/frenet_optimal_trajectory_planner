#include "QuarticPolynomial.h"
#ifdef USE_RECORDER
    #include "tool/recorder.h"
#endif

#include <Eigen/LU>
#include <cmath>

using namespace Eigen;

QuarticPolynomial::QuarticPolynomial(float xs, float vxs, float axs,
        float vxe, float axe, float t):
        a0(xs), a1(vxs) {
    a2 = axs / 2.0;
    Matrix2d A;
    Vector2d B;
    A << 3 * pow(t, 2), 4 * pow(t, 3), 6 * t, 12 * pow(t, 2);
    B << vxe - a1 - 2 * a2 * t, axe - 2 * a2;
    Matrix2d A_inv = A.inverse();
    Vector2d x = A_inv * B;
    a3 = x[0];
    a4 = x[1];

    #ifdef USE_RECORDER
        Recorder::getInstance()->saveData<float>("QuarticPolynomial::QuarticPolynomial::a0", xs);
        Recorder::getInstance()->saveData<float>("QuarticPolynomial::QuarticPolynomial::a1", vxs);
        Recorder::getInstance()->saveData<float>("QuarticPolynomial::QuarticPolynomial::a2", a2);
        Recorder::getInstance()->saveData<float>("QuarticPolynomial::QuarticPolynomial::a3", a3);
        Recorder::getInstance()->saveData<float>("QuarticPolynomial::QuarticPolynomial::a4", a4);
    #endif
}

float QuarticPolynomial::calc_point(float t) {
    return a0 + a1 * t + a2 * pow(t, 2) + a3 * pow(t, 3) + a4 * pow(t, 4);
}

float QuarticPolynomial::calc_first_derivative(float t) {
    return a1 + 2 * a2 * t + 3 * a3 * pow(t, 2) + 4 * a4 * pow(t, 3);
}

float QuarticPolynomial::calc_second_derivative(float t) {
    return 2 * a2 + 6 * a3 * t + 12 * a4 * pow(t, 2);
}

float QuarticPolynomial::calc_third_derivative(float t) {
    return 6 * a3 + 24 * a4 * t;
}