#include "QuinticPolynomial.h"
#ifdef USE_RECORDER
    #include "tool/recorder.h"
#endif

#include <Eigen/LU>
#include <cmath>

using namespace Eigen;

QuinticPolynomial::QuinticPolynomial(float xs, float vxs, float axs,
        float xe, float vxe, float axe, float t):
        a0(xs), a1(vxs) {
    a2 = axs / 2.0;
    Matrix3d A;
    Vector3d B;
    A << pow(t, 3), pow(t, 4), pow(t, 5), 3 * pow(t, 2),
    4 * pow(t, 3), 5 * pow(t, 4), 6 * t, 12 * pow(t, 2),
    20 * pow(t, 3);
    B << xe - a0 - a1 * t - a2 * pow(t, 2), vxe - a1 - 2 * a2 * t,
    axe - 2 * a2;
    Matrix3d A_inv = A.inverse();
    Vector3d x = A_inv * B;
    a3 = x[0];
    a4 = x[1];
    a5 = x[2];

    #ifdef USE_RECORDER
        Recorder::getInstance()->saveData<float>("QuinticPolynomial::QuinticPolynomial::a0", xs);
        Recorder::getInstance()->saveData<float>("QuinticPolynomial::QuinticPolynomial::a1", vxs);
        Recorder::getInstance()->saveData<float>("QuinticPolynomial::QuinticPolynomial::a2", a2);
        Recorder::getInstance()->saveData<float>("QuinticPolynomial::QuinticPolynomial::a3", a3);
        Recorder::getInstance()->saveData<float>("QuinticPolynomial::QuinticPolynomial::a4", a4);
        Recorder::getInstance()->saveData<float>("QuinticPolynomial::QuinticPolynomial::a5", a5);
    #endif
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