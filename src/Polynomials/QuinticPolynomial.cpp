#include "QuinticPolynomial.h"

#include <Eigen/LU>
#include <cmath>

using namespace Eigen;

QuinticPolynomial::QuinticPolynomial(fixp_c_d xs, fixp_c_d vxs, fixp_c_d axs,
        fixp_c_d xe, fixp_c_d vxe, fixp_c_d axe, fixp_maxt t):
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
}

fixp_d QuinticPolynomial::calc_point(fixp_maxt t) {
    return a0 + a1 * t + a2 * pow(t, 2) + a3 * pow(t, 3) +
    a4 * pow(t, 4) + a5 * pow(t, 5);
}

fixp_d_d QuinticPolynomial::calc_first_derivative(fixp_maxt t) {
    return a1 + 2 * a2 * t + 3 * a3 * pow(t, 2) + 4 * a4 * pow(t, 3) +
    5 * a5 * pow(t, 4);
}

fixp_d_dd QuinticPolynomial::calc_second_derivative(fixp_maxt t) {
    return 2 * a2 + 6 * a3 * t + 12 * a4 * pow(t, 2) + 20 * a5 * pow(t, 3);
}

fixp_d_ddd QuinticPolynomial::calc_third_derivative(fixp_maxt t) {
    return 6 * a3 + 24 * a4 * t + 60 * a5 * pow(t, 2);
}