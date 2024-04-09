#ifndef FRENET_OPTIMAL_TRAJECTORY_QUINTICPOLYNOMIAL_H
#define FRENET_OPTIMAL_TRAJECTORY_QUINTICPOLYNOMIAL_H

#include "tool/fp_datatype.h"

class QuinticPolynomial {
public:
    QuinticPolynomial() = default;
    QuinticPolynomial(float xs, float vxs, float axs, float xe,
                      float vxe, float axe, float t);
    fixp_d calc_point(float t);
    fixp_d_d calc_first_derivative(float t);
    fixp_d_dd calc_second_derivative(float t);
    fixp_d_ddd calc_third_derivative(float t);
private:
    fixp_quinticpolynomial_A0_A5 a0, a1, a2, a3, a4, a5;
};

#endif //FRENET_OPTIMAL_TRAJECTORY_QUINTICPOLYNOMIAL_H
