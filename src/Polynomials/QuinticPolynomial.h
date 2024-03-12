#ifndef FRENET_OPTIMAL_TRAJECTORY_QUINTICPOLYNOMIAL_H
#define FRENET_OPTIMAL_TRAJECTORY_QUINTICPOLYNOMIAL_H

#include "tool/fp_datatype.h"

class QuinticPolynomial {
public:
    QuinticPolynomial() = default;
    QuinticPolynomial(double xs, double vxs, double axs, double xe,
                      double vxe, double axe, double t);
    fixp_d calc_point(double t);
    fixp_d_d calc_first_derivative(double t);
    fixp_d_dd calc_second_derivative(double t);
    fixp_d_ddd calc_third_derivative(double t);
private:
    int_4_11 a0, a1, a2, a3, a4, a5;
};

#endif //FRENET_OPTIMAL_TRAJECTORY_QUINTICPOLYNOMIAL_H
