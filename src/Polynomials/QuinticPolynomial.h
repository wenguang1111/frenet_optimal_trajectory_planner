#ifndef FRENET_OPTIMAL_TRAJECTORY_QUINTICPOLYNOMIAL_H
#define FRENET_OPTIMAL_TRAJECTORY_QUINTICPOLYNOMIAL_H

#include "tool/fp_datatype.h"

class QuinticPolynomial {
public:
    QuinticPolynomial() = default;
    QuinticPolynomial(fixp_c_d xs, fixp_c_d vxs, fixp_c_d axs, fixp_c_d xe,
                      fixp_c_d vxe, fixp_c_d axe, fixp_maxt t);
    fixp_d calc_point(fixp_maxt t);
    fixp_d_d calc_first_derivative(fixp_maxt t);
    fixp_d_dd calc_second_derivative(fixp_maxt t);
    fixp_d_ddd calc_third_derivative(fixp_maxt t);
private:
    fixp_quinticpolynomial_A0_A5 a0, a1, a2, a3, a4, a5;
};

#endif //FRENET_OPTIMAL_TRAJECTORY_QUINTICPOLYNOMIAL_H
