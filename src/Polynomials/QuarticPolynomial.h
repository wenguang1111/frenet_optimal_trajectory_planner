#ifndef FRENET_OPTIMAL_TRAJECTORY_QUARTICPOLYNOMIAL_H
#define FRENET_OPTIMAL_TRAJECTORY_QUARTICPOLYNOMIAL_H

#include "tool/fp_datatype.h"

class QuarticPolynomial {
public:
    QuarticPolynomial() = default;
    QuarticPolynomial(double xs, double vxs, double axs, double vxe,
                      double axe, double t);
    fixp_s calc_point(double t);
    fixp_s_d calc_first_derivative(double t);
    fixp_s_dd calc_second_derivative(double t);
    fixp_s_ddd calc_third_derivative(double t);
private:
    int_8_7 a0; 
    int_4_11 a1, a2, a3, a4;
};

#endif //FRENET_OPTIMAL_TRAJECTORY_QUARTICPOLYNOMIAL_H
