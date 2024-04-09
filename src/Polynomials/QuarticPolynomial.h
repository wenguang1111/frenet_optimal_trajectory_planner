#ifndef FRENET_OPTIMAL_TRAJECTORY_QUARTICPOLYNOMIAL_H
#define FRENET_OPTIMAL_TRAJECTORY_QUARTICPOLYNOMIAL_H

#include "tool/fp_datatype.h"

class QuarticPolynomial {
public:
    QuarticPolynomial() = default;
    QuarticPolynomial(float xs, float vxs, float axs, float vxe,
                      float axe, float t);
    fixp_s calc_point(float t);
    fixp_s_d calc_first_derivative(float t);
    fixp_s_dd calc_second_derivative(float t);
    fixp_s_ddd calc_third_derivative(float t);
private:
    fixp_quarticpolynomial_A0 a0; 
    fixp_quarticpolynomial_A1_A4 a1, a2, a3, a4;
};

#endif //FRENET_OPTIMAL_TRAJECTORY_QUARTICPOLYNOMIAL_H
