#ifndef FRENET_OPTIMAL_TRAJECTORY_QUARTICPOLYNOMIAL_FX_H
#define FRENET_OPTIMAL_TRAJECTORY_QUARTICPOLYNOMIAL_FX_H

#include "tool/fp_datatype.h"

class QuarticPolynomial_fx {
public:
    QuarticPolynomial_fx() = default;
    QuarticPolynomial_fx(fp_type xs, fp_type vxs, fp_type axs, fp_type vxe,
                      fp_type axe, fp_type t);
    fp_type calc_point(fp_type t);
    fp_type calc_first_derivative(fp_type t);
    fp_type calc_second_derivative(fp_type t);
    fp_type calc_third_derivative(fp_type t);
private:
    fp_type a0; 
    fp_type a1, a2, a3, a4;
};

#endif //FRENET_OPTIMAL_TRAJECTORY_QUARTICPOLYNOMIAL_FX_H
