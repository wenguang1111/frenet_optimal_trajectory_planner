#ifndef FRENET_OPTIMAL_TRAJECTORY_CUBICSPLINE1D_H
#define FRENET_OPTIMAL_TRAJECTORY_CUBICSPLINE1D_H

#include "tool/fp_datatype.h"
#include <vector>

// 1-dimensional cubic spline class.
// For technical details see: http://mathworld.wolfram.com/CubicSpline.html
class CubicSpline1D {
public:
    int nx;
    CubicSpline1D();
    CubicSpline1D(const std::vector<fixp_s>& v1, const std::vector<fixp_x>& v2);
    fixp_x calc_der0(fixp_s t);
    fixp_dx calc_der1(fixp_s t);
    // float calc_der2(fixp_t t);
private:
    std::vector<fixp_x> a,y;
    std::vector<fixp_cubicspline_b> b;
    std::vector<fixp_cubicspline_c> c;
    std::vector<fixp_cubicspline_d> d;
    std::vector<fixp_s> x;
    std::vector<fixp_TM_a> tridionalmatrix_a;
    std::vector<fixp_TM_b> tridionalmatrix_b;
    std::vector<fixp_TM_c> tridionalmatrix_c;
    std::vector<fixp_TM_d> tridionalmatrix_d;
    int search_index(fixp_s t);
    void assignValue(std::vector<fixp_TM_a> &TM_a, std::vector<fixp_TM_b> &TM_b, 
            std::vector<fixp_TM_c> &TM_c, std::vector<fixp_TM_d> &TM_d, std::vector<fixp_s> &deltas);
};

#endif //FRENET_OPTIMAL_TRAJECTORY_CUBICSPLINE1D_H
