#ifndef FRENET_OPTIMAL_TRAJECTORY_CUBICSPLINE1D_FX_H
#define FRENET_OPTIMAL_TRAJECTORY_CUBICSPLINE1D_FX_H

#include "cnl/all.h"
#include "tool/fp_datatype.h" 
#include <vector>

// 1-dimensional cubic spline class.
// For technical details see: http://mathworld.wolfram.com/CubicSpline.html
class CubicSpline1D_fx {
public:
    int nx;
    CubicSpline1D_fx();
    CubicSpline1D_fx(const std::vector<fp_type>& v1, const std::vector<fp_type>& v2);
    fp_type calc_der0(fp_time t);
    fp_type calc_der1(fp_time t);
    bool isValidPath();
    inline std::vector<fp_type> getA(){return a;}
    inline std::vector<fp_type> getB(){return b;}
    inline std::vector<fp_type> getC(){return c;}
    inline std::vector<fp_type> getD(){return d;}
    inline std::vector<fp_type> getX(){return x;}
    inline std::vector<fp_type> getY(){return y;}
private:
    std::vector<fp_type> x;
    std::vector<fp_type> y;
    std::vector<fp_type> a;
    std::vector<fp_type> b;
    std::vector<fp_type> c;
    std::vector<fp_type> d;
    std::vector<fp_type> tridionalmatrix_a;
    std::vector<fp_type> tridionalmatrix_b;
    std::vector<fp_type> tridionalmatrix_c;
    std::vector<fp_type> tridionalmatrix_d;
    int search_index(fp_type t);
    void assignValue(std::vector<fp_type> &TM_a, std::vector<fp_type> &TM_b, 
            std::vector<fp_type> &TM_c, std::vector<fp_type> &TM_d, std::vector<fp_type> &deltas);
    bool validPath;
};

#endif //FRENET_OPTIMAL_TRAJECTORY_CUBICSPLINE1D_FX_H
