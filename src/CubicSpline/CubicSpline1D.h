#ifndef FRENET_OPTIMAL_TRAJECTORY_CUBICSPLINE1D_H
#define FRENET_OPTIMAL_TRAJECTORY_CUBICSPLINE1D_H

#include <vector>

// 1-dimensional cubic spline class.
// For technical details see: http://mathworld.wolfram.com/CubicSpline.html
class CubicSpline1D {
public:
    int nx;
    CubicSpline1D();
    CubicSpline1D(const std::vector<double>& v1, const std::vector<double>& v2);
    float calc_der0(float t);
    float calc_der1(float t);
    float calc_der2(float t);
private:
    std::vector<double> a, b, c, d, w, x, y;
    std::vector<double> tridionalmatrix_a, tridionalmatrix_b, tridionalmatrix_c, tridionalmatrix_d;
    int search_index(float t);
    void assignValue(std::vector<double> &TM_a, std::vector<double> &TM_b, 
            std::vector<double> &TM_c, std::vector<double> &TM_d, std::vector<double> &deltas);
};

#endif //FRENET_OPTIMAL_TRAJECTORY_CUBICSPLINE1D_H
