#ifndef FRENET_OPTIMAL_TRAJECTORY_CUBICSPLINE1D_H
#define FRENET_OPTIMAL_TRAJECTORY_CUBICSPLINE1D_H

#include <Eigen/LU>
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
    int search_index(float t);
    void matrix_a(std::vector<double>& deltas, Eigen::MatrixXd& result);
    void vector_b(std::vector<double>& deltas, Eigen::VectorXd& result);
};

#endif //FRENET_OPTIMAL_TRAJECTORY_CUBICSPLINE1D_H
