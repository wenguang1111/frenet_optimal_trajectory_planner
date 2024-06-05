#ifndef FRENET_OPTIMAL_TRAJECTORY_CUBICSPLINE2D_H
#define FRENET_OPTIMAL_TRAJECTORY_CUBICSPLINE2D_H

#include "CubicSpline1D.h"
#include "tool/fp_datatype.h"

#include <vector>

using namespace std;

// 2-dimensional cubic spline class.
// For technical details see: http://mathworld.wolfram.com/CubicSpline.html
class CubicSpline2D {
public:
    CubicSpline2D();
    CubicSpline2D(const vector<fixp_x> &x, const vector<fixp_y> &y);
    fixp_x calc_x(fixp_s t);
    fixp_y calc_y(fixp_s t);
    // float calc_curvature(fixp_t t);
    fixp_yaw calc_yaw(fixp_s t);
    fixp_s find_s(fixp_x x, fixp_y y, fixp_s s0);

private:
    vector<fixp_s> s;
    CubicSpline1D sx, sy;
    void calc_s(const vector<fixp_x>& x,
                const vector<fixp_y>& y);
    vector<vector<fixp_x>> remove_collinear_points(vector<fixp_x> x,
        vector<fixp_y> y);
    bool are_collinear(fixp_x x1, fixp_y y1, fixp_x x2, fixp_y y2, fixp_x x3, fixp_y y3);
};

#endif //FRENET_OPTIMAL_TRAJECTORY_CUBICSPLINE2D_H
