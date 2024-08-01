#ifndef FRENET_OPTIMAL_TRAJECTORY_CUBICSPLINE2D_FX_H
#define FRENET_OPTIMAL_TRAJECTORY_CUBICSPLINE2D_FX_H

#include "CubicSpline1D_fx.h"

#include <vector>

using namespace std;

// 2-dimensional cubic spline class.
// For technical details see: http://mathworld.wolfram.com/CubicSpline.html
class CubicSpline2D_fx {
public:
    CubicSpline2D_fx();
    CubicSpline2D_fx(const vector<fp_type> &x, const vector<fp_type> &y);
    fp_type calc_x(fp_type t);
    fp_type calc_y(fp_type t);
    fp_type calc_yaw(fp_type t);
    fp_type find_s(fp_type x, fp_type y, fp_type s0);
    bool isValidPath();
    inline fp_type getEndOfS()
    {
        return s.back();
    }

private:
    vector<fp_type> s;
    CubicSpline1D_fx sx, sy;
    void calc_s(const vector<fp_type>& x,
                const vector<fp_type>& y);
    vector<vector<fp_type>> remove_collinear_points(vector<fp_type> x,
        vector<fp_type> y);
    bool are_collinear(fp_type x1, fp_type y1, fp_type x2, fp_type y2, fp_type x3, fp_type y3);
};

#endif //FRENET_OPTIMAL_TRAJECTORY_CUBICSPLINE2D_FX_H
