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
    fp_type calc_x(fp_time t);
    fp_type calc_y(fp_time t);
    Trignometric calc_yaw(fp_time t);
    fp_type find_s(fp_type x, fp_type y, fp_type s0);
    bool isValidPath();
    inline fp_type getEndOfS()
    {
        return s.back();
    }
    inline std::vector<fp_type> getX_A(){return sx.getA();}
    inline std::vector<fp_type> getX_B(){return sx.getB();}
    inline std::vector<fp_type> getX_C(){return sx.getC();}
    inline std::vector<fp_type> getX_D(){return sx.getD();}
    inline std::vector<fp_type> getX_X(){return sx.getX();}
    inline std::vector<fp_type> getX_Y(){return sx.getY();}
    inline std::vector<fp_type> getY_A(){return sy.getA();}
    inline std::vector<fp_type> getY_B(){return sy.getB();}
    inline std::vector<fp_type> getY_C(){return sy.getC();}
    inline std::vector<fp_type> getY_D(){return sy.getD();}
    inline std::vector<fp_type> getY_X(){return sy.getX();}
    inline std::vector<fp_type> getY_Y(){return sy.getY();}


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
