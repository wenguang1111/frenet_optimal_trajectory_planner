#ifndef FRENET_OPTIMAL_TRAJECTORY_CUBICSPLINE2D_H
#define FRENET_OPTIMAL_TRAJECTORY_CUBICSPLINE2D_H

#include "CubicSpline1D.h"

#include <vector>

using namespace std;

// 2-dimensional cubic spline class.
// For technical details see: http://mathworld.wolfram.com/CubicSpline.html
class CubicSpline2D {
public:
    CubicSpline2D();
    CubicSpline2D(const vector<float> &x, const vector<float> &y);
    float calc_x(float t);
    float calc_y(float t);
    float calc_curvature(float t);
    float calc_yaw(float t);
    float find_s(float x, float y, float s0);
    inline float getEndOfS()
    {
        return s.back();
    }
    inline std::vector<float> getX_A(){return sx.getA();}
    inline std::vector<float> getX_B(){return sx.getB();}
    inline std::vector<float> getX_C(){return sx.getC();}
    inline std::vector<float> getX_D(){return sx.getD();}
    inline std::vector<float> getX_X(){return sx.getX();}
    inline std::vector<float> getX_Y(){return sx.getY();}
    inline std::vector<float> getY_A(){return sy.getA();}
    inline std::vector<float> getY_B(){return sy.getB();}
    inline std::vector<float> getY_C(){return sy.getC();}
    inline std::vector<float> getY_D(){return sy.getD();}
    inline std::vector<float> getY_X(){return sy.getX();}
    inline std::vector<float> getY_Y(){return sy.getY();}

private:
    vector<float> s;
    CubicSpline1D sx, sy;
    void calc_s(const vector<float>& x,
                const vector<float>& y);
    vector<vector<float>> remove_collinear_points(vector<float> x,
        vector<float> y);
    bool are_collinear(float x1, float y1, float x2, float y2, float x3, float y3);
};

#endif //FRENET_OPTIMAL_TRAJECTORY_CUBICSPLINE2D_H
