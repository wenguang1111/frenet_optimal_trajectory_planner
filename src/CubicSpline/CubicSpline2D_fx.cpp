#include "CubicSpline2D_fx.h"
#include "utils.h"
#include "cordic.h"
#ifdef USE_RECORDER
    #include "tool/recorder.h"
#endif

#include <algorithm>
#include <numeric>
#include <cmath>

using namespace std;

// Default constructor
CubicSpline2D_fx::CubicSpline2D_fx() = default;

// Construct the 2-dimensional cubic spline
CubicSpline2D_fx::CubicSpline2D_fx(const vector<fp_type> &x,
                             const vector<fp_type> &y) {
    vector<vector<fp_type>> filtered_points = remove_collinear_points(x, y);
    calc_s(filtered_points[0], filtered_points[1]);
    sx = CubicSpline1D_fx(s, filtered_points[0]);
    sy = CubicSpline1D_fx(s, filtered_points[1]);
}

// Calculate the s values for interpolation given x, y
void CubicSpline2D_fx::calc_s(const vector<fp_type>& x,
                           const vector<fp_type>& y) {
    int nx = x.size();
    vector<fp_type> dx (nx);//delta_x
    vector<fp_type> dy (nx);
    adjacent_difference(x.begin(), x.end(), dx.begin());
    adjacent_difference(y.begin(), y.end(), dy.begin());
    dx.erase(dx.begin());
    dy.erase(dy.begin());

    fp_type cum_sum = 0.0;
    s.push_back(cum_sum);
    for (int i = 0; i < nx - 1; i++) {
        cum_sum += norm<fp_type>(dx[i], dy[i]);
        // #ifdef USE_RECORDER
        //     Recorder::getInstance()->saveData<double>("CubicSpline2D::calc_s::cum_sum", cum_sum);
        // #endif
        s.push_back(cum_sum);
    }
    s.erase(unique(s.begin(), s.end()), s.end());
}

// Calculate the x position along the spline at given t
fp_type CubicSpline2D_fx::calc_x(fp_type t) {
    return sx.calc_der0(t);
}

// Calculate the y position along the spline at given t
fp_type CubicSpline2D_fx::calc_y(fp_type t) {
    return sy.calc_der0(t);
}

// // Calculate the curvature along the spline at given t
// float CubicSpline2D::calc_curvature(float t){
//     float dx = sx.calc_der1(t);
//     float ddx = sx.calc_der2(t);
//     float dy = sy.calc_der1(t);
//     float ddy = sy.calc_der2(t);
//     float k = (ddy * dx - ddx * dy) /
//             pow(pow(dx, 2) + pow(dy, 2), 1.5);
//     return k;
// }

// Calculate the yaw along the spline at given t
fp_type CubicSpline2D_fx::calc_yaw(fp_type t) {
    fp_type dx = sx.calc_der1(t);
    fp_type dy = sy.calc_der1(t);
    fp_type yaw = cordic_atan<fp_type>(dy, dx);
    return yaw;
}

// Given x, y positions and an initial guess s0, find the closest s value
fp_type CubicSpline2D_fx::find_s(fp_type x, fp_type y, fp_type s0) {
    fp_type s_closest = s0;
    fp_type closest = std::numeric_limits<fp_type>::max();
    fp_type si = s.front();

    do {
        fp_type px = calc_x(si);
        fp_type py = calc_y(si);
        fp_type dist = norm<fp_type>(x - px, y - py);
        if (dist < closest) {
            closest = dist;
            s_closest = si;
        }
        si += 0.1;
    } while (si < s.back());
    return s_closest;
}

// Remove any collinear points from given list of points by the triangle rule
vector<vector<fp_type>>
CubicSpline2D_fx::remove_collinear_points(vector<fp_type> x, vector<fp_type> y) {
    vector<vector<fp_type>> filtered_points;
    vector<fp_type> x_; 
    vector<fp_type> y_;
    x_.push_back(static_cast<fp_type>(x[0]));
    x_.push_back(static_cast<fp_type>(x[1]));
    y_.push_back(static_cast<fp_type>(y[0]));
    y_.push_back(static_cast<fp_type>(y[1]));
    for (size_t i = 2; i < x.size()-1; i++) {
        bool collinear = are_collinear(
            x[i - 2], y[i - 2],
            x[i - 1], y[i - 1],
            x[i], y[i]
            );
        if (collinear) {
            continue;
        }
        x_.push_back(static_cast<fp_type>(x[i]));
        y_.push_back(static_cast<fp_type>(y[i]));
    }
    // make sure to add the last point in case all points are collinear
    x_.push_back(x.back());
    y_.push_back(y.back());
    filtered_points.push_back(x_);
    filtered_points.push_back(y_);
    return filtered_points;
}

// Determine if 3 points are collinear using the triangle area rule
bool CubicSpline2D_fx::are_collinear(fp_type x1, fp_type y1, fp_type x2, fp_type y2,
    fp_type x3, fp_type y3) {
    fp_type a = x1 * (y2 - y3) +
               x2 * (y3 - y1) +
               x3 * (y1 - y2);
    // #ifdef USE_RECORDER
    //     Recorder::getInstance()->saveData<double>("CubicSpline2D::are_collinear::a", a);
    // #endif
    return a <= 0.01;
}

bool CubicSpline2D_fx::isValidPath()
{
    if(sx.isValidPath()==true && sy.isValidPath()==true)
    {
        return true;
    }
    else
    {
        return false;
    }
}
