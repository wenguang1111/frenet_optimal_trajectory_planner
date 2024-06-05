#include "CubicSpline2D.h"
#include "utils.h"
#include "tool/fp_datatype.h"
#include "cordic.h"
#ifdef USE_RECORDER
    #include "tool/recorder.h"
#endif

#include <algorithm>
#include <numeric>
#include <cmath>

using namespace std;

// Default constructor
CubicSpline2D::CubicSpline2D() = default;

// Construct the 2-dimensional cubic spline
CubicSpline2D::CubicSpline2D(const vector<fixp_x> &x,
                             const vector<fixp_y> &y) {
    vector<vector<fixp_x>> filtered_points = remove_collinear_points(x, y);
    calc_s(filtered_points[0], filtered_points[1]);
    sx = CubicSpline1D(s, filtered_points[0]);
    sy = CubicSpline1D(s, filtered_points[1]);
}

// Calculate the s values for interpolation given x, y
void CubicSpline2D::calc_s(const vector<fixp_x>& x,
                           const vector<fixp_y>& y) {
    int nx = x.size();
    vector<fixp_x> dx (nx);//delta_x
    vector<fixp_y> dy (nx);
    adjacent_difference(x.begin(), x.end(), dx.begin());
    adjacent_difference(y.begin(), y.end(), dy.begin());
    dx.erase(dx.begin());
    dy.erase(dy.begin());

    fixp_cum_sum cum_sum = 0.0;
    s.push_back(cum_sum);
    for (int i = 0; i < nx - 1; i++) {
        cum_sum += norm(dx[i], dy[i]);
        // #ifdef USE_RECORDER
        //     Recorder::getInstance()->saveData<double>("CubicSpline2D::calc_s::cum_sum", cum_sum);
        // #endif
        s.push_back(cum_sum);
    }
    s.erase(unique(s.begin(), s.end()), s.end());
}

// Calculate the x position along the spline at given t
fixp_x CubicSpline2D::calc_x(fixp_s t) {
    return sx.calc_der0(t);
}

// Calculate the y position along the spline at given t
fixp_y CubicSpline2D::calc_y(fixp_s t) {
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
fixp_yaw CubicSpline2D::calc_yaw(fixp_s t) {
    fixp_dx dx = sx.calc_der1(t);
    fixp_dy dy = sy.calc_der1(t);
    fixp_yaw yaw = cordic_atan<fixp_dx>(dy, dx);
    return yaw;
}

// Given x, y positions and an initial guess s0, find the closest s value
fixp_s CubicSpline2D::find_s(fixp_x x, fixp_y y, fixp_s s0) {
    fixp_s s_closest = s0;
    fixp_x closest = std::numeric_limits<fixp_x>::max();
    fixp_s si = s.front();

    do {
        fixp_x px = calc_x(si);
        fixp_y py = calc_y(si);
        fixp_x dist = norm(x - px, y - py);
        if (dist < closest) {
            closest = dist;
            s_closest = si;
        }
        si += 0.1;
    } while (si < s.back());
    return s_closest;
}

// Remove any collinear points from given list of points by the triangle rule
vector<vector<fixp_x>>
CubicSpline2D::remove_collinear_points(vector<fixp_x> x, vector<fixp_y> y) {
    vector<vector<fixp_x>> filtered_points;
    vector<fixp_x> x_; 
    vector<fixp_y> y_;
    x_.push_back(static_cast<fixp_x>(x[0]));
    x_.push_back(static_cast<fixp_x>(x[1]));
    y_.push_back(static_cast<fixp_y>(y[0]));
    y_.push_back(static_cast<fixp_y>(y[1]));
    for (size_t i = 2; i < x.size()-1; i++) {
        bool collinear = are_collinear(
            x[i - 2], y[i - 2],
            x[i - 1], y[i - 1],
            x[i], y[i]
            );
        if (collinear) {
            continue;
        }
        x_.push_back(static_cast<fixp_x>(x[i]));
        y_.push_back(static_cast<fixp_y>(y[i]));
    }
    // make sure to add the last point in case all points are collinear
    x_.push_back(x.back());
    y_.push_back(y.back());
    filtered_points.push_back(x_);
    filtered_points.push_back(y_);
    return filtered_points;
}

// Determine if 3 points are collinear using the triangle area rule
bool CubicSpline2D::are_collinear(fixp_x x1, fixp_y y1, fixp_x x2, fixp_y y2,
    fixp_x x3, fixp_y y3) {
    fixp_x a = x1 * (y2 - y3) +
               x2 * (y3 - y1) +
               x3 * (y1 - y2);
    // #ifdef USE_RECORDER
    //     Recorder::getInstance()->saveData<double>("CubicSpline2D::are_collinear::a", a);
    // #endif
    return a <= 0.01;
}
