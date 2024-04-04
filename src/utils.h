#ifndef FRENET_OPTIMAL_TRAJECTORY_UTILS_H
#define FRENET_OPTIMAL_TRAJECTORY_UTILS_H

#include <cmath>
#include <tuple>
#include <vector>

#include "tool/fp_datatype.h"

using namespace std;

typedef struct { 
    double x,y;
} Point;

typedef struct { 
    fixp_x x;
    fixp_y y;
} Point_FP;

typedef struct {
    fixp_x x;
    fixp_y y;
} Vector2D;

typedef struct {
    Vector2D points[4];
} Rectangle;

typedef vector<double> Pose;

inline double norm(double x, double y) {
    return sqrt(pow(x, 2) + pow(y, 2));
}

inline fixp_x norm_FP(fixp_x x, fixp_y y) {
    return sqrt(static_cast<float>(x*x) + static_cast<float>(y*y));
}

inline void as_unit_vector(tuple<double, double>& vec) {
    double magnitude = norm(get<0>(vec), get<1>(vec));
    if (magnitude > 0) {
        get<0>(vec) = get<0>(vec) / magnitude;
        get<1>(vec) = get<1>(vec) / magnitude;
    }
}

inline double dot(const tuple<double, double>& vec1,
                  const tuple<double, double>& vec2) {
    return get<0>(vec1) * get<0>(vec2) +
           get<1>(vec1) * get<1>(vec2);
}
#endif //FRENET_OPTIMAL_TRAJECTORY_UTILS_H
