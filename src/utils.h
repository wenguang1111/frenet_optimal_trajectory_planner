#ifndef FRENET_OPTIMAL_TRAJECTORY_UTILS_H
#define FRENET_OPTIMAL_TRAJECTORY_UTILS_H

#include <cmath>
#include <tuple>
#include <vector>

#include "tool/fp_datatype.h"

using namespace std;

template<typename T>
T pow_2(auto a)
{
    return a*a;
}

template<typename T>
T pow_3(auto a)
{
    return a*a*a;
}

template<typename T>
T pow_4(auto a)
{
    return a*a*a*a;
}

template<typename T>
T pow_5(auto a)
{
    return a*a*a*a*a;
}
struct Point{ 
    Point()=default;
    Point(fixp_x a, fixp_y b):x(a),y(b){};
    Point& operator =(const Point& a)
    {
        x=a.x;
        y=a.y;
        return *this;
    }
    fixp_x x;
    fixp_y y;
};

struct Pose{ 
    fixp_x x;
    fixp_y y;
    fixp_yaw yaw;
    Pose& operator =(const Pose& a)
    {
        x=a.x;
        y=a.y;
        yaw=a.yaw;
        return *this;
    }
};

struct Vector2D{
    Vector2D& operator =(const Vector2D& a)
    {
        x=a.x;
        y=a.y;
        return *this;
    }
    fixp_x x;
    fixp_y y;
};

struct Rectangle{
    Rectangle& operator =(const Rectangle& a)
    {
        points[0]=a.points[0];
        points[1]=a.points[1];
        points[2]=a.points[2];
        points[3]=a.points[3];
        return *this;
    }
    Vector2D points[4];
};

template <typename T>
inline T norm(T x, T y) {
    return cnl::sqrt(x*x+y*y);
    // return cnl::sqrt(x*x + y*y);
}

inline float norm_floating(float x, float y) {
    return sqrt(pow(x, 2) + pow(y, 2));
}

inline void as_unit_vector(tuple<float, float>& vec) {
    float magnitude = norm_floating(get<0>(vec), get<1>(vec));
    if (magnitude > 0) {
        get<0>(vec) = get<0>(vec) / magnitude;
        get<1>(vec) = get<1>(vec) / magnitude;
    }
}

inline float dot(const tuple<float, float>& vec1,
                  const tuple<float, float>& vec2) {
    return get<0>(vec1) * get<0>(vec2) +
           get<1>(vec1) * get<1>(vec2);
}

template<typename T>
void assignValueToFixedPoint(T* d_fp, float* d_float, int size)
{
    for(int i=0;i<size;i++)
    {
        d_fp[i] = d_float[i];
    }
}
#endif //FRENET_OPTIMAL_TRAJECTORY_UTILS_H
