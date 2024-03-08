#ifndef FRENET_OPTIMAL_TRAJECTORY_FRENETPATH_H
#define FRENET_OPTIMAL_TRAJECTORY_FRENETPATH_H

#include "py_cpp_struct.h"
#include "CubicSpline2D.h"
#include "Obstacle.h"
#include "Car.h"
#ifdef USE_RECORDER
    #include "tool/fp_datatype.h"
#endif


#include <eigen3/Eigen/Dense>
#include <vector>
#include <tuple>

using namespace std;
using namespace Eigen;

class FrenetPath {
public:
    // Frenet attributes
    vector<uint_3_13> t;          // time
    vector<int_3_12> d;          // lateral offset
    vector<int_3_12> d_d;        // lateral speed
    vector<int_4_11> d_dd;       // lateral acceleration
    vector<int_7_8> d_ddd;      // lateral jerk
    vector<int_8_7> s;          // s position along spline
    vector<int_5_10> s_d;        // s speed
    vector<int_5_10> s_dd;       // s acceleration
    vector<int_5_10> s_ddd;      // s jerk

    // Euclidean attributes
    vector<int_8_7> x;          // x position
    vector<int_3_12> y;          // y position
    vector<int_1_14> yaw;        // yaw in radc
    vector<int_3_12> ds;         // speed
    vector<int_3_12> c;          // curvature

    // Debug
    vector<double> ix;
    vector<double> iy;
    vector<double> iyaw;

    // Cost attributes
    // lateral costs
    double c_lateral_deviation = 0.0;
    double c_lateral_velocity = 0.0;
    double c_lateral_acceleration = 0.0;
    double c_lateral_jerk = 0.0;
    double c_lateral = 0.0;

    // longitudinal costs
    double c_longitudinal_acceleration = 0.0;
    double c_longitudinal_jerk = 0.0;
    double c_time_taken = 0.0;
    double c_end_speed_deviation = 0.0;
    double c_longitudinal = 0.0;

    // obstacle costs
    double c_inv_dist_to_obstacles = 0.0;

    // final cost
    double cf = 0.0;

    FrenetPath(FrenetHyperparameters *fot_hp_);
    bool to_global_path(CubicSpline2D* csp);
    bool is_valid_path(const vector<Obstacle *> obstacles);
    bool is_collision(const vector<Obstacle *> obstacles);
    double inverse_distance_to_obstacles(
        const vector<Obstacle *> obstacles);

private:
    // Hyperparameters
    FrenetHyperparameters *fot_hp;
};

#endif //FRENET_OPTIMAL_TRAJECTORY_FRENETPATH_H
