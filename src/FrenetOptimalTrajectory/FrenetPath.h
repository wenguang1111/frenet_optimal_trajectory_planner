#ifndef FRENET_OPTIMAL_TRAJECTORY_FRENETPATH_H
#define FRENET_OPTIMAL_TRAJECTORY_FRENETPATH_H

#include "py_cpp_struct.h"
#include "CubicSpline2D.h"
#include "Obstacle.h"
#include "Car.h"
#include "tool/fp_datatype.h"

#include <eigen3/Eigen/Dense>
#include <vector>
#include <tuple>

using namespace std;
using namespace Eigen;

class FrenetPath {
public:
    // Frenet attributes
    vector<fixp_maxt> t;          // time
    vector<fixp_d> d;          // lateral offset
    vector<fixp_d_d> d_d;        // lateral speed
    vector<fixp_d_dd> d_dd;       // lateral acceleration
    vector<fixp_d_ddd> d_ddd;      // lateral jerk
    vector<fixp_s> s;          // s position along spline
    vector<fixp_s_d> s_d;        // s speed
    vector<fixp_s_dd> s_dd;       // s acceleration
    vector<fixp_s_ddd> s_ddd;      // s jerk

    // Euclidean attributes
    vector<fixp_x> x;          // x position
    vector<fixp_y> y;          // y position
    vector<fixp_yaw> yaw;        // yaw in radc
    vector<fixp_ds> ds;         // speed
    vector<fixp_c> c;          // curvature

    // Debug
    vector<float> ix;
    vector<float> iy;
    vector<float> iyaw;

    // Cost attributes
    // lateral costs
    fixp_lateral_acceleration c_lateral_deviation = 0.0;
    fixp_lateral_velocity c_lateral_velocity = 0.0;
    fixp_lateral_acceleration c_lateral_acceleration = 0.0;
    fixp_lateral_jerk c_lateral_jerk = 0.0;
    fixp_c_lateral c_lateral = 0.0;

    // longitudinal costs
    fixp_longitudinal_acceleration c_longitudinal_acceleration = 0.0;
    fixp_longitudinal_jerk c_longitudinal_jerk = 0.0;
    fixp_maxt c_time_taken = 0.0;
    fixp_end_speed_deviation c_end_speed_deviation = 0.0;
    fixp_longitudinal c_longitudinal = 0.0;

    // obstacle costs
    fixp_inv_dist_to_obstacles c_inv_dist_to_obstacles = 0.0;

    // final cost
    fixp_cf cf = 0.0;

    FrenetPath(FrenetHyperparameters_FP *fot_hp_);
    bool to_global_path(CubicSpline2D* csp);
    bool is_valid_path(const vector<Obstacle *> obstacles);
    bool is_collision(const vector<Obstacle *> obstacles);
    float inverse_distance_to_obstacles(
        const vector<Obstacle *> obstacles);

private:
    // Hyperparameters
    FrenetHyperparameters_FP *fot_hp;
};

#endif //FRENET_OPTIMAL_TRAJECTORY_FRENETPATH_H
