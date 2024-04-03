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
    vector<float> t;          // time
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
    vector<fixp_x> y;          // y position
    vector<fixp_yaw> yaw;        // yaw in radc
    vector<fixp_ds> ds;         // speed
    vector<fixp_c> c;          // curvature

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
