#ifndef FRENET_OPTIMAL_TRAJECTORY_FRENETPATH_H
#define FRENET_OPTIMAL_TRAJECTORY_FRENETPATH_H

#include "CubicSpline2D_fx.h"
#include "tool/fp_datatype.h"

#include <vector>
#include <tuple>

using namespace std;

struct Trajectory_fx
{
    vector<fp_type> x;
    vector<fp_type> y;
    vector<fp_type> yaw;
};

struct FrenetHyperparameters_fx {
    fp_type max_speed;
    fp_type max_accel;
    fp_type max_curvature;
    fp_type max_road_width_l;
    fp_type max_road_width_r;
    fp_type d_road_w; //delta_road_width
    fp_type dt;
    fp_type maxt;
    fp_type mint;
    fp_type d_t_s;
    fp_type n_s_sample;
    fp_type obstacle_clearance;
    fp_type kd;
    fp_type kv;
    fp_type ka;
    fp_type kj;
    fp_type kt;
    fp_type ko;
    fp_type klat;
    fp_type klon;
    int num_threads;
};

class FrenetPath {
public:
    // Frenet attributes
    vector<fp_type> t;          // time
    vector<fp_type> d;          // lateral offset
    vector<fp_type> d_d;        // lateral speed
    vector<fp_type> d_dd;       // lateral acceleration
    vector<fp_type> d_ddd;      // lateral jerk
    vector<fp_type> s;          // s position along spline
    vector<fp_type> s_d;        // s speed
    vector<fp_type> s_dd;       // s acceleration
    vector<fp_type> s_ddd;      // s jerk

    // Euclidean attributes
    vector<fp_type> x;          // x position
    vector<fp_type> y;          // y position
    vector<fp_type> yaw;        // yaw in radc
    vector<fp_type> ds;         // speed
    vector<fp_type> c;          // curvature

    // Debug
    vector<fp_type> ix;
    vector<fp_type> iy;
    vector<fp_type> iyaw;

    // Cost attributes
    // lateral costs
    fp_type c_lateral_deviation = 0.0;
    fp_type c_lateral_velocity = 0.0;
    fp_type c_lateral_acceleration = 0.0;
    fp_type c_lateral_jerk = 0.0;
    fp_type c_lateral = 0.0;

    // longitudinal costs
    fp_type c_longitudinal_acceleration = 0.0;
    fp_type c_longitudinal_jerk = 0.0;
    fp_type c_time_taken = 0.0;
    fp_type c_end_speed_deviation = 0.0;
    fp_type c_longitudinal = 0.0;

    // obstacle costs
    fp_type c_inv_dist_to_obstacles = 0.0;

    // final cost
    fp_type cf = 0.0;

    FrenetPath(FrenetHyperparameters_fx* fot_hp_);
    Trajectory_fx to_global_path(CubicSpline2D_fx* csp);

private:
    // Hyperparameters
    FrenetHyperparameters_fx *fot_hp;
};

#endif //FRENET_OPTIMAL_TRAJECTORY_FRENETPATH_H
