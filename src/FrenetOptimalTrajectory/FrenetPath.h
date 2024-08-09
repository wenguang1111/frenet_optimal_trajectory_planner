#ifndef FRENET_OPTIMAL_TRAJECTORY_FRENETPATH_H
#define FRENET_OPTIMAL_TRAJECTORY_FRENETPATH_H

#include "CubicSpline2D.h"
#include "tool/fp_datatype.h"

#include <vector>
#include <tuple>

using namespace std;

struct Trajectory
{
    vector<float> x;
    vector<float> y;
    vector<float> yaw;
};

struct FrenetHyperparameters {
    float max_speed;
    float max_accel;
    float max_curvature;
    float max_road_width_l;
    float max_road_width_r;
    float d_road_w; //delta_road_width
    float dt;
    float maxt;
    float mint;
    float d_t_s;
    float n_s_sample;
    float obstacle_clearance;
    float kd;
    float kv;
    float ka;
    float kj;
    float kt;
    float ko;
    float klat;
    float klon;
    int num_threads;
};

class FrenetPath {
public:
    // Frenet attributes
    vector<float> t;          // time
    vector<float> d;          // lateral offset
    vector<float> d_d;        // lateral speed
    vector<float> d_dd;       // lateral acceleration
    vector<float> d_ddd;      // lateral jerk
    vector<float> s;          // s position along spline
    vector<float> s_d;        // s speed
    vector<float> s_dd;       // s acceleration
    vector<float> s_ddd;      // s jerk

    // Euclidean attributes
    vector<float> x;          // x position
    vector<float> y;          // y position
    vector<float> yaw;        // yaw in radc
    vector<float> ds;         // speed
    vector<float> c;          // curvature

    // Debug
    vector<float> ix;
    vector<float> iy;
    vector<float> iyaw;

    // Cost attributes
    // lateral costs
    float c_lateral_deviation = 0.0;
    float c_lateral_velocity = 0.0;
    float c_lateral_acceleration = 0.0;
    float c_lateral_jerk = 0.0;
    float c_lateral = 0.0;

    // longitudinal costs
    float c_longitudinal_acceleration = 0.0;
    float c_longitudinal_jerk = 0.0;
    float c_time_taken = 0.0;
    float c_end_speed_deviation = 0.0;
    float c_longitudinal = 0.0;

    // obstacle costs
    float c_inv_dist_to_obstacles = 0.0;

    // final cost
    float cf = 0.0;

    FrenetPath(FrenetHyperparameters *fot_hp_);
    Trajectory to_global_path(CubicSpline2D* csp);

private:
    // Hyperparameters
    FrenetHyperparameters *fot_hp;
};

#endif //FRENET_OPTIMAL_TRAJECTORY_FRENETPATH_H
