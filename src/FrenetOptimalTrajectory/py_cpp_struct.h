#ifndef FRENETOPTIMALTRAJECTORY_PY_CPP_STRUCT_H
#define FRENETOPTIMALTRAJECTORY_PY_CPP_STRUCT_H
#include <stddef.h>
#include "tool/fp_datatype.h"

const size_t MAX_PATH_LENGTH = 100;

struct FrenetInitialConditions_double {
    double s0; 
    double c_speed;
    double c_d;
    double c_d_d;
    double c_d_dd;
    double target_speed;
    double *wx; //goal position
    double *wy; //goal position
    int nw;
    double *o_llx;
    double *o_lly;
    double *o_urx;
    double *o_ury;
    int no;
};

struct FrenetReturnValues_double {
    int success;
    size_t path_length;
    double x_path[MAX_PATH_LENGTH];
    double y_path[MAX_PATH_LENGTH];
    double speeds[MAX_PATH_LENGTH];
    double ix[MAX_PATH_LENGTH];
    double iy[MAX_PATH_LENGTH];
    double iyaw[MAX_PATH_LENGTH];
    double d[MAX_PATH_LENGTH];
    double s[MAX_PATH_LENGTH];
    double speeds_x[MAX_PATH_LENGTH];
    double speeds_y[MAX_PATH_LENGTH];
    double params[MAX_PATH_LENGTH];
    double costs[MAX_PATH_LENGTH];
    double runtime;
};

struct FrenetHyperparameters_double {
    double max_speed;
    double max_accel;
    double max_curvature;
    double max_road_width_l;
    double max_road_width_r;
    double d_road_w; //delta_road_width
    double dt;
    double maxt;
    double mint;
    double d_t_s;
    double n_s_sample;
    double obstacle_clearance;
    double kd;
    double kv;
    double ka;
    double kj;
    double kt;
    double ko;
    double klat;
    double klon;
    int num_threads;
};

struct FrenetInitialConditions {
    fixp_s s0; 
    fixp_s_d c_speed;
    double c_d;
    double c_d_d;
    double c_d_dd;
    fixp_s_d target_speed;
    double *wx; //goal position
    double *wy; //goal position
    int nw;
    double *o_llx; //TODO: change to fixed point
    double *o_lly;
    double *o_urx;
    double *o_ury;
    int no;
};

struct FrenetHyperparameters {
    fixp_s_d max_speed;
    fixp_s_d max_accel;
    fixp_s_d max_curvature;
    fixp_s_d max_road_width_l;
    fixp_s_d max_road_width_r;
    fixp_s_d d_road_w; //delta_road_width
    fixp_s_d dt;
    fixp_s_d maxt;
    double mint;
    fixp_s_d d_t_s;
    fixp_s_d n_s_sample;
    double obstacle_clearance;
    fixp_s_d kd;
    fixp_s_d kv;
    fixp_s_d ka;
    fixp_s_d kj;
    fixp_s_d kt;
    fixp_s_d ko;
    fixp_s_d klat;
    fixp_s_d klon;
    int num_threads;
};
#endif //FRENETOPTIMALTRAJECTORY_PY_CPP_STRUCT_H
