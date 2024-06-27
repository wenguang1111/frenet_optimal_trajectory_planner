#ifndef FRENETOPTIMALTRAJECTORY_PY_CPP_STRUCT_H
#define FRENETOPTIMALTRAJECTORY_PY_CPP_STRUCT_H
#include <stddef.h>
#include "tool/fp_datatype.h"

const size_t MAX_PATH_LENGTH = 100;

struct FrenetInitialConditions_Float {
    float s0; 
    float c_speed;
    float c_d;
    float c_d_d;
    float c_d_dd;
    float target_speed;
    float *wx; //goal position
    float *wy; //goal position
    int nw;
    float *o_llx;
    float *o_lly;
    float *o_urx;
    float *o_ury;
    int no;
};

struct FrenetReturnValues_FLoat {
    int success;
    size_t path_length;
    float x_path[MAX_PATH_LENGTH];
    float y_path[MAX_PATH_LENGTH];
    float speeds[MAX_PATH_LENGTH];
    float ix[MAX_PATH_LENGTH];
    float iy[MAX_PATH_LENGTH];
    float iyaw[MAX_PATH_LENGTH];
    float d[MAX_PATH_LENGTH];
    float s[MAX_PATH_LENGTH];
    float speeds_x[MAX_PATH_LENGTH];
    float speeds_y[MAX_PATH_LENGTH];
    float params[MAX_PATH_LENGTH];
    float costs[MAX_PATH_LENGTH];
    float runtime;
};

struct FrenetHyperparameters_Float {
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

struct FrenetInitialConditions_FP {
    fixp_s s0; 
    fixp_s_d c_speed;
    fixp_c_d c_d;
    fixp_c_d c_d_d;
    fixp_c_d c_d_dd;
    fixp_target_speed target_speed;
    fixp_x *wx; //goal position
    fixp_y *wy; //goal position
    int nw;
    fixp_x *o_llx; //TODO: change to fixed point
    fixp_y *o_lly;
    fixp_x *o_urx;
    fixp_y *o_ury;
    int no;
};

struct FrenetHyperparameters_FP {
    fixp_s_d max_speed;
    fixp_s_d max_accel;
    fixp_s_d max_curvature;
    fixp_s_d max_road_width_l;
    fixp_s_d max_road_width_r;
    fixp_s_d d_road_w; //delta_road_width
    fixp_dt dt;
    fixp_maxt maxt;
    fixp_maxt mint;
    fixp_s_d d_t_s;
    fixp_s_d n_s_sample;
    fixp_obstacle_clearance obstacle_clearance;
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
