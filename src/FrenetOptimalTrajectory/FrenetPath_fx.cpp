#include "FrenetPath_fx.h"
#include "utils.h"
#include "cordic.h"
#ifdef USE_RECORDER
    #include "tool/recorder.h"
#endif

#include <algorithm>

const short COLLISION_CHECK_THRESHOLD = 6; // don't check unless within 6m

FrenetPath::FrenetPath(FrenetHyperparameters_fx *fot_hp_){
    fot_hp = fot_hp_;
}

// Convert the frenet path to global path in terms of x, y, yaw, velocity
Trajectory_fx FrenetPath::to_global_path(CubicSpline2D_fx* csp) {
    fp_type ix_, iy_;
    fp_type iyaw_;
    fp_type di;
    fp_type fx,dx;
    fp_type fy,dy;
    // calc global positions
    for (size_t i = 0; i < s.size(); i++) {
        fp_type s_i = s[i];
        ix_ = csp->calc_x(s_i);
        iy_ = csp->calc_y(s_i);
        //if (isnan(ix_) || isnan(iy_)) break; //FIXME: maybe create bug after deletion
        if (csp->isValidPath()!=true) break; 
        iyaw_ = csp->calc_yaw(s_i);
        ix.push_back(ix_);
        iy.push_back(iy_);
        iyaw.push_back(iyaw_);
        di = d[i];
        fx = ix_ + di * cordic_cos(iyaw_ + M_PI_2);
        fy = iy_ + di * cordic_sin(iyaw_ + M_PI_2);
        x.push_back(fx);
        y.push_back(fy);
        #ifdef USE_RECORDER
            // Recorder::getInstance()->saveData<float>("i", static_cast<float>(i));
            // Recorder::getInstance()->saveData<float>("ix", static_cast<float>(ix.back()));
            // Recorder::getInstance()->saveData<float>("iy", static_cast<float>(iy.back()));
            Recorder::getInstance()->saveData<float>("x", static_cast<float>(x.back()));
            Recorder::getInstance()->saveData<float>("y", static_cast<float>(y.back()));
            // Recorder::getInstance()->saveData<float>("iyaw", static_cast<float>(iyaw.back()));
        #endif
    }
    // // not enough points to construct a valid path
    // if (x.size() <= 1) {
    //     return false;
    // }

    // calc yaw and ds
    for (size_t i = 0; i < x.size() - 1; i++) {
        dx = x[i+1] - x[i];
        dy = y[i+1] - y[i];
        yaw.push_back(cordic_atan<fp_type>(dy, dx));
        ds.push_back(norm<fp_type>(dx,dy));
        #ifdef USE_RECORDER
            Recorder::getInstance()->saveData<float>("yaw",  static_cast<float>(yaw.back()));
            // Recorder::getInstance()->saveData<float>("ds",  static_cast<float>(ds.back()));
        #endif
    }
    yaw.push_back(yaw.back());
    ds.push_back(ds.back());
    #ifdef USE_RECORDER
        Recorder::getInstance()->saveData<float>("yaw", static_cast<float>(yaw.back()));
        // Recorder::getInstance()->saveData<float>("ds", static_cast<float>(ds.back()));
    #endif
    Trajectory_fx traj;
    traj.x=x;
    traj.y=y;
    traj.yaw=yaw;
    return traj;
}