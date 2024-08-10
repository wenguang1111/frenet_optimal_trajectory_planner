#include "FrenetPath.h"
#include "utils.h"
#ifdef USE_RECORDER
    #include "tool/recorder.h"
#endif

#include <algorithm>

const float COLLISION_CHECK_THRESHOLD = 6; // don't check unless within 6m

FrenetPath::FrenetPath(FrenetHyperparameters * fot_hp_) {
    fot_hp = fot_hp_;
}

// Convert the frenet path to global path in terms of x, y, yaw, velocity
bool FrenetPath::to_global_path(CubicSpline2D* csp) {
    float ix_, iy_, iyaw_, di, fx, fy, dx, dy;
    // Trajectory traj;
    // calc global positions
    for (size_t i = 0; i < s.size(); i++) {
        float s_i = static_cast<float>(s[i]);
        
        ix_ = csp->calc_x(s_i);
        iy_ = csp->calc_y(s_i);
        if (isnan(ix_) || isnan(iy_)) break;
        iyaw_ = csp->calc_yaw(s_i);
        ix.push_back(ix_);
        iy.push_back(iy_);
        iyaw.push_back(iyaw_);
        di = d[i];
        fx = ix_ + di * cos(iyaw_ + M_PI_2);
        fy = iy_ + di * sin(iyaw_ + M_PI_2);
        x.push_back(fx);
        y.push_back(fy);
        // traj.x.push_back(fx);
        // traj.y.push_back(fy);
        // #ifdef USE_RECORDER
        //     // Recorder::getInstance()->saveData<float>("FrenetPath::s_i", s_i);
        //     // Recorder::getInstance()->saveData<float>("FrenetPath::ix_", ix_);
        //     // Recorder::getInstance()->saveData<float>("FrenetPath::iy_", iy_);
        //     // Recorder::getInstance()->saveData<float>("FrenetPath::iyaw_", iyaw_);
        //     // Recorder::getInstance()->saveData<float>("FrenetPath::i", static_cast<float>(i));
        //     // Recorder::getInstance()->saveData<float>("FrenetPath::ix", ix.back());
        //     // Recorder::getInstance()->saveData<float>("FrenetPath::iy", iy.back());
        //     // Recorder::getInstance()->saveData<float>("FrenetPath::x", x.back());
        //     // Recorder::getInstance()->saveData<float>("FrenetPath::y", y.back());
        //     // Recorder::getInstance()->saveData<float>("FrenetPath::iyaw", iyaw.back());
        //     // Recorder::getInstance()->saveData<float>("FrenetPath::fx", fx);
        //     // Recorder::getInstance()->saveData<float>("FrenetPath::fy", fy);
        // #endif
    }

    // not enough points to construct a valid path
    if (x.size() <= 1) {
        return false;
    }

    // calc yaw and ds
    for (size_t i = 0; i < x.size() - 1; i++) {
        dx = static_cast<float>(x[i+1] - x[i]);
        dy = static_cast<float>(y[i+1] - y[i]);
        yaw.push_back(atan2(dy, dx));
        ds.push_back(hypot(dx, dy));
        // traj.yaw.push_back(yaw.back());
        // #ifdef USE_RECORDER
            // Recorder::getInstance()->saveData<float>("FrenetPath::dx", dx);
            // Recorder::getInstance()->saveData<float>("FrenetPath::dy", dy);
            // Recorder::getInstance()->saveData<float>("FrenetPath::yaw", yaw.back());
            // Recorder::getInstance()->saveData<float>("FrenetPath::ds", ds.back());
        // #endif
    }
    yaw.push_back(yaw.back());
    // traj.yaw.push_back(yaw.back());
    ds.push_back(ds.back());
    return true;
}

bool FrenetPath::is_valid_path() {
    if (any_of(s_d.begin(), s_d.end(),
            [this](auto i){return abs(i) > fot_hp->max_speed;})) {
        return false;
    }
    // max accel check
    else if (any_of(s_dd.begin(), s_dd.end(),
            [this](auto i){return abs(i) > fot_hp->max_accel;})) {
        return false;
    }
    // max curvature check
    else if (any_of(c.begin(), c.end(),
            [this](auto i){return abs(i) > fot_hp->max_curvature;})) {
        return false;
    }
    else {
        return true;
    }
}