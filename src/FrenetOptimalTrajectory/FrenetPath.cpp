#include "FrenetPath.h"
#include "utils.h"
#include "tool/fp_datatype.h"
#ifdef USE_RECORDER
    #include "tool/recorder.h"
#endif

#include <algorithm>

const float COLLISION_CHECK_THRESHOLD = 6; // don't check unless within 6m

FrenetPath::FrenetPath(FrenetHyperparameters *fot_hp_) {
    fot_hp = fot_hp_;
}

// Convert the frenet path to global path in terms of x, y, yaw, velocity
bool FrenetPath::to_global_path(CubicSpline2D* csp) {
    double ix_, iy_, iyaw_, di, fx, fy, dx, dy;
    // calc global positions
    for (size_t i = 0; i < s.size(); i++) {
        double s_i = static_cast<double>(s[i]);
        ix_ = csp->calc_x(s_i);
        iy_ = csp->calc_y(s_i);
        if (isnan(ix_) || isnan(iy_)) break;

        iyaw_ = csp->calc_yaw(s_i);
        ix.push_back(ix_);
        iy.push_back(iy_);
        iyaw.push_back(iyaw_);
        di = static_cast<double>(d[i]);
        fx = ix_ + di * cos(iyaw_ + M_PI_2);
        fy = iy_ + di * sin(iyaw_ + M_PI_2);
        x.push_back(fx);
        y.push_back(fy);
        // #ifdef USE_RECORDER
        //     Recorder::getInstance()->saveData<double>("FrenetOptimalTrajectory::calc_frenet_paths()::FrenetPath::x", x.back());
        //     Recorder::getInstance()->saveData<double>("FrenetOptimalTrajectory::calc_frenet_paths()::FrenetPath::y", y.back());
        //     Recorder::getInstance()->saveData<double>("FrenetOptimalTrajectory::calc_frenet_paths()::FrenetPath::ix_", ix_);
        //     Recorder::getInstance()->saveData<double>("FrenetOptimalTrajectory::calc_frenet_paths()::FrenetPath::iy_", iy_);
        //     Recorder::getInstance()->saveData<double>("FrenetOptimalTrajectory::calc_frenet_paths()::FrenetPath::iyaw_", iyaw_);
        // #endif
    }

    // not enough points to construct a valid path
    if (x.size() <= 1) {
        return false;
    }

    // calc yaw and ds
    for (size_t i = 0; i < x.size() - 1; i++) {
        dx = static_cast<double>(x[i+1] - x[i]);
        dy = static_cast<double>(y[i+1] - y[i]);
        yaw.push_back(atan2(dy, dx));
        ds.push_back(hypot(dx, dy));
        // #ifdef USE_RECORDER
        //     Recorder::getInstance()->saveData<double>("FrenetOptimalTrajectory::calc_frenet_paths()::FrenetPath::yaw", yaw.back());
        //     Recorder::getInstance()->saveData<double>("FrenetOptimalTrajectory::calc_frenet_paths()::FrenetPath::ds", ds.back());
        //     Recorder::getInstance()->saveData<double>("FrenetOptimalTrajectory::calc_frenet_paths()::FrenetPath::dx", dx);
        //     Recorder::getInstance()->saveData<double>("FrenetOptimalTrajectory::calc_frenet_paths()::FrenetPath::dy", dy);
        // #endif
    }
    yaw.push_back(yaw.back());
    ds.push_back(ds.back());


    // calc curvature
    for (size_t i = 0; i < yaw.size() - 1; i++) {
        double dyaw = static_cast<double>(yaw[i+1] - yaw[i]);
        if (dyaw > M_PI_2) {
            dyaw -= M_PI;
        } else if (dyaw < -M_PI_2) {
            dyaw += M_PI;
        }
        c.push_back(dyaw / ds[i]);
        //c.push_back(dyaw / fot_hp->dt);
        // #ifdef USE_RECORDER
        //     Recorder::getInstance()->saveData<double>("FrenetOptimalTrajectory::calc_frenet_paths()::FrenetPath::c", c.back());
        //     Recorder::getInstance()->saveData<double>("FrenetOptimalTrajectory::calc_frenet_paths()::FrenetPath::dyaw", dyaw);
        // #endif
    }

    return true;
}

// Validate the calculated frenet paths against threshold speed, acceleration,
// curvature and collision checks
bool FrenetPath::is_valid_path(const vector<Obstacle *> obstacles) {
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
    // collision check
    else if (is_collision(obstacles)) {
        return false;
    }
    else {
        return true;
    }
}

// check path for collision with obstacles
bool FrenetPath::is_collision(const vector<Obstacle *> obstacles) {
    // no obstacles
    if (obstacles.empty()) {
        return false;
    }

    Pose pose;
    Car car = Car();
    Vector2f p1, p2;
    Rectangle car_outline;
    // iterate over all obstacles
    for (auto obstacle : obstacles) {
        fixp_x llx = obstacle->bbox.first.x;
        fixp_x lly = obstacle->bbox.first.y;
        fixp_x urx = obstacle->bbox.second.x;
        fixp_x ury = obstacle->bbox.second.y;
        // #ifdef USE_RECORDER
        //     Recorder::getInstance()->saveData<double>("FrenetOptimalTrajectory::calc_frenet_paths()::FrenetPath::llx", llx);
        //     Recorder::getInstance()->saveData<double>("FrenetOptimalTrajectory::calc_frenet_paths()::FrenetPath::lly", lly);
        //     Recorder::getInstance()->saveData<double>("FrenetOptimalTrajectory::calc_frenet_paths()::FrenetPath::urx", urx);
        //     Recorder::getInstance()->saveData<double>("FrenetOptimalTrajectory::calc_frenet_paths()::FrenetPath::ury", ury);
        // #endif

        for (size_t i = 0; i < x.size(); i++) {
            fixp_x d1 = norm_FP(llx - x[i], lly - y[i]);
            fixp_x d2 = norm_FP(llx - x[i], ury - y[i]);
            fixp_x d3 = norm_FP(urx - x[i], ury - y[i]);
            fixp_x d4 = norm_FP(urx - x[i], lly - y[i]);
            // #ifdef USE_RECORDER
            //     Recorder::getInstance()->saveData<double>("FrenetOptimalTrajectory::calc_frenet_paths()::FrenetPath::d1", d1);
            //     Recorder::getInstance()->saveData<double>("FrenetOptimalTrajectory::calc_frenet_paths()::FrenetPath::d2", d2);
            //     Recorder::getInstance()->saveData<double>("FrenetOptimalTrajectory::calc_frenet_paths()::FrenetPath::d3", d3);
            //     Recorder::getInstance()->saveData<double>("FrenetOptimalTrajectory::calc_frenet_paths()::FrenetPath::d4", d4);
            // #endif

            fixp_x closest = min({d1, d2, d3, d4});
            // only check for collision if one corner of bounding box is
            // within COLLISION_CHECK_THRESHOLD of waypoint
            if (closest <= COLLISION_CHECK_THRESHOLD) {
                double xp = static_cast<double>(x[i]);
                double yp = static_cast<double>(y[i]);
                double yawp = static_cast<double>(yaw[i]);
                pose.assign({xp, yp, yawp});
                car.setPose(pose);
                car_outline = car.getOutline();
                if(obstacle->isOverlap(car_outline))
                {
                    return true;
                }
            }
        }
    }

    // no collisions
    return false;
}

// calculate the sum of 1 / distance_to_obstacle
fixp_inverse_distanceToObstacles
FrenetPath::inverse_distance_to_obstacles(
    const vector<Obstacle *> obstacles) {
    fixp_inverse_distanceToObstacles total_inverse_distance = 0.0;

    for (auto obstacle : obstacles) {
        fixp_x llx = obstacle->bbox.first.x;
        fixp_x lly = obstacle->bbox.first.y;
        fixp_x urx = obstacle->bbox.second.x;
        fixp_x ury = obstacle->bbox.second.y;

        for (size_t i = 0; i < x.size(); i++) {
            fixp_x d1 = norm_FP(llx - x[i], lly - y[i]);
            fixp_x d2 = norm_FP(llx - x[i], ury - y[i]);
            fixp_x d3 = norm_FP(urx - x[i], ury - y[i]);
            fixp_x d4 = norm_FP(urx - x[i], lly - y[i]);

            fixp_x closest = min({d1, d2, d3, d4});
            total_inverse_distance += 1.0 / closest;
        }
    }
    return total_inverse_distance;
}
