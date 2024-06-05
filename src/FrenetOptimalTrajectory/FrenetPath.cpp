#include "FrenetPath.h"
#include "utils.h"
#include "tool/fp_datatype.h"
#include "cordic.h"
#ifdef USE_RECORDER
    #include "tool/recorder.h"
#endif

#include <algorithm>

const float COLLISION_CHECK_THRESHOLD = 6; // don't check unless within 6m

FrenetPath::FrenetPath(FrenetHyperparameters_FP *fot_hp_) {
    fot_hp = fot_hp_;
}

// Convert the frenet path to global path in terms of x, y, yaw, velocity
bool FrenetPath::to_global_path(CubicSpline2D* csp) {
    float ix_, iy_, iyaw_, di, fx, fy, dx, dy;
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
        di = static_cast<float>(d[i]);
        fx = ix_ + di * cordic_cos(iyaw_ + M_PI_2);
        fy = iy_ + di * cordic_sin(iyaw_ + M_PI_2);
        // fx = ix_ + di * cos(iyaw_ + M_PI_2);
        // fy = iy_ + di * sin(iyaw_ + M_PI_2);
        x.push_back(fx);
        y.push_back(fy);
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
    }
    yaw.push_back(yaw.back());
    ds.push_back(ds.back());


    // calc curvature
    for (size_t i = 0; i < yaw.size() - 1; i++) {
        float dyaw = static_cast<float>(yaw[i+1] - yaw[i]);
        if (dyaw > M_PI_2) {
            dyaw -= M_PI;
        } else if (dyaw < -M_PI_2) {
            dyaw += M_PI;
        }
        c.push_back(dyaw / ds[i]);
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
        fixp_y lly = obstacle->bbox.first.y;
        fixp_x urx = obstacle->bbox.second.x;
        fixp_y ury = obstacle->bbox.second.y;

        for (size_t i = 0; i < x.size(); i++) {
            fixp_x d1 = norm(llx - x[i], lly - y[i]);
            fixp_x d2 = norm(llx - x[i], ury - y[i]);
            fixp_x d3 = norm(urx - x[i], ury - y[i]);
            fixp_x d4 = norm(urx - x[i], lly - y[i]);

            fixp_x closest = min({d1, d2, d3, d4});
            // only check for collision if one corner of bounding box is
            // within COLLISION_CHECK_THRESHOLD of waypoint
            if (closest <= COLLISION_CHECK_THRESHOLD) {
                float xp = static_cast<float>(x[i]);
                float yp = static_cast<float>(y[i]);
                float yawp = static_cast<float>(yaw[i]);
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
float
FrenetPath::inverse_distance_to_obstacles(
    const vector<Obstacle *> obstacles) {
    float total_inverse_distance = 0.0;

    for (auto obstacle : obstacles) {
        fixp_x llx = obstacle->bbox.first.x;
        fixp_y lly = obstacle->bbox.first.y;
        fixp_x urx = obstacle->bbox.second.x;
        fixp_y ury = obstacle->bbox.second.y;

        for (size_t i = 0; i < x.size(); i++) {
            fixp_x d1 = norm(llx - x[i], lly - y[i]);
            fixp_x d2 = norm(llx - x[i], ury - y[i]);
            fixp_x d3 = norm(urx - x[i], ury - y[i]);
            fixp_x d4 = norm(urx - x[i], lly - y[i]);

            fixp_x closest = min({d1, d2, d3, d4});
            total_inverse_distance += static_cast<float>(1.0 / closest);
        }
    }
    return total_inverse_distance;
}
