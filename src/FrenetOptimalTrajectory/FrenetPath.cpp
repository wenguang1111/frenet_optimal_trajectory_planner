#include "FrenetPath.h"
#include "utils.h"
#include "cordic.h"
#ifdef USE_RECORDER
    #include "tool/recorder.h"
#endif

#include <algorithm>

const short COLLISION_CHECK_THRESHOLD = 6; // don't check unless within 6m

FrenetPath::FrenetPath(FrenetHyperparameters_FP *fot_hp_){
    fot_hp = fot_hp_;
}

// Convert the frenet path to global path in terms of x, y, yaw, velocity
bool FrenetPath::to_global_path(CubicSpline2D* csp) {
    fixp_x ix_, iy_;
    fixp_yaw iyaw_;
    fixp_d di;
    fixp_x fx,dx;
    fixp_y fy,dy;
    // calc global positions
    for (size_t i = 0; i < s.size(); i++) {
        fixp_s s_i = s[i];
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
    }
    // not enough points to construct a valid path
    if (x.size() <= 1) {
        return false;
    }

    // calc yaw and ds
    for (size_t i = 0; i < x.size() - 1; i++) {
        dx = x[i+1] - x[i];
        dy = y[i+1] - y[i];
        yaw.push_back(cordic_atan<fixp_x>(dy, dx));
        ds.push_back(sqrt(dx*dx+dy*dy));
    }
    yaw.push_back(yaw.back());
    ds.push_back(ds.back());

    // calc curvature
    for (size_t i = 0; i < yaw.size() - 1; i++) {
        fixp_yaw dyaw = yaw[i+1] - yaw[i];
        if (dyaw > M_PI_2) {
            dyaw -= M_PI;
        } else if (dyaw < -M_PI_2) {
            dyaw += M_PI;
        }
        if(ds[i] == 0)
        {   //TODO: Here could be the reseaon of bug
            c.push_back(std::numeric_limits<fixp_c>::max());
        }
        else
        {
            c.push_back(cnl::quotient(dyaw, ds[i]));
        }
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
                fixp_x xp = x[i];
                fixp_y yp = y[i];
                fixp_yaw yawp = yaw[i];
                pose=Pose{xp, yp, yawp};
                car.setPose(pose);
                car_outline = car.getOutline();
                if(obstacle->isOverlap(car_outline))
                {
                    return true;
                }
            }
        }
    }
    return false;
}

// calculate the sum of 1 / distance_to_obstacle
fixp_x
FrenetPath::inverse_distance_to_obstacles(
    const vector<Obstacle *> obstacles) {
    fixp_x total_inverse_distance = 0.0;

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

            if(closest==0)
            {
                total_inverse_distance = std::numeric_limits<fixp_x>::max();
                break;
            }
            else{
                total_inverse_distance += cnl::quotient(fixp_x(1.0) , closest);
            }
        }
    }
    return total_inverse_distance;
}