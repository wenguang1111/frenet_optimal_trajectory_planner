#include "Car.h"
#include "cordic.h"

// Set the pose of the car
void Car::setPose(Pose p) {
    pose = p;
}

#include <iostream>
// Compute the outline of the car given its current pose
Rectangle Car::getOutline() {
    fixp_x x;
    fixp_y y;
    fixp_yaw yaw;
    fixp_x tail_x, tail_y, head_x, head_y;
    Vector2D tail_l, tail_r;
    Vector2D head_l, head_r;

    x = pose.x;
    y = pose.y;
    yaw = pose.yaw;
    
    tail_x = x - cordic_cos(yaw) * length * 0.5;
    tail_y = y - cordic_sin(yaw) * length * 0.5;
    tail_l.x = tail_x + fixp_x(cnl::quotient(cordic_cos(yaw + M_PI_2) * width, fixp_x(2.0)));
    tail_l.y = tail_y + fixp_y(cnl::quotient(cordic_sin(yaw + M_PI_2) * width, fixp_x(2.0)));
    tail_r.x = tail_x + fixp_x(cnl::quotient(cordic_cos(yaw - M_PI_2) * width, fixp_x(2.0)));
    tail_r.y = tail_y + fixp_y(cnl::quotient(cordic_sin(yaw - M_PI_2) * width, fixp_x(2.0)));

    head_x = x + cordic_cos(yaw) * length * 0.5;
    head_y = y + cordic_sin(yaw) * length * 0.5;
    head_l.x = head_x + fixp_x(cnl::quotient(cordic_cos(yaw + M_PI_2) * width, fixp_x(2.0)));
    head_l.y = head_y + fixp_y(cnl::quotient(cordic_sin(yaw + M_PI_2) * width, fixp_y(2.0)));
    head_r.x = head_x + fixp_x(cnl::quotient(cordic_cos(yaw - M_PI_2) * width, fixp_x(2.0)));
    head_r.y = head_y + fixp_y(cnl::quotient(cordic_sin(yaw - M_PI_2) * width, fixp_y(2.0)));

    Rectangle outline ={tail_l, tail_r, head_r, head_l};
    return outline;
}