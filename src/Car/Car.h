#ifndef FRENETOPTIMALTRAJECTORY_CAR_H
#define FRENETOPTIMALTRAJECTORY_CAR_H

#include "utils.h"

#include <vector>
#include <tuple>

using namespace std;

// Lincoln MKZ configuration
const float VEHICLE_LENGTH = 4.93;
const float VEHICLE_WIDTH = 1.86;

class Car {
public:
    Car() {
        length = VEHICLE_LENGTH;
        width = VEHICLE_WIDTH;
    };
    Car(Pose pose_): pose(pose_) {
        length = VEHICLE_LENGTH;
        width = VEHICLE_WIDTH;
    };
    void setPose(Pose p);
    Rectangle getOutline();
private:
    fixp_x length;
    fixp_x width;
    Pose pose; // x, y, yaw
};

#endif //FRENETOPTIMALTRAJECTORY_CAR_H
