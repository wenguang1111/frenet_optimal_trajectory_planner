#ifndef FRENETOPTIMALTRAJECTORY_OBSTACLE_H
#define FRENETOPTIMALTRAJECTORY_OBSTACLE_H

#include <eigen3/Eigen/Dense>
#include "utils.h"
#include "tool/fp_datatype.h"

using namespace Eigen;

class Obstacle {
public:
    std::pair<Point, Point> bbox;
    Obstacle(Point first_point, Point second_point,
             fixp_obstacle_clearance obstacle_clearance);
    bool isOverlap(Rectangle& car_outline);
private:
    Vector2D subtract(Vector2D a, Vector2D b);
    fixp_x dotProduct(Vector2D a, Vector2D b);
    Vector2D perpendicular(Vector2D vector);
    bool isSeparated(Rectangle rect1, Rectangle rect2, Vector2D axis);
};

#endif //FRENETOPTIMALTRAJECTORY_OBSTACLE_H
