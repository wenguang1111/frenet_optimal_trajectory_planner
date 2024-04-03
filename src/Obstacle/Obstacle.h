#ifndef FRENETOPTIMALTRAJECTORY_OBSTACLE_H
#define FRENETOPTIMALTRAJECTORY_OBSTACLE_H

#include <eigen3/Eigen/Dense>
#include "utils.h"

using namespace Eigen;

class Obstacle {
public:
    std::pair<Point_FP, Point_FP> bbox;
    Obstacle(Point_FP first_point, Point_FP second_point,
             double obstacle_clearance);
    bool isOverlap(Rectangle& car_outline);
    bool isPointNearObstacle(Vector2f &p, double radius);
    double getArea();
private:
    Vector2D subtract(Vector2D a, Vector2D b);
    float dotProduct(Vector2D a, Vector2D b);
    Vector2D perpendicular(Vector2D vector);
    bool isSeparated(Rectangle rect1, Rectangle rect2, Vector2D axis);
};

#endif //FRENETOPTIMALTRAJECTORY_OBSTACLE_H
