#include "Obstacle.h"

#include <QLine>

using namespace Eigen;
using namespace std;

Obstacle::Obstacle(Vector2f first_point, Vector2f second_point, double obstacle_clearance)
{
    // Get topLeft and bottomRight points from the given points.
    Vector2f tmp;
    if (first_point.x() > second_point.x() && first_point.y() > second_point.y()) {
        tmp = first_point;
        first_point = second_point;
        second_point = tmp;
    } else if (first_point.x() < second_point.x() && first_point.y() > second_point.y()) {
        float height = first_point.y() - second_point.y();
        first_point.y() -= height;
        second_point.y() += height;
    } else if (first_point.x() > second_point.x() && first_point.y() < second_point.y()) {
        float length = first_point.x() - second_point.x();
        first_point.x() -= length;
        second_point.x() += length;
    }
    first_point.x() -= obstacle_clearance;
    first_point.y() -= obstacle_clearance;
    second_point.x() += obstacle_clearance;
    second_point.y() += obstacle_clearance;

    bbox.first.x() = first_point.x();
    bbox.first.y() = first_point.y();
    bbox.second.x() = second_point.x();
    bbox.second.y() = second_point.y();
}

//Separating Axis Theorem Algorithmus
bool Obstacle::isOverlap(Rectangle& car_outline)
{
    Rectangle obstacle = {{{bbox.first.x(),bbox.first.y()},{bbox.second.x(), bbox.first.y()}, {bbox.second.x(), bbox.second.y()}, {bbox.first.x(), bbox.second.y()}}};

    Vector2D axes[4] ={
        perpendicular(subtract(car_outline.points[1], car_outline.points[0])),
        perpendicular(subtract(car_outline.points[2], car_outline.points[1])),
        perpendicular(subtract(obstacle.points[1], obstacle.points[0])),
        perpendicular(subtract(obstacle.points[2], obstacle.points[1]))
    };

    for (int i = 0; i < 4; i++) {
        if(isSeparated(car_outline, obstacle, axes[i])){
            return false;
        }
    }
    return true;
}

bool Obstacle::isSeparated(Rectangle rect1, Rectangle rect2, Vector2D axis) {
    float min1, max1, min2, max2;

    min1 = max1 = dotProduct(rect1.points[0], axis);
    min2 = max2 = dotProduct(rect2.points[0], axis);

    for (int i = 1; i < 4; i++) {
        float projection = dotProduct(rect1.points[i], axis);
        if (projection < min1) min1 = projection;
        if (projection > max1) max1 = projection;

        projection = dotProduct(rect2.points[i], axis);
        if (projection < min2) min2 = projection;
        if (projection > max2) max2 = projection;
    }

    if (max1 < min2 || max2 < min1) {
        return true; // Separated
    }

    return false; // Not separated
}

Vector2D Obstacle::subtract(Vector2D a, Vector2D b) {
    Vector2D result;
    result.x = a.x - b.x;
    result.y = a.y - b.y;
    return result;
}

float Obstacle::dotProduct(Vector2D a, Vector2D b) {
    return a.x * b.x + a.y * b.y;
}

Vector2D Obstacle::perpendicular(Vector2D vector) {
    Vector2D result;
    result.x = -vector.y;
    result.y = vector.x;
    return result;
}

bool Obstacle::isPointNearObstacle(Vector2f &p, double radius) {
    double dist_to_ll, dist_to_lr, dist_to_ul, dist_to_ur;
    dist_to_ll = sqrt(pow(bbox.first.x() - p.x(), 2) +
                      pow(bbox.first.y() - p.y(), 2));
    dist_to_lr = sqrt(pow(bbox.second.x() - p.x(), 2) +
                      pow(bbox.first.y() - p.y(), 2));
    dist_to_ul = sqrt(pow(bbox.first.x() - p.x(), 2) +
                      pow(bbox.second.y() - p.y(), 2));
    dist_to_ur = sqrt(pow(bbox.second.x() - p.x(), 2) +
                      pow(bbox.second.y() - p.y(), 2));
    if (dist_to_ll <= radius || dist_to_lr <= radius ||
        dist_to_ul <= radius || dist_to_ur <= radius ) {
        return true;
    }
    return false;
}