#ifndef DEFENSE_H
#define DEFENSE_H

#include <vector>
#include <cmath>

//Point2f struct for storing 2D coordinates
struct Point2f {
    float x;
    float y;

    Point2f() : x(0), y(0) {}
    Point2f(float _x, float _y) : x(_x), y(_y) {}

    Point2f operator+(const Point2f& other) const {
        return Point2f(x + other.x, y + other.y);
    }

    Point2f operator-(const Point2f& other) const {
        return Point2f(x - other.x, y - other.y);
    }

    Point2f operator*(float scalar) const {
        return Point2f(x * scalar, y * scalar);
    }

    float distanceTo(const Point2f& other) const {
        return sqrt((x - other.x) * (x - other.x) + (y - other.y) * (y - other.y));
    }
};

//distance between two points
float distance(const Point2f& a, const Point2f& b);

//check if an obstacle is blocking the path between robot and enemy
bool isLineOfSightBlocked(Point2f from, Point2f to, const std::vector<Point2f>& obstacles, float radius);

//choose a perpendicular evasive direction for the robot
Point2f getEvadeTarget(Point2f myPos, Point2f opponentPos, const std::vector<Point2f>& obstacles);

//main function to run defense mode logic
void defenseMode(Point2f myPosition, Point2f opponentPosition, const std::vector<Point2f>& obstacles, float obstacleRadius);

#endif
