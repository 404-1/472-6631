#ifndef defense_h
#define defense_h

#include <vector>
#include <cmath>
#include <iostream>

// 2D Point structure for representing positions or vectors in the arena
struct Point2f {
    float x;
    float y;

    Point2f() : x(0), y(0) {}
    Point2f(float _x, float _y) : x(_x), y(_y) {}

    //operator overloads for basic vector math
    Point2f operator+(const Point2f& other) const {
        return Point2f(x + other.x, y + other.y);
    }

    Point2f operator-(const Point2f& other) const {
        return Point2f(x - other.x, y - other.y);
    }

    Point2f operator*(float scalar) const {
        return Point2f(x * scalar, y * scalar);
    }

    //Euclidean distance to another point
    float distanceTo(const Point2f& other) const {
        return sqrt((x - other.x) * (x - other.x) + (y - other.y) * (y - other.y));
    }
};

// This calculates the euclidean distance between two (x, y) positions
void getDistance(float x1, float y1, float x2, float y2, double& distance);

// This calculates the angle (in deg) between two points, useful for heading or aiming
void getAngle(float x1, float y1, float x2, float y2, double& angle);

// determines whether the direct path from `from` to `to` is blocked by any obstacle
bool isLineOfSightBlocked(Point2f from, Point2f to, const std::vector<Point2f>& obstacles, float radius);

// chooses the best hiding spot behind the nearest obstacle, away from the opponent
Point2f getEvadeTarget(Point2f myPos, Point2f opponentPos, const std::vector<Point2f>& obstacles);

// This will command the robot to move toward a target while adjusting for angle and proximity
void moveTo(Point2f target, Point2f myPosition, Point2f opponentPosition,
    const std::vector<Point2f>& obstacles, float obstacleRadius,
    float& pw_l, float& pw_r);

// Just stops the robot in place by zeroing out wheel speeds
void stop();

// The main defensive logic: hides the robot behind cover if visible to opponent
void defenseMode(Point2f myPosition, Point2f opponentPosition,
    const std::vector<Point2f>& obstacles, float obstacleRadius,
    float& pw_l, float& pw_r);

#endif
