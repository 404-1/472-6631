#include "defense.h"
#include <iostream>
#include <vector>
#include <cmath>
#include "vision.h"
#include "image_transfer.h"

//macro for safe distance and max speed (can be adjusted)
#define SAFE_DISTANCE 50.0f
#define MAX_SPEED 5.0f

//function prototypes
bool isLineOfSightBlocked(Point2f from, Point2f to, const std::vector<Point2f>& obstacles, float radius);
Point2f getEvadeTarget(Point2f myPos, Point2f opponentPos, const std::vector<Point2f>& obstacles);
void moveTo(Point2f target);
void stop();

//defense mode logic
void defenseMode(Point2f myPosition, Point2f opponentPosition, const std::vector<Point2f>& obstacles, float obstacleRadius) {
    //check if opponent has line of sight to us
    bool exposed = !isLineOfSightBlocked(opponentPosition, myPosition, obstacles, obstacleRadius);

    if (exposed) {
        //we're exposed, time to move to cover
        Point2f evadeTarget = getEvadeTarget(myPosition, opponentPosition, obstacles);
        moveTo(evadeTarget);
    } else {
        //already in cover
        stop();
    }
}

//check if any obstacle blocks line of sight
bool isLineOfSightBlocked(Point2f from, Point2f to, const std::vector<Point2f>& obstacles, float radius) {
    for (const auto& obs : obstacles) {
        float num = abs((to.y - from.y) * obs.x - (to.x - from.x) * obs.y + to.x * from.y - to.y * from.x);
        float den = sqrt(pow(to.y - from.y, 2) + pow(to.x - from.x, 2));
        float dist = num / den;

        if (dist < radius) {
            return true; //line is blocked
        }
    }
    return false;
}

//picking a point behind the closest cover
Point2f getEvadeTarget(Point2f myPos, Point2f oppPos, const std::vector<Point2f>& obstacles) {
    Point2f bestCover = myPos;
    float minDist = FLT_MAX;

    for (const auto& obs : obstacles) {
        Point2f dir = obs - oppPos;
        float length = sqrt(dir.x * dir.x + dir.y * dir.y);
        if (length == 0) continue;

        dir.x /= length;
        dir.y /= length;

        Point2f hidingSpot = obs + dir * 30.0f; // 30 units away from obstacle

        float dist = sqrt(pow(hidingSpot.x - myPos.x, 2) + pow(hidingSpot.y - myPos.y, 2));
        if (dist < minDist) {
            minDist = dist;
            bestCover = hidingSpot;
        }
    }
    return bestCover;
}

//movement functions (replace with actual robot control)
void moveTo(Point2f target) {
    cout << "Moving to (" << target.x << ", " << target.y << ")" << endl;
}

void stop() {
    cout << "Stopping in cover." << endl;
}
