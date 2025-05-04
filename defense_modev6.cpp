#include "defense.h"
#include <iostream>
#include <vector>
#include <cmath>
#include <cfloat>

using namespace std;

#define SAFE_DISTANCE 50.0f   // the minimum safe distance to keep from obstacles or opponents
#define MAX_SPEED      5.0f   // the maximum motor speed (both left and right wheels)
const double PI = 4 * atan(1.0);  // accurate value of PI 

// this calculates the straight-line euclidean distance between two points (x1, y1) and (x2, y2)
// the result is stored in the reference variable 'distance'
void getDistance(float x1, float y1, float x2, float y2, double& distance) {
    distance = sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

// this calculates the angle (in deg) from point (x1, y1) to (x2, y2)
// this is useful for determining the direction to face or move
void getAngle(float x1, float y1, float x2, float y2, double& angle) {
    angle = atan2(y2 - y1, x2 - x1) * 180.0 / PI;  // atan2 returns angle in rad, converted to deg
}

/*the following function checks whether the line from 'from' to 'to' is blocked by any obstacles
this simulates whether the opponent's laser (line of sight) is obstructed
it has the following parameters:
- from: start point (opponent position)
- to: end point (our robot position)
- obstacles: list of obstacle positions
- radius: approximate "blocking range" of each obstacle
 
 it returns true if there is an obstacle blocking the path (line of sight is blocked), false otherwise*/
bool isLineOfSightBlocked(Point2f from, Point2f to, const vector<Point2f>& obstacles, float radius)
{
    // the vector from 'from' to 'to'
    Point2f d = to - from;
    float len2 = d.x*d.x + d.y*d.y;

    //if the vector is essentially zero, consider it not blocked
    if(len2 < 1e-6f) return false;

    // iterate through each obstacle to check if it's near the line
    for(const auto& obs : obstacles) {
        Point2f v = obs - from;

        // then we project obstacle onto the line segment
        float t = (v.x*d.x + v.y*d.y) / len2;

        // it's ignored if obstacle lies beyond either end of the line segment
        if(t < 0.0f || t > 1.0f) continue;

        // compute perpendicular distance from the obstacle to the line
        float num = fabs(d.y*obs.x - d.x*obs.y + to.x*from.y - to.y*from.x);
        float den = sqrt(len2);
        float dist = num / den;

        // if the obstacle is close enough to the line, it blocks the view
        if(dist < radius) return true;
    }

    return false;// no blocking obstacle found
}

/* This next function computes a target position to move to for hiding behind the closest obstacle
it identifies the obstacle that (when used as cover) is closest to our robot
it then computes a position behind that obstacle (opposite direction from opponent)
note that the point2f class is structured in the header file */
Point2f getEvadeTarget(Point2f myPos, Point2f oppPos, const vector<Point2f>& obstacles)
{ 
    Point2f bestCover = myPos;  // defaults to current position
    float minDist = FLT_MAX;    // starts with a very large distance

    for(const auto& obs : obstacles) {
        // we compute unit direction from opponent to obstacle
        Point2f dir = obs - oppPos;
        float len = sqrt(dir.x*dir.x + dir.y*dir.y);
        if(len < 1e-6f) continue;  // skip degenerate cases
        dir.x /= len;
        dir.y /= len;

        // computes a hiding spot behind the obstacle
        Point2f hidingSpot = obs + dir * SAFE_DISTANCE;

        // measures distance from robot to hiding spot
        float dx = hidingSpot.x - myPos.x;
        float dy = hidingSpot.y - myPos.y;
        float dist = sqrt(dx*dx + dy*dy);

        // update the best cover if this spot is closer
        if(dist < minDist) {
            minDist = dist;
            bestCover = hidingSpot;
        }
    }

    return bestCover;  // return the best spot to hide behind
}

/*The following drives the robot towards a target point, while adjusting wheel speeds to steer
the robot steers left or right depending on angle difference between its heading and target
These parameters are used:
- target: where we want to go (hiding spot)
- myPosition: current robot position
- opponentPosition: used to infer current heading direction
- obstacles: not directly used here (but passed for potential future use)
- obstacleRadius: not used, could be used for smarter path planning (maybe)
- pw_l / pw_r: output variables controlling left/right motor speeds */
void moveTo(Point2f target, Point2f myPosition, Point2f opponentPosition, const vector<Point2f>& obstacles, float obstacleRadius, float& pw_l, float& pw_r)
{
    // calculate straight line distance to target
    double dist;
    getDistance(myPosition.x, myPosition.y, target.x, target.y, dist);

    // if we're already close enough just retreat forward quickly
    if (dist < SAFE_DISTANCE) {
        pw_l = MAX_SPEED;
        pw_r = MAX_SPEED;
    } else {
        // calculates direction to the target
        double targetDeg;
        getAngle(target.x, target.y, myPosition.x, myPosition.y, targetDeg);

        // infer robot's current heading based on direction to opponent
        double currentDeg = atan2(myPosition.y - opponentPosition.y, myPosition.x - opponentPosition.x) * 180 / PI;

        // compute angular difference and normalized
        double diff = fmod(targetDeg - currentDeg + 360.0, 360.0);

        // adjust motor speeds to steer toward target direction
        if (diff < 180.0) {
            // target is to the left; turn slightly left
            pw_l = MAX_SPEED;
            pw_r = MAX_SPEED * 0.8;
        } else {
            // target is to the right; turn slightly right
            pw_l = MAX_SPEED * 0.8;
            pw_r = MAX_SPEED;
        }
    }

    // just printing movement command for debugging/logging purposes
    cout << "Move command issued: (" << target.x << ", " << target.y << ") with pw_l: " << pw_l << ", pw_r: " << pw_r << "\n";
}

/*this will stop the robot by setting both motors to zero speed
called when robot is safe and does not need to move*/
void stop(float& pw_l, float& pw_r) {
    pw_l = 0.0f;
    pw_r = 0.0f;

    cout << "Robot is stopping, holding position in cover\n";
}

/*Then the main logic for activating defense behavior:
the robot checks whether it is currently exposed to the opponent
if it is exposed (line of sight is not blocked), it finds cover
if it is not exposed, it stops moving and holds position*/
void defenseMode(Point2f myPosition, Point2f opponentPosition, const vector<Point2f>& obstacles, float obstacleRadius, float& pw_l, float& pw_r)
{
    // check whether the opponent has a direct line of sight to the robot
    bool exposed = !isLineOfSightBlocked(opponentPosition, myPosition, obstacles, obstacleRadius);

    if (exposed) {
        // if we are exposed to enemy laser, move to a better hiding spot
        Point2f target = getEvadeTarget(myPosition, opponentPosition, obstacles);
        moveTo(target, myPosition, opponentPosition, obstacles, obstacleRadius, pw_l, pw_r);
    } else {
        // if already hidden (line of sight blocked), stop and hold position
        stop(pw_l, pw_r);
    }
}
