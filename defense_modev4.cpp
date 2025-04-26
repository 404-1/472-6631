#include "defense.h"
#include <iostream>
#include <vector>
#include <cmath>
#include <cfloat> 
#include "vision.h"
#include "image_transfer.h"

using namespace std;

#define SAFE_DISTANCE 50.0f
#define MAX_SPEED      5.0f

//check if any obstacle blocks path
bool isLineOfSightBlocked(Point2f from, Point2f to,
                          const vector<Point2f>& obstacles,
                          float radius)
{
    Point2f d = to - from;
    float len2 = d.x*d.x + d.y*d.y;
    // avoid division by zero
    if(len2 < 1e-6f) return false;

    for(const auto& obs : obstacles) {
        // project obs onto the line segment
        Point2f v = obs - from;
        float t = (v.x*d.x + v.y*d.y) / len2;
        if(t < 0.0f || t > 1.0f) continue;  // outside segment

        // perpendicular distance
        float num = fabs(d.y*obs.x - d.x*obs.y + to.x*from.y - to.y*from.x);
        float den = sqrt(len2);
        float dist = num/den;
        if(dist < radius) return true;
    }
    return false;
}

// pick a hiding spot ~30 units behind the closest obstacle
Point2f getEvadeTarget(Point2f myPos, Point2f oppPos,
                       const vector<Point2f>& obstacles)
{
    Point2f bestCover = myPos;
    float  minDist   = FLT_MAX;

    for(const auto& obs : obstacles) {
        Point2f dir = obs - oppPos;
        float  len = sqrt(dir.x*dir.x + dir.y*dir.y);
        if(len < 1e-6f) continue;
        // unit vector away from opponent
        dir.x /= len;  
        dir.y /= len;

        // candidate hiding spot
        Point2f hidingSpot = obs + dir * SAFE_DISTANCE;

        // distance from current position
        float dx = hidingSpot.x - myPos.x;
        float dy = hidingSpot.y - myPos.y;
        float dist = sqrt(dx*dx + dy*dy);
        if(dist < minDist) {
            minDist   = dist;
            bestCover = hidingSpot;
        }
    }
    return bestCover;
}

void moveTo(Point2f target) {
    //insert movement here
    cout << "Move command issued to: (" << target.x << ", " << target.y << ")" << endl;
}

void stop() {
    //insert movement/stop here
    cout << "Robot is holding position (in cover)." << endl;
}

// defense‐mode entry point
void defenseMode(Point2f myPosition,
                 Point2f opponentPosition,
                 const vector<Point2f>& obstacles,
                 float obstacleRadius)
{
    //is opponent’s laser line of sight clear?
    bool exposed = !isLineOfSightBlocked(opponentPosition,
                                         myPosition,
                                         obstacles,
                                         obstacleRadius);
    if(exposed) {
        Point2f target = getEvadeTarget(myPosition,
                                        opponentPosition,
                                        obstacles);
        moveTo(target);
    }
    else {
        stop();
    }
}
