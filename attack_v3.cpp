#include <iostream>
#include <fstream>

#include <cmath>
#include <Windows.h>

using namespace std;

void get_angle(double x1, double y1, double x2, double y2, double& angle) {
	double dx = x1 - x2;
	double dy = y1 - y2;
	angle = atan2(dy, dx) * (180.0 / 3.1415);
	if (angle < 0.0) angle += 360.0;
	if (angle >= 360.0) angle -= 360;
}

void get_distance(double x1, double y1, double x2, double y2, double& distance) {
	double dx = x1 - x2;
	double dy = y1 - y2;
	distance = sqrt(dx * dx + dy * dy);
}

void get_robo_midpoint(double x1, double y1, double x2, double y2, double& position_x, double& position_y) {

	position_x = (x1 + x2) / 2;
	position_y = (y1 + y2) / 2;
}

// returns true if the line segment (x1,y1)->(x2,y2) end points (x1,y1), (x2,y2) have LOS with each other (line segment doesnt hit any obstacles)
bool has_line_of_sight(double x1, double y1, double x2, double y2, const double obs_ic[], const double obs_jc[], const double obs_r[], int obs_count) {

	// vector from start to end of the segment
	double ax = x2 - x1;
	double ay = y2 - y1;
	double length_squared = ax * ax + ay * ay;  // squared segment length

	for (int k = 1; k <= obs_count; k++) {
		double cx = obs_ic[k];
		double cy = obs_jc[k];
		double r = obs_r[k]; //ADDED 40 PIXEL BUFFER THE OBSTACLE RADIUS TO ACCOUNT FOR ROBOT THICKNESS--- SEE VISION2.CPP

		// vector from segment start to circle center
		double bx = cx - x1;
		double by = cy - y1;

		// project center onto the line (standard dot product projection)
		// p=0 corresponds to (x1,y1), t=1 to (x2,y2)
		double p = (bx * ax + by * ay) / length_squared;

		// were looking for point thats closest to obstacle buyt on the vector (x1,y1)->(x2,y2) .
		// ... the closest point when t>1 is the end point... when t<1 its the start
		if (p < 0.0) p = 0.0;
		else if (p > 1.0) p = 1.0;

		// the closest point on segment to circle center is then
		double closest_x = x1 + p * ax;
		double closest_y = y1 + p * ay;

		// squared distance from circle center to that closest point
		double dist_sq = (cx - closest_x) * (cx - closest_x) + (cy - closest_y) * (cy - closest_y);

		// if that distance <= r^2 then the segment intersects the circle
		if (dist_sq <= r * r) {

			return false;
		}
	}

	// no intersections means the poitns have LOS w eachother return true for success
	return true;

}


//checks if point is in an obstacle
bool is_in_obstacle(double x, double y, double obs_ic[], double obs_jc[], double obs_r[], int obs_count)
{
	for (int k = 1; k <= obs_count; k++) {
		double dx = x - obs_ic[k];
		double dy = y - obs_jc[k];
		double R = obs_r[k]; //ADDED 40 PIXEL BUFFER THE OBSTACLE RADIUS TO ACCOUNT FOR ROBOT THICKNESS--- SEE VISION2.CPP


		if (dx * dx + dy * dy <= R * R) {
			return true;
		}
	}
	return false;
}


bool find_vantage_point(
	int self_x,
	int self_y,
	int opp_x,
	int opp_y,
	double obs_ic[],
	double obs_jc[],
	double obs_r[],
	int obs_count,
	int& vantage_x,
	int& vantage_y) {

	double rmax = 200.0; //max search radius 
	double rmin= 50; // dont want vantage point to be too close to where the robot already is. give it time and space to line up laser
	double dr = 3.0; // radius divisions (pixels)
	double ds = 3.0; // arc-length divisions (pixels)
	double s, smax, theta;
	int i, j;
	for (double r =rmin; r <= rmax; r += dr) {
		smax = 2 * 3.1416 * r; // maximum arc length
		for (s = 0; s <= smax; s += ds) {
			theta = s / r; // s = r*theta
			i = (int)(self_x + r * cos(theta));
			j = (int)(self_y + r * sin(theta));

			if (i < 0 + 100 || i >= 640 - 100 || j < 0 + 100 || j >= 480 - 100) //MUST ADD A BUFFER ZONE SO ROBOT DOESNT GO OFF SCREEN ->100 PIXELS
				continue;

			if (is_in_obstacle(i, j, obs_ic, obs_jc, obs_r, obs_count))
				continue;

			if (has_line_of_sight(i, j, opp_x, opp_y, obs_ic, obs_jc, obs_r, obs_count)) {
				vantage_x = i;
				vantage_y = j;
				return true;
			}
		}
	}

	return false;
}


bool pointturn_to_angle(double currentDeg, double targetDeg, double& pw_l, double& pw_r) {

	double diff = fmod(targetDeg - currentDeg + 360.0, 360.0);
	//cout << "\n" << diff;
	if (diff < 2 || diff>358) { //within 2 degrees
		// already on target
		pw_l = pw_r = 1500;
		return true;
	}

	else if (diff <= 180.0) {
		// shortest path is CCW
		pw_l = 2000;
		pw_r = 2000;
		return false;
	}

	else {
		// shortest path is CW
		pw_l = 1000;
		pw_r = 1000;
		return false;
	}
}

void go_to(int self_x, //goes to point in straight line
	int self_y,
	double currentDeg,
	int target_x,
	int target_y,
	double& pw_l,
	double& pw_r) {

	//cout << "\n" << target_x << "\t" << target_y;

	double dist;
	get_distance(target_x, target_y, self_x, self_y, dist);

	//cout << "\n" << self_y; 

	if (dist < 5) { //if within 5 pixels stop moving
		pw_l = pw_r = 1500;
		return;
	}

	double targetDeg;
	get_angle(target_x, target_y, self_x, self_y, targetDeg);

	double diff = fmod(targetDeg - currentDeg + 360.0, 360.0);


	if (diff < 15 || diff>345) { //if within 15 degrees move forward

		pw_l = 1000;
		pw_r = 2000;

	}

	else if (diff <= 180.0) {
		// shortest path is CCW
		pw_l = 2000;
		pw_r = 2000;

	}

	else {
		// shortest path is CW
		pw_l = 1000;
		pw_r = 1000;

	}
}

bool find_detour_point(int self_x, int self_y,
	int goal_x, int goal_y,
	double obs_ic[], double obs_jc[], double obs_r[],
	int obs_count,
	int& detour_x, int& detour_y)
{
	double rmax = 200.0; //max search radius 
	double dr = 3.0; // radius divisions (pixels)
	double ds = 3.0; // arc-length divisions (pixels)
	double s, smax, theta;
	
	for (double r = 1.0; r <= rmax; r += dr) {
		smax = 2 * 3.1416 * r; // maximum arc length
		for (s = 0; s <= smax; s += ds) {
			theta = s / r; // s = r*theta
			int dx = (int)(self_x + r * cos(theta));
			int dy = (int)(self_y + r * sin(theta));

			// if out of screen forget it
			if (dx < 0 + 100 || dx >= 640 - 100 || dy < 0 + 100 || dy >= 480 - 100) //MUST ADD A BUFFER ZONE SO ROBOT DOESNT GO OFF SCREEN ->100 PIXELS
				continue;

			// cant be inside obstacle
			if (is_in_obstacle(dx, dy, obs_ic, obs_jc, obs_r, obs_count))
				continue;

			//robo must have LOS with the detour point AND
			//detour point must have LOS with the goal

			if (has_line_of_sight(self_x, self_y, dx, dy, obs_ic, obs_jc, obs_r, obs_count)
				&& has_line_of_sight(dx, dy, goal_x, goal_y, obs_ic, obs_jc, obs_r, obs_count)) {

				detour_x = dx; //if both are satisified then set the detour point and return true to say we found a detour
				detour_y = dy;

				return true;



			}


		}
	}
	return false; //detour point wasnt found
}

void go_to_withavoidance(int  self_x,
	int    self_y,
	double currentDeg,
	int    goal_x,
	int    goal_y,
	double& pw_l,
	double& pw_r,
	double obs_ic[], double obs_jc[], double obs_r[],
	int    obs_count)
{

	// If base point has LOS with the goal then go straigth there
	if (has_line_of_sight(self_x, self_y, goal_x, goal_y, obs_ic, obs_jc, obs_r, obs_count))
	{

		go_to(self_x, self_y, currentDeg, goal_x, goal_y, pw_l, pw_r);

		return;
	}

	int det_x, det_y;
	if (find_detour_point(self_x, self_y, goal_x, goal_y, obs_ic, obs_jc, obs_r, obs_count, det_x, det_y))
	{

		// we found a detour point (det_x,det_y) that robot can see from base point and the detour point can see the goal
		go_to(self_x, self_y, currentDeg, det_x, det_y, pw_l, pw_r);

		return;
	}


	//cout << "\ncant find detour and doesnt have LOS";
	pw_l = 1000; 
	pw_r = 1500; //try to get unstuck

}

void run_attack(int& gx, int& gy, int& rx, int& ry, int& ox, int& oy, int& bx, int& by, double obs_ic[], 
	double  obs_jc[], double obs_r[], int obs_count, double &pw_l, double &pw_r, double &laser) {
	
	double angle_self, angle_opp, x_midposition_self, x_midposition_opp, y_midposition_self, y_midposition_opp;;
	static int vantage_x = 1000, vantage_y = 1000; //initial vantage point position of 1k, 1k ensures vantage point is calculated on first loop
	static bool preparing_to_fire = false;
	static bool laser_fired = false;
	
	//once we have the locations of items of interest we can set which robot we are and which the opponent is. (gx, gy)= green marker centroid location		
	double front_self_x = gx;
	double front_self_y = gy;
	double back_self_x = rx;
	double back_self_y = ry;

	double front_opp_x = ox;
	double front_opp_y = oy;
	double back_opp_x = bx;
	double back_opp_y = by;

	get_angle(front_self_x, front_self_y, back_self_x, back_self_y, angle_self);
	get_angle(front_opp_x, front_opp_y, back_opp_x, back_opp_y, angle_opp);
	get_robo_midpoint(front_self_x, front_self_y, back_self_x, back_self_y, x_midposition_self, y_midposition_self);
	get_robo_midpoint(front_opp_x, front_opp_y, back_opp_x, back_opp_y, x_midposition_opp, y_midposition_opp);

	double self_to_oppfront_angle, self_to_oppback_angle;
	get_angle(front_opp_x, front_opp_y, front_self_x, front_self_y, self_to_oppfront_angle);
	get_angle(back_opp_x, back_opp_y, front_self_x, front_self_y, self_to_oppback_angle);

	//get_distance(front_self_x, front_self_y, vantage_x, vantage_y, vantage_to_attacker);

	find_vantage_point(front_self_x, front_self_y, x_midposition_opp, y_midposition_opp, obs_ic, obs_jc, obs_r, obs_count, vantage_x, vantage_y);

	int min_LOS_frames = 20;
	static int los_frames = 0;      // only initialize one time

	bool front_los = has_line_of_sight(front_self_x, front_self_y, x_midposition_opp, y_midposition_opp, obs_ic, obs_jc, obs_r, obs_count);

	if (front_los) {
		los_frames++;
	}
	
	else {
		los_frames = 0;
		preparing_to_fire = false;
	}
		
	if (los_frames >= min_LOS_frames) {
		preparing_to_fire = true;
	}


	if (!preparing_to_fire) {
			go_to_withavoidance(front_self_x, front_self_y, angle_self, vantage_x, vantage_y, pw_l, pw_r, obs_ic, obs_jc, obs_r, obs_count);
	}

	else if (preparing_to_fire) {

		if (pointturn_to_angle(angle_self, self_to_oppfront_angle, pw_l, pw_r)) {
			laser = 1;
			laser_fired = true;
		}
	} 


	if (laser_fired) {
			pw_r = pw_l = 1500;
	} 

}