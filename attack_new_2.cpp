// attack functions

#include <iostream>
#include <fstream>

#include <cmath>
#include <Windows.h>

using namespace std;

// include this header file for basic image transfer functions
#include "image_transfer.h"

// include this header file for computer vision functions
#include "vision.h"

// include this header file for attack mode functions
#include "attack_new_2.h"

#define KEY(c) ( GetAsyncKeyState((int)(c)) & (SHORT)0x8000 )

//this function gets the angle between 2 centroids to the positive x-axis
//it also gets the distance between the 2 centroids
int angle_distance(double x1, double y1, double x2, double y2, double &distance, double &angle){
	double dx, dy;
	
	dx = x1 - x2;
	dy = y1 - y2;
	
	distance = sqrt(dx * dx + dy * dy);
	angle = atan2(dy, dx) * (180.0/3.1415);
	if (angle < 0) angle += 360.0;
	
	return 0;
}

//this function checks if th distance from the obstacle projection on the path to the obstacle is less than the obstacle radius and extra margin
int projection_blocked(double xs, double ys, double xor, double yor, double xo, double yo, double obs_rad, double &clear_angle){
	double dx, dy, dx_o, dy_o, p_x, p_y;
	double proj_x, proj_y, dist_p;
	
	dx = xor - xs;
	dy = yor - ys;
	dx_o = xo - xs;
	dy_o = yo - xs;
	
	proj_x = ((dx*dx_o + dy*dy_o) / (dx*dx + dy*dy)) * dx; //vector of obstacle projection for x
	proj_y = ((dx*dx_o + dy*dy_o) / (dx*dx + dy*dy)) * dy; //vector of obstacle projection for y
	
	p_x = xs + proj_x; //x coordinate of projected point
	p_y = ys + proj_y; //y coordinate of projected point
	
	dx_p = p_x - xo;
	dy_p = p_y - yo;
	
	dist_p = sqrt(dx_p * dx_p + dy_p * dy_p);
	
	if (dist_p <= (obs_rad + 10)) clear_angle = false; //if its less than obstacle radius and robot radius
	else clear_angle = true;
	
	return 0;
}

//checks if self is close to edge
int edge_detection(double width1, double height1, double th, double xs_f, double ys_f, bool &boundary_detected){
	double x_min = 0;
	double x_max = width1;
	double y_min = 0;
	double y_max = height1;
	
	double dist_l = xs_f - x_min;
	double dist_r = x_max - xs_f;
	double dist_b = ys_f - y_min;
	double dist_f = y_max - ys_f;
	
	if (dist_l <= th || dist_r <= th || dist_b <= th || dist_f <= th) boundary_detected = true; //close to edge
	else boundary_detected = false; //not close to edge
	
	return 0;
}

//rotates the robot to the right (clockwise)
int clockwise (double &pw_l_new, double &pw_r_new){
	pw_l_new = 1000;
	pw_r_new = 1000;
	
	return 0;
}

//rotates the robot to the left (counterclockwise)
int counterclockwise(double &pw_l_new, double &pw_r_new){
	pw_l_new = 2000;
	pw_r_new = 2000;
	
	return 0;
}

//moves the robot to straight forward
int move_straight(double &pw_l_new, double &pw_r_new){
	pw_l_new =  1000;
	pw_r_new =  2000;
	
	return 0;
}

//decides when to rotate the robot to align with opponent for laser shooting
int rotate_to_opponent(double angle_s, double angle_r, bool &aligned, double &pw_l_new, double &pw_r_new){
	if (angle_s	> (angle_r - 10) && angle_s < (angle_r + 10)){
		aligned = true;
		move_straight(pw_l_new, pw_r_new);
	}
	else if (angle_s <= (angle_r +10)){
		aligned = false;
		counterclockwise(pw_l_new, pw_r_new);
	}
	else { 
		aligned = false;
		clockwise(pw_l_new, pw_r_new);
	}
	
	return 0;
}

//decides where to rotate the robot to align with opponent for laser shooting
int rotate_to_center(double width, double height, double xs, double ys, double angle_s, double &pw_l_new, double &pw_r_new){
	double dx, dy, angle, difference;
	
	double x = width / 2;
	double y = height / 2;
	
	dx = x - xs;
	dy = y - ys;
	
	angle = atan2 (dy/dx);
	if (angle < 0) angle += 360.0;
	
	difference = angle - angle_s;
	if (difference < 0) difference += 360.0;
	
	if (difference <= 180.0) counterclockwise(pw_l_new, pw_r_new);
	else clockwise(pw_l_new, pw_r_new);
	
	return 0;
}

//this function sets the goal target to shoot. It will be half the distance
//set dist_ratio from 0 to 1. It is the ration when the goal point would be between the robots
int goal(double xs_f, double ys_f, double xor_f, double yor_f, double ratio, double &goal_x, double &goal_y, double &dist_goal, double &angle_goal){
	double dx, dy , dx_r, dy_r;
	
	dx_r = xor_f - xs_f;
	dy_r = yor_f - ys_f;
	
	goal_x = xs_f + ratio * dx_r;
	goal_y = ys_f + ratio * dy_r;
	
	dx = goal_x - xs_f;
	dy = goal_y - ys_f;
	
	dist = sqrt(dx * dx + dy * dy);
	
	angle_goal = atan2(dy/dx) * (180.0/3.1415);
	if (angle_goal < 0) angle_goal += 360.0;
	
	return 0;
}

//decides when to rotate the robot to align with goal
int rotate_to_goal(double goal_x, double goal_y, double xs_f, double ys_f, double angle_s, double pw_l, double pw_r, double &pw_l_new, double &pw_r_new){
	double dx, dy, angle;
	
	dx = goal_x - xs_f;
	dy = goal_y - ys_f;
	
	angle = atan2(dy/dx) * (180.0/3.1415);
	if (angle < 0.0) angle += 360.0;
	
	//using the angle error between obs and self
	//check which direction to rotate
	if (angle_s <= angle - 10.0){ 
	//turns left
		counterclockwise(pw_l_new, pw_r_new);
	}
	else if (angle_s >= angle + 10.0){ 
	//turns right
		clockwise(pw_l_new, pw_r_new);
	}
	else { 
	//goes straight forward
		move_straight(pw_l_new, pw_r_new);
	}
	
	return 0;
}

// this function sets a detour goal point if obstacle exists between the robots.
// go to this point when there is an obstacle. In main loop, if statement if obstacle in path
// set an obstacle margin/radius obs_rad is the radius of obs, and margin is the theshold
int detour(double xo, double yo, double obs_rad, double &detour_x, double &detour_y){
	detour_x = xo + (obs_rad + margin) * cos(angle_r + 90°);
	detour_y = yo + (obs_rad + margin) * sin(angle_r + 90°);
	
	return 0;
}

//decides where to rotate the robot to align with detour
int rotate_to_detour(double detour_x, double detour_y, double xs_f, double ys_f, double angle_s, double pw_l, double pw_r, double &pw_l_new, double &pw_r_new){
	double dx, dy, angle;
	
	dx = detour_x - xs_f;
	dy = detour_y - ys_f;
	
	angle = atan2(dy/dx) * (180.0/3.1415);
	if (angle < 0.0) angle += 360.0;
	
	//using the angle error between obs and self
	//check which direction to rotate
	if (angle_s <= angle - 10.0){ 
	//turns left
		counterclockwise(pw_l_new, pw_r_new);
	}
	else if (angle_s >= angle + 10.0){ 
	//turns right
		clockwise(pw_l_new, pw_r_new);
	}
	else { 
	//goes straight forward
		pw_l_new =  1000;
		pw_r_new =  2000;
	}
	
	return 0;
}

int line_aligned(double xs, double ys, double xor, double yor, double &aligned){
	double m, b, dx, dy, y1, y2;	
	
	dx = xor - xs;
	dy = yor - ys;
	
	m = dy/dx;
	b = ys-(m*xs);
	
	y1 = xor * m + (b + 10);
	y2 = xor * m + (b - 10);
	
	if (y1 < yor && y2 > yor) aligned = 1;
	else aligned = 0;
	
	return 0;
}