#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <Windows.h>

using namespace std;

#include "image_transfer.h"

#include "vision.h"

#include "attack_new_2.h"

#define KEY(c) ( GetAsyncKeyState((int)(c)) & (SHORT)0x8000 )

int main()
{ 
while (1){
bool aligned, clear_dist, clear_angle, clear, goal_reached, obs_to_right, obs_to_left;
double margin;

double xo, yo;						//obstacle centroids
double xs_f, ys_f, xs_b, ys_b;		// self robot centroids
double xor_f, yor_f, xor_b, yor_b;	// opponet centroids

//after getting centroids for all objects
//get all position/angle parameters to find exact relative positions


//Parameters
//get angles of robots relative to positive x-axis
	//self robot
	angle_distance(xs_f, ys_f, xs_b, ys_b, dist_s, angle_s); //gets dist_s, angle_s
	
	//opponent robot
	angle_distance(xor_f, yor_f, xor_b, yor_b, dist_or, angle_or); //gets dist_or, angle_or

//get distance and angle between robots
	angle_distance(xor_f, yor_f, xs_f, ys_f, dist_r, angle_r); //gets dist_r, angle_r 
	
//for now, use one obstacle

//gets distance and angle from self robot to obstacle
	angle_distance(xo, yo, xs_f, ys_f, dist_obs, angle_obs); 
	//gets dist_obs, angle_obs

//gets goal coordinates to shoot laser
	//goal(xs_f, ys_f, xor_f, yor_f, 0.5, goal_x, goal_y, dist_goal, angle_goal);
	//gets goal_x/y, dist_goal, angle_goal
	
//check if edge in near to self
	edge_detection(width1, height1, th, xs_f, ys_f, boundary_detected); //gets boundary_detected
	
	
//Conditions	
//check if robot is aligned with opponent to go towards it
	if (abs(angle_s - angle_r) <= 10) aligned = true;
	else aligned = false;
	
//checks if obstacle is closer than target
	if (dist_obs <= dist_r) clear_dist = false; //maybe blocked. more checks
	else clear = true; //clear for sure since its after the target
		
//check if obstacle is cutting the linear path
	if (!clear_dist){
		//check projection so if its cuts the linear path from robots
		projection_blocked(xs, ys, xor, yor, xo, yo, obs_rad, clear_angle); //gets clear_angle
		
		//checks if its blocked and give a detour point away from obstacle
		if (!clear_angle) clear = false;
		else clear = true;
	}
	
//check if obstacle is close to robot
	if (!clear && (dist_obs <= (obs_rad + margin))) obs_close = true;
	if (!clear && (dist_obs > (obs_rad + margin))){
		obs_close = false;
		obs_to_right = false;			
		obs_to_left = false;
	}
		
	if (obs_close){ 
		if (angle_s <= angle_obs){ //should go ccw at 90 degree around obstacle
			obs_to_right = true;
			obs_to_left = false;
		}
		
		else { //should go cw at 90 degree around obstacle
			obs_to_right = false;			
			obs_to_left = true;
		}
	if (clear){
		obs_to_right = false;			
		obs_to_left = false;
	}
	
	
//Actions
	if (boundary_detected) rotate_to_center(width, height, xs_f, ys_f, angle_s, pw_l_new, pw_r_new);
	//and move forward
	
	if (obs_to_right) counterclockwise(pw_l_new, pw_r_new);
	//and move forward
	
	if (obs_to_left) clockwise(pw_l_new, pw_r_new);
	//and move forward
	
	if (aligned) move_straight(pw_l_new, pw_r_new);
	
	if (!aligned && clear) rotate_to_opponent(angle_s, angle_r, aligned, pw_l_new, pw_r_new);
	
	
	//else rotate_to_goal(goal_x, goal_y, xs_f, ys_f, angle_s, pw_l_new, pw_r_new); 
		//gets pw_new for goal
	
//check if the robot reached the goal point (certain distance to opponent)
	//if (dist_goal > 10) goal_reached = false;
	//else goal_reached = true;
	
//align self robot with opponent before shooting
//	if (goal_reached) rotate_to_opponent(angle_s, angle_r, aligned, pw_l_new, pw_r_new);
					  //gets pw_new for opponent laser shoot
					  
//check if vision is clear and aligned to target
	if (aligned && clear) laser = 1; //shoots laser
	else laser = 0;
}