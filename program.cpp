
#include <cstdio>
#include <iostream>
#include <fstream>

#include <Windows.h>

using namespace std;

#define KEY(c) ( GetAsyncKeyState((int)(c)) & (SHORT)0x8000 )

#include "image_transfer.h"

// include this header file for computer vision functions
#include "vision.h"

#include "robot.h"

#include "vision_simulation.h"

#include "timer.h"

#include "vision2.h"

#include "attack_new_2.h"

extern robot_system S1;

void getAngle(double x1, double y1, double x2, double y2, double& angle) {
	double dx = x1 - x2;
	double dy = y1 - y2;
	angle = atan2(dy, dx) * (180.0 / 3.1415);
	if (angle < 0.0) angle += 360.0;
	if (angle >= 360.0) angle -= 360;
}

void getDistance(double x1, double y1, double x2, double y2, double& distance) {
	double dx = x1 - x2;
	double dy = y1 - y2;
	distance = sqrt(dx * dx + dy * dy);
} 

void getRobotPosition(double x1, double y1, double x2, double y2, double& position_x, double & position_y) {

	position_x = (x1 + x2) / 2; 
	position_y = (y1 + y2) / 2; 
}

// returns true if the line segment (x1,y1)->(x2,y2) end points (x1,y1), (x2,y2) have LOS with each other (line segment doesnt hit any obstacles)
bool hasLineOfSight(double x1, double y1, double x2, double y2, const double obs_ic[], const double obs_jc[], const double obs_r[], int obs_count) {

	// vector from start to end of the segment
	double ax = x2 - x1;
	double ay = y2 - y1;
	double segLen2 = ax * ax + ay * ay;  // squared segment length

	for (int k = 1; k <= obs_count; k++) {
		double cx = obs_ic[k];
		double cy = obs_jc[k];
		double r = obs_r[k]+15; //ADD 15 PIXEL BUFFER TO THE OBSTACLE RADIUS TO ACCOUNT FOR ROBOT THICKNESS

		// vector from segment start to circle center
		double bx = cx - x1;
		double by = cy - y1;

		// project center onto the line (standard dot product projection)
		//t=0 corresponds to (x1,y1), t=1 to (x2,y2)
		double t = (bx * ax + by * ay) / segLen2;

		// were looking for point thats closest to obstacle buyt on the vector (x1,y1)->(x2,y2) .
		// ... the closest point when t>1 is the end point... when t<1 its the start
		if (t < 0.0) t = 0.0;
		else if (t > 1.0) t = 1.0;

		// the closest point on segment to circle center is then
		double closestX = x1 + t * ax;
		double closestY = y1 + t * ay;

		// squared distance from circle center to that closest point
		double dist2 = (cx - closestX) * (cx - closestX) + (cy - closestY) * (cy - closestY);

		// if that distance <= r^2 then the segment intersects the circle
		if (dist2 <= r * r) {
			 
			return false;
		}
	}

	// no intersections means the poitns have LOS w eachother return true for success
	return true;

}


//checks if point is in an obstacle
bool isInObstacle(double x, double y, double obs_ic[], double obs_jc[], double obs_r[],int obs_count)
{
	for (int k = 1; k <= obs_count; k++) {
		double dx = x - obs_ic[k];
		double dy = y - obs_jc[k];
		double R = obs_r[k] +15; //ADD 15 PIXEL BUFFER TO THE OBSTACLE RADIUS TO ACCOUNT FOR ROBOT THICKNESS


		if (dx * dx + dy * dy <= R*R) {
			return true;
		}
	}
	return false;
}


bool findVantagePoint(
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
	double dr = 3.0; // radius divisions (pixels)
	double ds = 3.0; // arc-length divisions (pixels)
	double s, smax, theta;
	int i, j; 
	for (double r = 1.0; r <= rmax; r += dr) {
		smax = 2 * 3.1416 * r; // maximum arc length
		for (s = 0; s <= smax; s += ds) {
			theta = s / r; // s = r*theta
			i =(int) (self_x + r * cos(theta));
			j = (int)(self_y + r * sin(theta));

			if (i < 0 || i >= 640 || j < 0 || j >= 480)
				continue;

			if (isInObstacle(i, j, obs_ic, obs_jc, obs_r, obs_count))
				continue;

			if (hasLineOfSight(i, j, opp_x, opp_y, obs_ic, obs_jc, obs_r, obs_count)) {
				vantage_x = i;
				vantage_y = j;
				return true;
			}
		}
	}

	return false;
}


void pointturnToAngle(double currentDeg, double targetDeg, double& pw_l, double& pw_r) {

	double diff = fmod(targetDeg - currentDeg + 360.0, 360.0);
	cout << "\n" << diff;
	if (diff < 2 || diff>358) { //within 2 degrees
		// already on target
		pw_l = pw_r = 1500;
		 
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

void goTo(int self_x, //goes to point in straight line
	int self_y,
	double currentDeg,
	int target_x,
	int target_y,
	double& pw_l,
	double& pw_r) {

	//cout << "\n" << target_x << "\t" << target_y;

	double dist;
	getDistance(target_x, target_y, self_x, self_y, dist);
	
	//cout << "\n" << self_y; 
	
	if (dist < 5) { //if within 5 pixels stop moving
		pw_l = pw_r = 1500;  
		return;
	}
	
	double targetDeg; 
	getAngle( target_x, target_y, self_x, self_y, targetDeg);

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

bool findDetourPoint(int self_x, int self_y,
	int goal_x, int goal_y,
	double obs_ic[], double obs_jc[], double obs_r[],
	int obs_count,
	int& detour_x, int& detour_y)
{
	
	
	hasLineOfSight(self_x, self_y,goal_x, goal_y,obs_ic, obs_jc, obs_r,obs_count); // must call this func to get the blocking obstacle id for later. 
	
	
	// how long of a detour (pixl)
	double DETOUR_DIST = 200.0;
	// angle increments (rad) 
	double DETOUR_ANGLE = 5.0 * 3.14159 / 180.0;  // 5degs
	// max sweep of 90 deg
	double MAX_ANGLE = 3.14159 / 2;

	//  start at the angle where the robo is going 
	double starting_angle = atan2(goal_y - self_y, goal_x - self_x);

	// try offsets in 10 deg increments up to 90
	for (double a = DETOUR_ANGLE; a <= MAX_ANGLE; a += DETOUR_ANGLE) {
		
		for (int sign = -1; sign <= 1; sign += 2) { //check +ve and -ve angles
		
			double θ = starting_angle + sign * a;
			int  dx = (int)(self_x + DETOUR_DIST * cos(θ));
			int  dy = (int)(self_y + DETOUR_DIST * sin(θ));
			
			// if out of screen forget it
			if (dx < 0 || dx >= 640 || dy < 0 || dy >= 480)
				continue;
			
			// cant be inside obstacle
			if (isInObstacle(dx, dy, obs_ic, obs_jc, obs_r, obs_count))
				continue;
			
			//robo must have LOS with the detour point AND
			//detour point must have LOS with the goal

			
			if (hasLineOfSight(self_x, self_y, dx, dy, obs_ic, obs_jc, obs_r, obs_count)
				&& hasLineOfSight(dx, dy, goal_x, goal_y, obs_ic, obs_jc, obs_r, obs_count)){
			 
				detour_x = dx; //if both are satisified then set the detour point and return true to say we found a detour
				detour_y = dy; 
				
				return true;

				
				
			}
		
		
		}
	}
	return false; //detour point wasnt found
}

void goTo_WithAvoidance(int  self_x,
	int    self_y,
	double currentDeg,
	int    goal_x,
	int    goal_y,
	double& pw_l,
	double& pw_r,
	double obs_ic[], double obs_jc[], double obs_r[],
	int    obs_count)
{
	int blocking_obs_id_dummy; 
		
	// If base point has LOS with the goal then go straigth there
	if (hasLineOfSight(self_x, self_y, goal_x, goal_y, obs_ic, obs_jc, obs_r, obs_count))
	{
		
		goTo(self_x, self_y, currentDeg, goal_x, goal_y, pw_l, pw_r);
		
		return;
	}

	int det_x, det_y;
	if (findDetourPoint(self_x, self_y, goal_x, goal_y, obs_ic, obs_jc, obs_r, obs_count, det_x, det_y))
	{
		
		// we found a detour point (det_x,det_y) that robot can see from base point and the detour point can see the goal
		goTo(self_x, self_y, currentDeg, det_x, det_y, pw_l, pw_r);
		
		return;
	}

	
	cout << "\ncant find detour and doesnt have LOS"; 
	
	
}


//check_to_fire(); 


int main()
{

	image temp_grey1, temp_grey2, rgb, black_mask, original, color_mask, mask; // declare some image structures
	image label;
	int cam_number;
	int R, G, B, nlabels, obs_count;
	double obs_ic[50], obs_jc[50], obs_r[50];


	const int maxlabels = 50;
	double ic[maxlabels + 1];
	double jc[maxlabels + 1];
	double area[maxlabels + 1];
	double R_ave[maxlabels + 1];
	double G_ave[maxlabels + 1];
	double B_ave[maxlabels + 1];
	int label_number[maxlabels + 1];

	int gx, gy, rx, ry, ox, oy, bx, by;
	double angle_self, angle_opp, x_position_self, x_position_opp, y_position_self, y_position_opp;;
	int vantage_x=1000, vantage_y=1000; //initial vantage point position of 1k, 1k ensures vantage point is calculated on first loop
	double vantage_to_attacker;

	Item* items[maxlabels + 1]; //array of pointers to item objects


	double x0, y0, theta0, max_speed, opponent_max_speed;
	double pw_l, pw_r, pw_laser, laser;
	double light, light_gradient, light_dir, image_noise;
	double width1, height1;
	int N_obs, n_robot;
	double x_obs[50], y_obs[50], size_obs[50];
	double D, Lx, Ly, Ax, Ay, alpha_max;
	double tc, tc0; // clock time
	int mode, level;
	int pw_l_o, pw_r_o, pw_laser_o, laser_o;
	bool laser_fire = false;

	// TODO: it might be better to put this model initialization
	// section in a separate function

	// note that the vision simulation library currently
	// assumes an image size of 640x480
	width1 = 640;
	height1 = 480;

	// number of obstacles
	N_obs = 2;

	x_obs[1] = 400; // pixels
	y_obs[1] = 270; // pixels
	size_obs[1] = 1.5; // scale factor 1.0 = 100% (not implemented yet)	

	x_obs[2] = 135; // pixels
	y_obs[2] = 205; // pixels
	size_obs[2] = 1.0; // scale factor 1.0 = 100% (not implemented yet)	

	// set robot model parameters ////////

	D = 121.0; // distance between front wheels (pixels)

	// position of laser in local robot coordinates (pixels)
	// note for Lx, Ly we assume in local coord the robot
	// is pointing in the x direction		
	Lx = 31.0;
	Ly = 0.0;

	// position of robot axis of rotation halfway between wheels (pixels)
	// relative to the robot image center in local coordinates
	Ax = 37.0;
	Ay = 0.0;

	alpha_max = 3.14159 / 2; // max range of laser / gripper (rad)

	// number of robot (1 - no opponent, 2 - with opponent, 3 - not implemented yet)
	n_robot = 2;

	cout << "\npress space key to begin program.";
	pause();

	// you need to activate the regular vision library before 
	// activating the vision simulation library
	activate_vision();

	// note it's assumed that the robot points upware in its bmp file

	// however, Lx, Ly, Ax, Ay assume robot image has already been
	// rotated 90 deg so that the robot is pointing in the x-direction
	// -- ie when specifying these parameters assume the robot
	// is pointing in the x-direction.

	// note that the robot opponent is not currently implemented in 
	// the library, but it will be implemented soon.

	activate_simulation(width1, height1, x_obs, y_obs, size_obs, N_obs,
		"robot_A.bmp", "robot_B.bmp", "background.bmp", "obstacle.bmp", D, Lx, Ly,
		Ax, Ay, alpha_max, n_robot);

	// open an output file if needed for testing or plotting
//	ofstream fout("sim1.txt");
//	fout << scientific;

	// set simulation mode (level is currently not implemented)
	// mode = 0 - single player mode (manual opponent)
	// mode = 1 - two player mode, player #1
	// mode = 2 - two player mode, player #2	
	mode = 0;
	level = 1;
	set_simulation_mode(mode, level);

	// set robot initial position (pixels) and angle (rad)
	x0 = 470;
	y0 = 170;
	theta0 = 0;
	set_robot_position(x0, y0, theta0);

	// set opponent initial position (pixels) and angle (rad)
	x0 = 400;
	y0 = 375;
	theta0 = 3.14159 / 4;
	set_opponent_position(x0, y0, theta0);

	// set initial inputs / on-line adjustable parameters /////////

	// inputs
	pw_l =1500; // pulse width for left wheel servo (us)
	pw_r = 1500; // pulse width for right wheel servo (us)
	pw_laser = 1500; // pulse width for laser servo (us)
	laser = 0; // laser input (0 - off, 1 - fire)

	// paramaters
	max_speed = 100; // max wheel speed of robot (pixels/s)
	opponent_max_speed = 100; // max wheel speed of opponent (pixels/s)

	// lighting parameters (not currently implemented in the library)
	light = 1.0;
	light_gradient = 1.0;
	light_dir = 1.0;
	image_noise = 1.0;

	// set initial inputs
	set_inputs(pw_l, pw_r, pw_laser, laser,
		light, light_gradient, light_dir, image_noise,
		max_speed, opponent_max_speed);

	// opponent inputs
	pw_l_o = 1500; // pulse width for left wheel servo (us)
	pw_r_o = 1500; // pulse width for right wheel servo (us)
	pw_laser_o = 1500; // pulse width for laser servo (us)
	laser_o = 0; // laser input (0 - off, 1 - fire)

	// manually set opponent inputs for the simulation
	// -- good for testing your program
	set_opponent_inputs(pw_l_o, pw_r_o, pw_laser_o, laser_o,
		opponent_max_speed);

	// regular vision program ////////////////////////////////

	// note that at this point you can write your vision program
	// exactly as before.

	// in addition, you can set the robot inputs to move it around
	// the image and fire the laser.

	int height, width;

	// note that the vision simulation library currently
	// assumes an image size of 640x480
	width = 640;
	height = 480;

	rgb.type = RGB_IMAGE;
	rgb.width = width;
	rgb.height = height;

	original.type = RGB_IMAGE;
	original.width = width;
	original.height = height;

	// set the type and size of the images
	black_mask.type = GREY_IMAGE;
	black_mask.width = width;
	black_mask.height = height;

	color_mask.type = GREY_IMAGE;
	color_mask.width = width;
	color_mask.height = height;

	temp_grey1.type = GREY_IMAGE;
	temp_grey1.width = width;
	temp_grey1.height = height;

	temp_grey2.type = GREY_IMAGE;
	temp_grey2.width = width;
	temp_grey2.height = height;

	label.type = LABEL_IMAGE;
	label.width = width;
	label.height = height;

	mask.type = GREY_IMAGE;
	mask.width = width;
	mask.height = height;

	// allocate memory for the images
	allocate_image(temp_grey1);
	allocate_image(temp_grey2);
	allocate_image(label);
	allocate_image(rgb);
	allocate_image(black_mask);
	allocate_image(original);
	allocate_image(color_mask);
	allocate_image(mask);

	// measure initial clock time
	tc0 = high_resolution_time();

	while (1) {

		// simulates the robots and acquires the image from simulation
		acquire_image_sim(original);

		copy(original, rgb);

		build_black_mask(rgb, black_mask, temp_grey1);

		// label the objects in a binary image
		// labels go from 1 to nlabels
		label_image(black_mask, label, nlabels);

		features(black_mask, original, label, nlabels, label_number, ic, jc, area, R_ave, G_ave, B_ave);

		int min_area = 2000;

		remove_small_areas(black_mask, label, area, min_area);

		build_color_mask(original, color_mask, temp_grey1);

		//copy(color_mask, rgb); 

		combine_masks(mask, color_mask, black_mask);

		//copy(mask, rgb); 

		//view_rgb_image(rgb);


		label_image(mask, label, nlabels);

		features(mask, original, label, nlabels, label_number, ic, jc, area, R_ave, G_ave, B_ave);


		for (int i = 1; i <= nlabels; i++) {

			items[i] = new Item(label_number[i], ic[i], jc[i], area[i], R_ave[i], G_ave[i], B_ave[i]);
		}

		set_robot_centroids(gx, gy, rx, ry, ox, oy, bx, by, items, nlabels);

		set_obstacle_centroids_and_radii(obs_ic, obs_jc, obs_r, items, nlabels, obs_count);


		getAngle(gx, gy, rx, ry, angle_self);
		getAngle(ox, oy, bx, by, angle_opp);
		getRobotPosition(gx, gy, rx, ry, x_position_self, y_position_self);
		getRobotPosition(ox, oy, bx, by, x_position_opp, y_position_opp);

		double selfToOppAngle;
		getAngle( x_position_opp, y_position_opp,gx, gy, selfToOppAngle);

		//pointturnToAngle(angle_self, selfToOppAngle, pw_l, pw_r); 

		

		
		
		getDistance(gx, gy, vantage_x, vantage_y, vantage_to_attacker); 

		if (vantage_to_attacker > 10) {
			cout <<vantage_to_attacker<<"\n";
			findVantagePoint(gx, gy, x_position_opp, y_position_opp, obs_ic, obs_jc, obs_r, obs_count, vantage_x, vantage_y);
			goTo_WithAvoidance(gx, gy, angle_self, vantage_x, vantage_y, pw_l, pw_r, obs_ic, obs_jc, obs_r, obs_count);
		}
		

		//check_to_fire(); 

		//goTo_WithAvoidance(gx, gy, angle_self, 370, 350, pw_l, pw_r, obs_ic, obs_jc, obs_r, obs_count);
		//draw_point_rgb(original,  370, 350, 0, 0, 255);

		

		//cout << "\n\n" << vantage_x << "\n" << vantage_y;

		// cout << "\n\n" << x_position_self<<"\n"<< y_position_self;


		/*
		cout << "\n gx:" << gx;
		cout << "\n gy:" << gy;
		cout << "\n rx:" << rx;
		cout << "\n ry:" << ry;
		cout << "\n ox:" << ox;
		cout << "\n oy:" << oy;
		cout << "\n bx" << bx; 
		cout << "\n by:" << by;
		*/


		//centroid checks for the labels


		for (int k = 1; k <= nlabels; k++) { //LOOP FOR TESTING

			// compute the centroid of the last object

			//cout << "\ncentroid: ic = " << ic[k] << " , jc = " << jc[k];
			//cout << "\narea: " << area[k]; 
			//cout << "\nR_ave: " << R_ave[k]; 
			//cout << "\nG_ave: " << G_ave[k];
			//cout << "\nB_ave: " << B_ave[k];
			//copy(mask, rgb);
			// mark the centroid point on the image


			R = 255; G = 0; B = 255;
			draw_point_rgb(original, (int)ic[k], (int)jc[k], R, G, B);
			draw_point_rgb(original, x_position_self, y_position_self, R, G, B);
			draw_point_rgb(original, x_position_opp, y_position_opp, R, G, B);

			//draw_point_rgb(original, obs_ic[2], obs_jc[2], R, G, B);


			// convert to RGB image format
			//view_rgb_image(rgb);
			//cout << "\nimage after a centroid is marked, label: "<<k;
			//pause();

		}
		//cout << "\n\n" << obs_r[1] << "\n" << obs_r[2]; 
		//pause(); 




		draw_point_rgb(original, vantage_x, vantage_y, 255, 255, 0);

		//draw_point_rgb(original, 390 , 336, 255, 255, 0);
		



	

			// change the inputs to move the robot around
			// or change some additional parameters (lighting, etc.)

			// only the following inputs work so far
			// pw_l -- pulse width of left servo (us) (from 1000 to 2000)
			// pw_r -- pulse width of right servo (us) (from 1000 to 2000)
			// pw_laser -- pulse width of laser servo (us) (from 1000 to 2000)
			// -- 1000 -> -90 deg
			// -- 1500 -> 0 deg
			// -- 2000 -> 90 deg
			// laser -- (0 - laser off, 1 - fire laser for 3 s)
			// max_speed -- pixels/s for right and left wheels
		set_inputs(pw_l, pw_r, pw_laser, laser,
			light, light_gradient, light_dir, image_noise,
			max_speed, opponent_max_speed);

		// manually set opponent inputs for the simulation
		// -- good for testing your program
		set_opponent_inputs(pw_l_o, pw_r_o, pw_laser_o, laser_o,
			opponent_max_speed);

		view_rgb_image(original);

		// don't need to simulate too fast
		Sleep(10); // 100 fps max

		reset_items(items, nlabels);

	}

	// free the image memory before the program completes
	free_image(original);
	free_image(temp_grey1);
	free_image(temp_grey2);
	free_image(label);
	free_image(rgb);
	free_image(mask);
	free_image(black_mask);
	free_image(color_mask);


	deactivate_vision();

	deactivate_simulation();

	cout << "\ndone.\n";

	return 0;
}






