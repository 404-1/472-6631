
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

void attack(
	int gx, int gy, int rx, int ry,
	int ox, int oy, int bx, int by,
	double width1, double height1,
	double& pw_l, double& pw_r, double& laser, bool& laser_fire,
	double obs_ic[], double obs_jc[], double obs_r[], int obs_count
) {
	bool aligned = false, clear_dist = false, clear_angle_front = false, clear_angle_back = false, clear = false;
	bool obs_to_right = false, obs_to_left = false, obs_close = false;
	bool boundary_detected = false;
	double margin = 80.0;

	double dist_s, angle_s;
	double dist_or, angle_or;
	double dist_r, angle_r_f, angle_r_b;
	double dist_obs, angle_obs;

	// Core distances and angles
	angle_distance(gx, gy, rx, ry, dist_s, angle_s); // self robot
	angle_distance(ox, oy, bx, by, dist_or, angle_or); // opponent robot
	angle_distance(ox, oy, gx, gy, dist_r, angle_r_f); // distance between robots (front)
	angle_distance(ox, oy, rx, ry, dist_r, angle_r_b); // distance between robots (back)

	// Find closest obstacle
	for (int i = 1; i <= obs_count; i++) {
		angle_distance(obs_ic[i], obs_jc[i], gx, gy, dist_obs, angle_obs);

		//checks if obstacle is closer than target
		if (dist_obs <= dist_r) clear_dist = false; //maybe blocked. more checks
		else clear = true; //clear for sure since its after the target

		//check if obstacle is cutting the linear path
		if (!clear_dist) {
			//check projection so if its cuts the linear path from robots
			projection_blocked(gx, gy, ox, oy, obs_ic[i], obs_jc[i], obs_r[i], clear_angle_front); // front
			projection_blocked(gx, gy, bx, by, obs_ic[i], obs_jc[i], obs_r[i], clear_angle_back); // back

			//checks if its blocked and give a detour point away from obstacle
			if (!clear_angle_front || !clear_angle_back) {
				clear = false;
			}
			else if (clear_angle_front && clear_angle_back) {
				clear = true;
			}
		}

		////check if obstacle is close to robot
		//if (!clear && (dist_obs <= (obs_r[i] + margin))) obs_close = true;
		//if (!clear && (dist_obs > (obs_r[i] + margin))) {
		//	obs_close = false;
		//	obs_to_right = false;
		//	obs_to_left = false;
		//}

		//if (obs_close) {
		//	if (angle_s <= angle_obs) { //should go ccw at 90 degree around obstacle
		//		obs_to_right = false;
		//		obs_to_left = true;
		//	}

		//	else { //should go cw at 90 degree around obstacle
		//		obs_to_right = true;
		//		obs_to_left = false;
		//	}
		//	if (clear) {
		//		obs_to_right = false;
		//		obs_to_left = false;
		//	}
		int obs_cooldown = 0;
		if (dist_obs <= (obs_r[i] + margin) && abs(angle_s - angle_obs) < 60) {
			obs_close = true;
			obs_cooldown = 20;
			pw_l = 1400;
			pw_r = 1200;
			//clockwise(pw_l, pw_r);
		}
			/*else if (dist_obs <= (obs_r[i] + margin)) {
				move_straight(pw_l, pw_r);
			}*/
			/*else if (dist_obs >= (obs_r[i] + margin + 1000)) {
				obs_close = false;
			}*/
		}

		// Check boundary
		edge_detection(width1, height1, 100, gx, gy, boundary_detected);

		// Check alignment with opponent
		if (abs(angle_s - abs(angle_r_b - angle_r_f)) <= 10.0)
			aligned = true;

		//if (obs_to_right) counterclockwise(pw_l, pw_r);
		////and move forward

		//if (obs_to_left) clockwise(pw_l, pw_r);
		////and move forward

		// Check projection from robot to opponent: is it blocked?
		//clear = false;
		//for (int i = 1; i <= obs_count; i++) {
		//	//bool clear_angle = false;

		//	projection_blocked(gx, gy, ox, oy, obs_ic[i], obs_jc[i], obs_r[i], clear_angle_front); // front
		//	projection_blocked(gx, gy, bx, by, obs_ic[i], obs_jc[i], obs_r[i], clear_angle_back); // back
		//	if (!clear_angle_front || !clear_angle_back) {
		//		clear = false;
		//	}
		//	else if (clear_angle_front && clear_angle_back) {
		//		clear = true;
		//	}
		//}

		// Action Decisions

		if (boundary_detected) {
			rotate_to_center(width1, height1, gx, gy, angle_s, pw_l, pw_r);
			laser = 0;
			return;
		}

		/// If too close to obstacle then slow clockwise rotation
		/*if (dist_obs <= (obs_r + margin)) {
			pw_l = 1550;
			pw_r = 1450;
			laser = 0;
			return;
		}*/

		// If too close to opponent then stop
		if (dist_r < 150) {
			pw_l = 1500;
			pw_r = 1500;
			laser = 0;
			return;
		}

		// If all clear and aligned, move straight
		if (aligned && clear && !obs_close) {
			move_straight(pw_l, pw_r);
		}
		else if (!aligned && !obs_close){
			rotate_to_opponent(angle_s, angle_r_f, aligned, pw_l, pw_r);
		}

		// Fire laser
		if (aligned && clear && !laser_fire) {
			laser = 1;
			laser_fire = true;
		}
		else if (laser_fire) {
			pw_l = 1500;
			pw_r = 1500;
		}
		else {
			laser = 0;
		}
	}
// end attack

extern robot_system S1;

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

	x_obs[1] = 270; // pixels
	y_obs[1] = 270; // pixels
	size_obs[1] = 1.5; // scale factor 1.0 = 100% (not implemented yet)	

	x_obs[2] = 135; // pixels
	y_obs[2] = 135; // pixels
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
	x0 = 150;
	y0 = 375;
	theta0 = 3.14159 / 4;
	set_opponent_position(x0, y0, theta0);

	// set initial inputs / on-line adjustable parameters /////////

	// inputs
	pw_l = 1250; // pulse width for left wheel servo (us)
	pw_r = 2000; // pulse width for right wheel servo (us)
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
	pw_l_o = 1300; // pulse width for left wheel servo (us)
	pw_r_o = 1600; // pulse width for right wheel servo (us)
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

		attack(gx, gy, rx, ry, ox, oy, bx, by, width1, height1, pw_l, pw_r, laser, laser_fire, obs_ic, obs_jc, obs_r, obs_count);


		//if (nlabels > 6) {
		//	view_rgb_image(rgb);
		//	pause();
		//}

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


		for (int k = 1;k <= nlabels;k++) { //LOOP FOR TESTING

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

			//draw_point_rgb(original, obs_ic[2], obs_jc[2], R, G, B);


			// convert to RGB image format
			//view_rgb_image(rgb);
			//cout << "\nimage after a centroid is marked, label: "<<k;
			//pause();

		}
		//cout << "\n\n" << obs_r[1] << "\n" << obs_r[2]; 
		//pause(); 











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






