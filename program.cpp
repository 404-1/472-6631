
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

#include "attack_v3.h"


extern robot_system S1;

int main()
{

	image original; 
	
	original.type = RGB_IMAGE;
	original.width = 640;
	original.height = 480;
	
	allocate_image(original);


	//int cam_number;
	
	double obs_ic[50], obs_jc[50], obs_r[50]; 
	int obs_count;
	

	int gx, gy, rx, ry, ox, oy, bx, by;
	

	
	///////////////////////////////////////////////////////////////////////////////////

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

	x_obs[2] = 165; // pixels
	y_obs[2] = 295; // pixels
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
		"robot_A.bmp", "robot_B.bmp", "background.bmp", "obstacle_green.bmp", D, Lx, Ly,
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
	x0 = 570;
	y0 = 270;
	theta0 = 0;
	set_robot_position(x0, y0, theta0);

	// set opponent initial position (pixels) and angle (rad)
	x0 = 300;
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
	pw_l_o = 1300; // pulse width for left wheel servo (us)
	pw_r_o = 1300; // pulse width for right wheel servo (us)
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
	

	// measure initial clock time
	tc0 = high_resolution_time();


	


	init_vision(); //create/allocate all image structures we will be needing. see vision2
	while (1) {


		acquire_image_sim(original);
		
		run_vision(original, gx, gy, rx, ry, ox, oy, bx, by, obs_ic, obs_jc, obs_r, obs_count);  //run our vision program and extract all relevant information from the original image


		run_attack(gx, gy, rx, ry, ox, oy, bx, by, obs_ic, obs_jc, obs_r, obs_count, pw_l, pw_r, laser); 
		


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

		
	}

	// free the image memory before the program completes
	free_image(original);
		
	cleanup_vision(); 

	deactivate_vision();

	deactivate_simulation();

	cout << "\ndone.\n";

	return 0;
}






