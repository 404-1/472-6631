
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

extern robot_system S1;

class Item {

public:

	int label;
	int ic, jc, area;
	double R, G, B;

	Item(int label, int ic, int jc, int area, double R, double G, double B);


};

void features(image& a, image& rgb, image& label, int n_labels, int label_number[], double ic[], double jc[], double area[],
	double R_ave[], double G_ave[], double B_ave[]);

//image a is the greyscaled and manipulated image (filtered, etc), rgb is the orig. image, label is the label image. 
//each of these images holds valuable info that the features function can use...

void build_black_mask(image& rgb, image& mask_black, image& temp_grey);

void remove_small_areas(image& grey, image& label, double count[], int min_area);

void build_color_mask(image& rgb, image& color_mask, image& temp_grey);

void combine_masks(image& mask, image& color_mask, image& black_mask);

void set_robot_centroids(int& gx, int& gy, int& rx, int& ry, int& ox, int& oy, int& bx, int& by, Item* items[], int nlabels);


int main()
{
	
	image temp_grey1, temp_grey2, rgb, black_mask, original, color_mask, mask; // declare some image structures
	image label;
	int cam_number;
	int R, G, B, nlabels;

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
	int pw_l, pw_r, pw_laser, laser;
	double light, light_gradient, light_dir, image_noise;
	double width1, height1;
	int N_obs, n_robot;
	double x_obs[50], y_obs[50], size_obs[50];
	double D, Lx, Ly, Ax, Ay, alpha_max;
	double tc, tc0; // clock time
	int mode, level;
	int pw_l_o, pw_r_o, pw_laser_o, laser_o;
	
	// TODO: it might be better to put this model initialization
	// section in a separate function
	
	// note that the vision simulation library currently
	// assumes an image size of 640x480
	width1  = 640;
	height1 = 480;
	
	// number of obstacles
	N_obs  = 2;

	x_obs[1] = 270; // pixels
	y_obs[1] = 270; // pixels
	size_obs[1] = 1.0; // scale factor 1.0 = 100% (not implemented yet)	

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
	
	alpha_max = 3.14159/2; // max range of laser / gripper (rad)
	
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

	activate_simulation(width1,height1,x_obs,y_obs,size_obs,N_obs,
		"robot_A.bmp","robot_B.bmp","background.bmp","obstacle.bmp",D,Lx,Ly,
		Ax,Ay,alpha_max,n_robot);	

	// open an output file if needed for testing or plotting
//	ofstream fout("sim1.txt");
//	fout << scientific;
	
	// set simulation mode (level is currently not implemented)
	// mode = 0 - single player mode (manual opponent)
	// mode = 1 - two player mode, player #1
	// mode = 2 - two player mode, player #2	
	mode = 0;
	level = 1;
	set_simulation_mode(mode,level);	
	
	// set robot initial position (pixels) and angle (rad)
	x0 = 470;
	y0 = 170;
	theta0 = 0;
	set_robot_position(x0,y0,theta0);
	
	// set opponent initial position (pixels) and angle (rad)
	x0 = 150;
	y0 = 375;
	theta0 = 3.14159/4;
	set_opponent_position(x0,y0,theta0);

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
	set_inputs(pw_l,pw_r,pw_laser,laser,
		light,light_gradient,light_dir,image_noise,
		max_speed,opponent_max_speed);

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
	width  = 640;
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

	while(1) {
		
		// simulates the robots and acquires the image from simulation
	acquire_image_sim(original);

	copy(original, rgb); 

	build_black_mask(rgb, black_mask, temp_grey1); 
		
	// label the objects in a binary image
	// labels go from 1 to nlabels
	label_image(black_mask,label,nlabels);

	features(black_mask, original, label, nlabels, label_number, ic, jc, area, R_ave, G_ave, B_ave);

	int min_area = 2000; 

	remove_small_areas(black_mask, label, area, min_area); 
	
	build_color_mask(original, color_mask, temp_grey1); 

	copy(color_mask, rgb); 

	combine_masks(mask, color_mask, black_mask); 

	copy(mask, rgb); 
	
	//view_rgb_image(rgb);
	
	
	label_image(mask, label, nlabels);

	features(mask, original, label, nlabels, label_number, ic, jc, area, R_ave, G_ave, B_ave);


	for (int i = 1; i <= nlabels; i++) {

		items[i] = new Item(label_number[i], ic[i], jc[i], area[i], R_ave[i], G_ave[i], B_ave[i]);
	}
	
	set_robot_centroids(gx, gy, rx, ry, ox, oy, bx, by, items, nlabels); 

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
	

	for(int k=1;k<=nlabels;k++) { //LOOP FOR TESTING

		// compute the centroid of the last object
		
		//cout << "\ncentroid: ic = " << ic[k] << " , jc = " << jc[k];
		//cout << "\narea: " << area[k]; 
		//cout << "\nR_ave: " << R_ave[k]; 
		//cout << "\nG_ave: " << G_ave[k];
		//cout << "\nB_ave: " << B_ave[k];
		//copy(mask, rgb);
		// mark the centroid point on the image
		
		
		R = 255; G = 0; B = 255;
		draw_point_rgb(original,(int)ic[k], (int)jc[k], R, G, B);

		
		
		// convert to RGB image format
		//view_rgb_image(rgb);
		//cout << "\nimage after a centroid is marked, label: "<<k;
		//pause();

	}
	





		







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
		set_inputs(pw_l,pw_r,pw_laser,laser,
			light,light_gradient,light_dir,image_noise,
			max_speed,opponent_max_speed);

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





void set_robot_centroids(int& gx, int& gy, int& rx, int& ry, int& ox, int& oy, int& bx, int& by, Item* items[], int nlabels) {

	for (int i = 1; i <= nlabels; i++) {
		Item* it = items[i];
		int max_area = 1000; 

		if (items[i]->area > max_area) continue; 


		// green marker
		if (it->R < 100 && it->G > 170 && it->B < 150) {
			gx = it->ic;
			gy = it->jc;
		}
		// red marker
		else if (it->R > 200 && it->G < 110 && it->B < 100) {
			rx = it->ic;
			ry = it->jc;
		}
		// orange marker
		else if (it->R > 200 && it->G > 100 && it->B < 150) {
			ox = it->ic;
			oy = it->jc;
		}
		// blue marker
		else if (it->R < 80 && it->G < 170 && it->B >200) {
			bx = it->ic;
			by = it->jc;
		}



	}


}

void combine_masks(image& mask, image& color_mask, image& black_mask) {

	int W = color_mask.width;
	int H = color_mask.height;
	int N = W * H;

	for (int i = 0; i < N; i++) {

		if (color_mask.pdata[i] == 255 || black_mask.pdata[i] == 255)
			mask.pdata[i] = 255;

		else
			mask.pdata[i] = 0;
	}

}

void build_color_mask(image& rgb, image& color_mask, image& temp_grey) {


	int W = rgb.width;
	int H = rgb.height;
	int N = W * H;

	ibyte* prgb = rgb.pdata;   // length = 3*N
	ibyte* pm = color_mask.pdata; // length = N

	for (int i = 0; i < N; i++, pm++) {
		int b = prgb[3 * i + 0];
		int g = prgb[3 * i + 1];
		int r = prgb[3 * i + 2];

		// orange test
		bool Orange = (r > 200 && g > 100 && b < 150);

		// blue test
		bool Blue = (r < 80 && g < 170 && b > 200);

		bool Red = (r > 200 && g < 110 && b < 100);

		bool Green = (r < 100 && g > 170 && b < 150);

		if (Orange || Blue || Red || Green) {
			*pm = 255;
		}

		else(*pm = 0);


	}

	dialate(color_mask,temp_grey); 
	copy(temp_grey, color_mask); 

	dialate(color_mask, temp_grey); 
	copy(temp_grey,color_mask); 
	
	dialate(color_mask, temp_grey);
	copy(temp_grey, color_mask);

	erode(color_mask, temp_grey);
	copy(temp_grey, color_mask);

	erode(color_mask, temp_grey);
	copy(temp_grey, color_mask);

	erode(color_mask, temp_grey);
	copy(temp_grey, color_mask);
	
	//copy(color_mask, rgb); 
	//view_rgb_image(rgb); 
	//pause(); 

}

void features(image& grey, image& rgb, image& label, int n_labels, int label_number[], double ic[], double jc[], double area[],
	double R_ave[], double G_ave[], double B_ave[]) {

	int width = grey.width;
	int height = grey.height;

	ibyte* p_rgb, * p0_rgb;
	i2byte* p_label;  // LABEL_IMAGE type (pixel values are 0 to 65535) (2 bit int)

	p0_rgb = rgb.pdata; //start of image 'rgb' image data
	p_label = (i2byte*)label.pdata; //casting to i2byte*, we can use pixel number to read from p_label data the label value (array of 16 byte values)

	int pixel_number, label_id;

	double* sumR = new double[n_labels + 1] {0};
	double* sumG = new double[n_labels + 1] {0};
	double* sumB = new double[n_labels + 1] {0};
	double* count = new double[n_labels + 1] {0};

	for (int j = 0; j < height; j++) {

		for (int i = 0; i < width; i++) {

			pixel_number = i + width * j;

			p_rgb = p0_rgb + 3 * pixel_number; //pointer to the kth pixel. each pixel has 3 bytes of rgb data. 

			label_id = p_label[pixel_number]; //label_id corresponds to the integer label given to the pixel the loop is on  
			//cout << label_id; 
			if (label_id > 0 && label_id <= n_labels) {

				sumB[label_id] += *(p_rgb + 0);
				sumG[label_id] += *(p_rgb + 1);
				sumR[label_id] += *(p_rgb + 2);
				count[label_id] += 1.0;
				label_number[label_id] = label_id;
			}
		}
	}

	for (int i = 1; i <= n_labels; ++i) {
		if (count[i] > 0.0) {
			centroid(grey, label, i, ic[i], jc[i]);
			area[i] = count[i];
			R_ave[i] = sumR[i] / count[i];
			G_ave[i] = sumG[i] / count[i];
			B_ave[i] = sumB[i] / count[i];
		}
		else {
			ic[i] = jc[i] = area[i] = 0.0;
			R_ave[i] = G_ave[i] = B_ave[i] = 0.0;
		}
	}

	delete[] sumR;
	delete[] sumG;
	delete[] sumB;
	delete[] count;
}


void build_black_mask(image& rgb, image& black_mask, image& temp_grey) {


	//  RGB to greyscale
	copy(rgb, temp_grey);

	copy(temp_grey, rgb);    // convert to RGB image format
	//view_rgb_image(rgb);
	

	//scaling

	scale(temp_grey, black_mask);
	copy(black_mask, temp_grey);

	copy(temp_grey, rgb);    // convert to RGB image format
	//view_rgb_image(rgb);
	


	//filtering

	lowpass_filter(temp_grey, black_mask);
	copy(black_mask, temp_grey);

	copy(temp_grey, rgb);    // convert to RGB image format
	//view_rgb_image(rgb);
	

	//threshold

	const int DARK_THRESH = 70;  // change as needed
	// bright (>=DARK_THRESH) -> 255, dark (<DARK_THRESH) -> 0
	threshold(temp_grey, black_mask, DARK_THRESH);
	copy(black_mask, temp_grey);

	copy(temp_grey, rgb); // convert to RGB image format
	//view_rgb_image(rgb);
	

	// invert the image
	invert(temp_grey, black_mask);
	copy(black_mask, temp_grey);

	copy(temp_grey, rgb);    // convert to RGB image format
	//view_rgb_image(rgb);
	
	//erode

	erode(temp_grey, black_mask);
	copy(black_mask, temp_grey);

	// peform one more erosion to make sure there aren't many
	// small objects
	erode(temp_grey, black_mask);
	copy(black_mask, temp_grey);


	copy(temp_grey, rgb);    // convert to RGB image format
	//view_rgb_image(rgb);
	
	// perform a dialation function to fill in 
	// and grow the objects
	dialate(temp_grey, black_mask);
	copy(black_mask, temp_grey);

	copy(temp_grey, rgb);    // convert to RGB image format
	//view_rgb_image(rgb);
	
}

void remove_small_areas(image& grey, image& label, double area[], int min_area) { //removes labels and turns pixels to black

	int W = grey.width;
	int H = grey.height;
	int N = W * H;
	int label_id;

	i2byte* l = (i2byte*)label.pdata;
	ibyte* g = grey.pdata;

	for (int n = 0; n < N; n++) { //dont need to do p = p0 + 3 * pixel_number since grey image has 1 byte per pixel

		label_id = l[n];

		if (area[label_id] < min_area) {
			g[n] = 0;
			l[n] = 0;
		}
	}
}

Item::Item(int label, int ic, int jc, int area, double R, double G, double B) {
	this->label = label;
	this->ic = ic;
	this->jc = jc;
	this->area = area;
	this->R = R;
	this->G = G;
	this->B = B;
}