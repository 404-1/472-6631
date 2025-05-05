#include <iostream>
#include <fstream>

#include <cmath>
#include <Windows.h>

using namespace std;

// include this header file for basic image transfer functions
#include "image_transfer.h"

// include this header file for computer vision functions
#include "vision.h"

#include "vision2.h"

image rgb, color_mask, black_mask, mask, temp_grey1, temp_grey2, label;

int OBSTACLE_RADIUS_TOLERANCE = 40; 

const int maxlabels = 50;
double ic[maxlabels + 1];
double jc[maxlabels + 1];
double area[maxlabels + 1];
double R_ave[maxlabels + 1];
double G_ave[maxlabels + 1];
double B_ave[maxlabels + 1];
int label_number[maxlabels + 1];

Item* items[maxlabels + 1]; //array of pointers to item objects

int nlabels;

void set_obstacle_centroids_and_radii(double obs_ic[], double  obs_jc[], double obs_r[], Item* items[], int nlabels, int& obs_count) {

	int min_area = 1000, count = 0;

	for (int i = 1; i <= nlabels; i++) {
		Item* it = items[i];

		if (it->area > min_area) {
			count++;
			obs_ic[count] = it->ic;
			obs_jc[count] = it->jc;
			obs_r[count] = sqrt(it->area / 3.14159)+ OBSTACLE_RADIUS_TOLERANCE;
		}
	}
	obs_count = count;
}


void reset_items(Item* items[], int nlabels) {
	for (int i = 1; i <= nlabels; ++i) {
		delete items[i];
		items[i] = nullptr;
	}
}


void set_robot_centroids(int& gx, int& gy, int& rx, int& ry, int& ox, int& oy, int& bx, int& by, Item* items[], int nlabels) {

	for (int i = 1; i <= nlabels; i++) {
		Item* it = items[i];
		int max_area = 1000;

		if (items[i]->area > max_area) continue;


		// green marker for self front
		if (it->R < 100 && it->G > 170 && it->B < 150) {
			gx = it->ic; 
			gy = it->jc;
		}
		// red marker for self back
		else if (it->R > 200 && it->G < 110 && it->B < 100) {
			rx = it->ic;
			ry = it->jc;
		}
		// orange marker for opp front
		else if (it->R > 200 && it->G > 100 && it->B < 150) {
			ox = it->ic;
			oy = it->jc;
		}
		// blue marker for opp back
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

	dialate(color_mask, temp_grey);
	copy(temp_grey, color_mask);

	dialate(color_mask, temp_grey);
	copy(temp_grey, color_mask);

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

	copy(temp_grey, rgb);    
	//view_rgb_image(rgb);
	
	//scaling

	scale(temp_grey, black_mask);
	copy(black_mask, temp_grey);

	//copy(temp_grey, rgb);    
	//view_rgb_image(rgb);

	//filtering

	lowpass_filter(temp_grey, black_mask);
	copy(black_mask, temp_grey);

	//copy(temp_grey, rgb);    
	//view_rgb_image(rgb);

	//threshold

	const int DARK_THRESH = 70;  // change as needed
	// bright (>=DARK_THRESH) = 255, dark (<DARK_THRESH) =0
	threshold(temp_grey, black_mask, DARK_THRESH);
	copy(black_mask, temp_grey);

	//copy(temp_grey, rgb); 
	//view_rgb_image(rgb);

	// invert the image
	invert(temp_grey, black_mask);
	copy(black_mask, temp_grey);

	//copy(temp_grey, rgb);    
	//view_rgb_image(rgb);

	//erode

	erode(temp_grey, black_mask);
	copy(black_mask, temp_grey);

	erode(temp_grey, black_mask);
	copy(black_mask, temp_grey);


	//copy(temp_grey, rgb);    
	//view_rgb_image(rgb);

	// perform a dialation function to fill in 
	// and grow the objects
	dialate(temp_grey, black_mask);
	copy(black_mask, temp_grey);

	copy(temp_grey, rgb);   
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


void run_vision(image original, int& gx, int& gy, int& rx, int& ry, int& ox, int& oy, int& bx,
	int& by, double obs_ic[], double obs_jc[], double obs_r[], int& obs_count) {

	copy(original, rgb);

	build_black_mask(rgb, black_mask, temp_grey1);

	label_image(black_mask, label, nlabels);

	features(black_mask, original, label, nlabels, label_number, ic, jc, area, R_ave, G_ave, B_ave);

	int min_area = 2000;

	remove_small_areas(black_mask, label, area, min_area); //the only black items we need to keep are the obstacles 

	build_color_mask(original, color_mask, temp_grey1);

	combine_masks(mask, color_mask, black_mask);

	label_image(mask, label, nlabels);

	features(mask, original, label, nlabels, label_number, ic, jc, area, R_ave, G_ave, B_ave);

	for (int i = 1; i <= nlabels; i++) {

		items[i] = new Item(label_number[i], ic[i], jc[i], area[i], R_ave[i], G_ave[i], B_ave[i]);
	}

	set_robot_centroids(gx, gy, rx, ry, ox, oy, bx, by, items, nlabels);

	set_obstacle_centroids_and_radii(obs_ic, obs_jc, obs_r, items, nlabels, obs_count);

	reset_items(items, nlabels);

}

void init_vision() {
	int height, width;

	// note that the vision simulation library currently
	// assumes an image size of 640x480
	width = 640;
	height = 480;

	rgb.type = RGB_IMAGE;
	rgb.width = width;
	rgb.height = height;

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
	allocate_image(color_mask);
	allocate_image(mask);
}


void cleanup_vision() {
	free_image(temp_grey1);
	free_image(temp_grey2);
	free_image(label);
	free_image(rgb);
	free_image(mask);
	free_image(black_mask);
	free_image(color_mask);


}