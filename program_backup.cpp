
// vision example #1:

// how to load and save an image using the image transfer 
// function library and access pixel information

#include <cstdio>
#include <iostream>
#include <fstream>

// allows the "image_transfer.h" and "vision.h" libraries to access
// some needed windows functions
#include <Windows.h>

using namespace std;

// this header file is for image transfer functions
#include "image_transfer.h"

// this header file is for computer vision functions
#include "vision.h"

int main()
{ 
	image rgb;
	ibyte *p, grey, R, G, B;
	int i, width, height, size; // 4 byte ints (-2 billion to 2 billion)

	// setup / initialize the image transfer library
	activate_vision();
	
	cout << "\npress any key to get an image";

	// pause and wait for the space key to be pressed
	// -- this function is defined in vision.cpp
	pause();

	// finds the size of the image in the file, *allocate image memory, 
	// and loads the image from a bitmap file (*.bmp) into memory
	set_rgb_image("a.bmp",rgb);

	// Note: this function assumes an rgb format (24 bits/pixel).
	// -- Don't use a rgba (32 bits/pixel) format.
	// If you have file in 32 bits/pixel format then use paint.net
	// to convert it to 24 bits/pixel.

	// view an rgb image in the image_view.exe program
	view_rgb_image(rgb);

	cout << "\noriginal image rgb";
	pause();

	// note: rgb.width and rgb.height are only 2 byte ints so
	// it's better to use 4 byte ints width and height to avoid
	// overflow errors when performing calculations with them
	// (eg size = rgb.width * rgb.height) -- oveflow error
	width = rgb.width;
	height = rgb.height;

	cout << "\nwidth = " << width;
	cout << "\nheight = " << height;

	// calculate the number of pixels in the image
	size = width*height;

	// get the pointer to the image data
	p = rgb.pdata; // p points to the first byte of the image

	// examine each pixel in the image
	
	// note: the format of the image (24 bit / pixel format) is given by 
	// B G R B G R ....
	// where R, G, and B are one byte integers that hold
	// values from 0 to 255 representing the colour component
	// for each pixel

	// access each pixel i of the image
	for(i=0;i<size;i++) {

		// get the colour components for pixel i
		B = *p;
		G = *(p+1);
		R = *(p+2);

		// this doesn't work -- overflow error since R, G, B
		// are 1 byte integers
//		grey = (B+G+R)/3; 

		// this works since ints are 4 byte integers
//		grey = (ibyte)( ( (int)B + (int)G + (int)R ) / 3.0 );
		// converting double to int rounds down x = 2.6 -> x = 2

		// this also works and is likely faster
		//grey = (ibyte)( ( (int)B + G + R ) / 3 );
		// int division rounds down eg 5/2 = 2

		//*p     = grey; // B
		//*(p+1) = grey; // G
		//*(p+2) = grey; // R

		// the old "switcheroo"
//		*p     = R; // B
//		*(p+1) = G; // G
//		*(p+2) = B; // R

		// move 3 bytes forward to the next pixel
		p += 3;
	}

	view_rgb_image(rgb);

	cout << "\noutput image rgb";
	pause();

	// save the image into a *.bmp file with rgb (24 bit/pixel format)
	save_rgb_image("aout.bmp",rgb);

	// stop using the vision image transfer library
	deactivate_vision();

	// delete the image memory before the program completes
	free_image(rgb);
	
	cout << "\n\ndone.\n";
	pause();

 	return 0;
}

