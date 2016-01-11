#ifndef _IOCONTROLLER_H_
#define _IOCONTROLLER_H_

#include <opencv2\opencv.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\nonfree\features2d.hpp>
#include <opencv2\features2d\features2d.hpp>
#include <opencv2\calib3d\calib3d.hpp>

#include <armadillo.hpp>

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <iomanip>

#include <math.h>
#include <Windows.h>

#include "Definition.h"

using namespace std;
using namespace cv;
using namespace arma;

class IOController
{
private :
	string
		path_img,    // path of input image file configuration  
		path_eo,     // path of input EO file
		path_io,
		path_tp,
		path_log;     // path of input IO file

	ifstream
		is_config,  // input path list file
		is_img,     // input image list file
		is_eo,      // input EO list file
		is_io;      // input IO file expressed in upper-triangular intrinsic matrix K

	void readConfig();

public :
	string 
		path_imgFolder;   // path of input image folder

	string
		path_keypoints,
		path_epipolar,
		path_optical_flow,
		path_matching_pair;


	ofstream
		os_tp,   // output tie-point file
		os_log;  // output matching log file

	/* IOController constructor */
	// input - configuration text file which contains path of three input files in order
	IOController(string& config);

	void setImg(vector<string> &imgList);
	void setEO(vector<EO> &eoList);
	void setIO(IO &io);

};

#endif
