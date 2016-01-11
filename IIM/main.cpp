#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

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
#include "IOController.h"
#include "Epipolar.h"
#include "FeatureExtractor.h"

void main()
{
	/*
		Set-up variables for image matching
	*/

	string detector, descriptor, matcher;
	string config_file;

	detector = "STAR";
	descriptor = "BRISK";
	matcher = "BruteForce-Hamming";

	config_file = "D:/Experiment/Set_3_SIDEVIEW/Input_SV_REF/config.txt";

	FeatureExtractor test_im
		(detector, descriptor, matcher, config_file);

	test_im.matchImgSeq();



	cout << "the end" << endl;


}
