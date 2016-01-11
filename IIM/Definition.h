#ifndef _DEFINITION_H_
#define	_DEFINITION_H_

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


using namespace std;
using namespace cv;
using namespace arma;

struct EO
{
	int id_img;

	double
		Xc,
		Yc,
		Zc,
		omega,
		phi,
		kappa;

	EO()
	{
		id_img = -1;
		Xc = 0.0;
		Yc = 0.0;
		Zc = 0.0;
		omega = 0.0;
		phi = 0.0;
		kappa = 0.0;
	}

	EO(int imgID,
		double x, double y, double z,
		double om, double ph, double kp)
	{
		id_img = imgID;
		Xc = x;
		Yc = y;
		Zc = z;
		omega = om;
		phi = ph;
		kappa = kp;
	}
};

struct IO
{
	cv::Mat K;        // Upper-triangular intrinsic matrix K
	
	double f_pcs;         // Focal length in pixel coordinate system
	double f_ccs;         // Focal length in camera coordinate system (um)

	cv::Point2d pp_pcs;   // Calibrated principal point in pixel coordinate system (upper-left)
	cv::Point2d pp_ccs;   // Calibrated principal point in camera coordinate system

	cv::Size2d size_img;    // Size of image resolutions in pixel unit
	cv::Size2d size_pixel;  // Size of pixel in meter unit

	IO()
	{
		K = (cv::Mat_<double>(3, 3) <<
			0, 0, 0,
			0, 0, 0,
			0, 0, 0);

		f_pcs = 0.0;

		pp_pcs.x = 0.0;
		pp_pcs.y = 0.0;

		pp_ccs.x = 0.0;
		pp_ccs.y = 0.0;

		size_img.width = 0;
		size_img.height = 0;

		size_pixel.width = 0.0;
		size_pixel.height = 0.0;
	}

	IO(double k11, double k12, double k13,
		double k21, double k22, double k23,
		double k31, double k32, double k33,
		int w_img, int h_img, double px)
	{
		K = (cv::Mat_<double>(3, 3) <<
			k11, k12, k13,
			k21, k22, k23,
			k31, k32, k33);

		size_img.width = w_img;
		size_img.height = h_img;

		size_pixel.width = px;
		size_pixel.height = px;
		
		f_pcs = k11;
		f_ccs = k11 * px;

		cout << f_ccs << endl;

		pp_pcs.x = k13;
		pp_pcs.y = k23;

		pp_ccs.x = 0.0;
		pp_ccs.y = 0.0;
	}
};

enum ROT_ORDER
{
	XYZ = 1,
	ZYX = 2
};

struct sort_by_dist
{
	bool operator() (DMatch const &a, DMatch const &b) const
	{
		return a.distance < b.distance;
	}
};

#endif
