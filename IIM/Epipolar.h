#ifndef _EPIPOLAR_H_
#define _EPIPOLAR_H_

#include <opencv2\opencv.hpp>
#include <armadillo.hpp>

#include <vector>
#include <string>
#include <math.h>

#include "Definition.h"

using namespace std;
using namespace arma;
using namespace cv;

class Epipolar
{
private :
	EO eo1, eo2;
	IO io;
	double threshold;

	arma::mat::fixed<3, 1> epipole1, epipole2;
	arma::mat::fixed<4, 1> img_plane1, img_plane2;

	arma::mat::fixed<3, 3> computeRotation
		(/* input */ double om, double ph, double kp, ROT_ORDER flag);
	
	arma::mat::fixed<4, 1> computeImgPlane(/* input */ EO& eo, IO& io);

	arma::mat::fixed<3, 1> convertIP2MCS(EO& eo, IO& io, cv::Point2f& ip);

	cv::Point2f convertIP2CCS(EO& eo, IO& io, arma::mat::fixed<3, 1>& ip_mcs);

	arma::mat::fixed<4, 1> determineEpipolarPlane
		(/* input */ EO& eo, IO& io, arma::mat::fixed<3, 1>& epp, cv::Point2f& ip);

	void computeEpipole
		(/* output */ 
		arma::mat::fixed<3, 1>& epp1, arma::mat::fixed<3, 1>& epp2,
		arma::mat::fixed<4, 1>& img1, arma::mat::fixed<4, 1>& img2);

	// Compute distance between plane and point
	double computeDistance
		(/* input */ 
		EO& eo, IO& io,
		arma::mat::fixed<4, 1>& img_plane, 
		arma::mat::fixed<4, 1>& epi_plane,
		arma::mat::fixed<3, 1>& epipole,
		arma::mat::fixed<3, 1>& img_point,
		/* output */
		Vec3f& el_dir);


public :
	vector<Vec3f> elines1, elines2;

	bool checkCrossEpipolar
		(/* input */ cv::Point2f ip1, cv::Point2f ip2);

	Epipolar(EO& eo1, EO& eo2, IO& io, double th);

};

#endif
