#ifndef _FEATURE_EXTRACTOR_H_
#define _FEATURE_EXTRACTOR_H_

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
#include "IOController.h"
#include "Epipolar.h"

using namespace std;
using namespace cv;
using namespace arma;

class FeatureExtractor
{
private :

	/* Pointers to the feature point detector, descriptor, matcher */
	// feature detector 
	Ptr<FeatureDetector> detector;
	// feature descriptor
	Ptr<DescriptorExtractor> descriptor;
	// descriptor matcher
	Ptr<DescriptorMatcher> matcher;

	/* Parameters to the keypoint filtering setup */
	// NN ratio test
	float ratio_thresh;
	// Epipolar geometry test
	float epi_thresh;
	// Max number of matches
	int max_matches;

public :
	
	/* IOController for Feature Extractor */
	IOController IOhelper;

	/* Input of Feature Extraction */
	vector<string> in_images;  // list of input image file name
	vector<EO> in_eop;         // list of input EOs of images
	IO in_iop;                 // input IO

	
	/* Output of Feature Extraction */
	vector<vector<DMatch>> out_matches;  // list of matches
	vector<vector<int>> out_gpid;        // list of index of ground points
	vector<vector<KeyPoint>> out_kps;    // list of extracted keypoints


	
	/* Constructor of object class */
	/*
		 1) supported detectors in OpenCV
		 "FAST"
		 "STAR"
		 "SIFT"
		 "SURF"
		 "ORB"
		 "BRISK"
		 "MSER"
		 "GFTT"
		 "HARRIS"
		 "Dense"
		 "SimpleBlob"

		 2) supported descriptors in OpenCV
		 "SIFT"
		 "SURF"
		 "BRIEF"
		 "BRISK"
		 "ORB"
		 "FREAK"

		 3) supported matcher in OpenCV
		 "BruteForce"
		 "BruteForce-L1"
		 "BruteForce-Hamming"
		 "BruteForce-Hamming(2)"
		 "FlannBased"
	*/
	FeatureExtractor(
		string& detect, 
		string& describe, 
		string& match,
		string& in_config);

	/* Main function of object class */
	void matchImgSeq(/* input */ vector<String>& imgList,
		/* output */vector<vector<DMatch>>& matches,
		vector<vector<int>>& GPID,
		vector<vector<KeyPoint>>& keypoints);

	void matchImgSeq();


	/* Filtering functions */
	// Check 2NN ratio
	int checkNNratio(vector<vector<DMatch>> &matches);

	// Check Symmetricity
	void checkSymmetricity
	(vector<vector<DMatch>>& matches1, 
	vector<vector<DMatch>>& matches2, 
	vector<DMatch>& symMatches);

	// Check cross Epipolar geometry
	void checkEpipolar
		(vector<DMatch>& matches,
		vector<KeyPoint>& keypoint1,
		vector<KeyPoint>& keypoint2,
		EO& eo1,
		EO& eo2,
		IO& io,
		vector<DMatch>& validMatches,
		vector<Vec3f>& elines1,
		vector<Vec3f>& elines2);


	/* Supporting functions */
	// Set train(2nd) image ID
	void setImgID(vector<DMatch>& matches, int imgID);

	void setGPID
		(vector<DMatch>& prevMatches,
		vector<DMatch>& curMatches,
		vector<int>& prevGPID,
		vector<int>& curGPID,
		vector<KeyPoint>& curKeypointA,
		vector<KeyPoint>& curKeypointB);

	void setGPID
		(vector<DMatch>& initMatches,
		vector<int>& initGPID,
		vector<KeyPoint>& curKeypointA,
		vector<KeyPoint>& curKeypointB);

	// DUMMY FUNCTION
	//void setCorrectMatchesFlag
	//	(/*input*/ vector<vector<DMatch>>& matches1to2,
	//	vector<DMatch>& corrMatches,
	//	/*output*/ vector<vector<uchar>>& corrMatchesMask);

	void computeRepeatability
		(/* input */
		cv::Mat& img1,
		cv::Mat& img2,
		vector<DMatch>& matches, 
		vector<KeyPoint>& keypoint1,
		vector<KeyPoint>& keypoint2,
		/* output */
		float& repeatability, int& corrCount);

	void performANMS
		(/*input*/ 
		vector<KeyPoint>& keypoints,
		const int k,
		vector<int>& idx,
		cv::Size imgSize);

	int dist
		(/*input*/ const cv::Point2f& a, const cv::Point2f& b);

	// DUMMY FUNCTION
	//void detectGridImg
	//	(/*input*/
	//	Ptr<cv::FeatureDetector>& detector, 
	//	cv::Mat& img, vector<KeyPoint>& kp);

	cv::Mat drawEpiLine(
		/*input*/
		cv::Mat img1,
		cv::Mat img2,
		vector<DMatch>& goodmatches, 
		vector<KeyPoint>& keypoint1,
		vector<KeyPoint>& keypoint2,
		vector<Vec3f> eplines1,
		vector<Vec3f> eplines2);

	cv::Mat drawOpticFlow(
		/* input */
		cv::Mat img2,
		vector<DMatch>& goodmatches, 
		vector<KeyPoint>& keypoint1,
		vector<KeyPoint>& keypoint2);

	cv::Mat drawMatchingPair(
		/* input */
		cv::Mat img1,
		cv::Mat img2,
		vector<DMatch>& goodmatches, 
		vector<KeyPoint>& keypoint1,
		vector<KeyPoint>& keypoint2);


	void tooMany(
		/* input */vector<DMatch>& matches, 
		/* output */vector<DMatch>& maxMatches);

};

#endif
