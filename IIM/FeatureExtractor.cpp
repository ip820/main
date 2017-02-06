#include <opencv2\opencv.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\nonfree\features2d.hpp>
#include <opencv2\features2d\features2d.hpp>
#include <opencv2\calib3d\calib3d.hpp>

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <iomanip>
#include <algorithm>
#include <utility>
#include <random>

#include <math.h>
#include <Windows.h>

#include <armadillo.hpp>

#include "Definition.h"
#include "IOController.h"
#include "FeatureExtractor.h"
#include "Timer.h"
#include "Epipolar.h"

using namespace std;
using namespace cv;
using namespace arma;

/*
	*************************************************************************************
	Constructor of Feature Extractor
	*************************************************************************************
*/
FeatureExtractor::FeatureExtractor
	(string& detect, 
	string& describe, 
	string& match,
	string& in_config)
	: ratio_thresh(0.9f), epi_thresh(10000.0), IOhelper(in_config), max_matches(150)
{
	// Initialize feature detector
	if(detect == "SIFT")
		FeatureExtractor::detector = new SiftFeatureDetector;
	
	else if(detect == "SURF")
		FeatureExtractor::detector = new SurfFeatureDetector;
	
	else if(detect == "STAR")
	{
		FeatureExtractor::detector = new StarFeatureDetector;

		detector->setInt("maxSize", 12);
		detector->setInt("suppressNonmaxSize", 6);

	}
	else if(detect == "Fast")
	{
		/*Ptr<FeatureDetector> base = new FastFeatureDetector;

		base->setBool("nonmaxSuppression", true);
		base->setInt("threshold", 100);*/
		//base->setInt("type", FastFeatureDetector::TYPE_9_16);

		Ptr<AdjusterAdapter> controller = new FastAdjuster;
		controller->setInt("init_thresh", 40);
		controller->setBool("nonmax", false);

		Ptr<FeatureDetector> base = new DynamicAdaptedFeatureDetector(controller, 300, 600, 3);

		FeatureExtractor::detector = new PyramidAdaptedFeatureDetector(base, 2);;
	}
	else
	{		
		FeatureExtractor::detector = FeatureDetector::create(detect);

		// detector->setBool("nonmax", false);

		//FeatureExtractor::detector = 
		//	new GridAdaptedFeatureDetector(pyramid_detector, 500, 2, 5);

		//FeatureExtractor::detector = FeatureDetector::create(detect);
	}

	// Initialize featrue descriptor
	if(describe == "SIFT")
		FeatureExtractor::descriptor = new SiftDescriptorExtractor;
	
	else if(describe == "SURF")
		FeatureExtractor::descriptor = new SurfDescriptorExtractor;
	
	else
		FeatureExtractor::descriptor = DescriptorExtractor::create(describe);

	// Initialize descriptor matcher
	FeatureExtractor::matcher = DescriptorMatcher::create(match);


	// Read & Set input data
	FeatureExtractor::IOhelper.setImg(FeatureExtractor::in_images);
	FeatureExtractor::IOhelper.setEO(FeatureExtractor::in_eop);
	FeatureExtractor::IOhelper.setIO(FeatureExtractor::in_iop);

}



/*
	*************************************************************************************
	Filtering Functions of Feature Extractor
	1 ) 2NN ratio test
	2 ) Symmetricity test
	3 ) Epipolar geometry cross test
	4 ) too Many - select specified number of best matches
	*************************************************************************************
*/
int FeatureExtractor::checkNNratio
	(vector<vector<DMatch>> &matches)
{
	int removed_num = 0;

	float ratio_temp;
	vector<vector<DMatch>>::iterator matcheIterator;
	
	for(matcheIterator = matches.begin();
		matcheIterator != matches.end();
		++matcheIterator)
	{
		if(matcheIterator->size() > 1)
		{
			ratio_temp = (*matcheIterator)[0].distance / (*matcheIterator)[1].distance;
			
			if(ratio_temp > ratio_thresh)
			{
				matcheIterator->clear();
				removed_num++;
			}
		}

		else
		{
			matcheIterator->clear();
			removed_num++;
		}
	}

	return removed_num;
}

void FeatureExtractor::checkSymmetricity
	(vector<vector<DMatch>>& matches1, 
	vector<vector<DMatch>>& matches2, 
	vector<DMatch>& symMatches)
{
	vector<vector<DMatch>>::iterator matcheIterator1;
	vector<vector<DMatch>>::iterator matcheIterator2;

	for(matcheIterator1 = matches1.begin(); 
		matcheIterator1 != matches1.end();
		++matcheIterator1)
	{
		if(matcheIterator1->size() < 2)
			continue;

		for(matcheIterator2 = matches2.begin();
			matcheIterator2 != matches2.end();
			++matcheIterator2)
		{
			if(matcheIterator2->size() < 2)
				continue;

			if((*matcheIterator1)[0].queryIdx == (*matcheIterator2)[0].trainIdx
				&& (*matcheIterator2)[0].queryIdx == (*matcheIterator1)[0].trainIdx)
			{
				symMatches.push_back(DMatch((*matcheIterator1)[0].queryIdx, (*matcheIterator1)[0].trainIdx, (*matcheIterator1)[0].distance));
				break;
			}
		}
	}
}

void FeatureExtractor::checkEpipolar
	(vector<DMatch>& matches,
	vector<KeyPoint>& keypoint1,
	vector<KeyPoint>& keypoint2,
	EO& eo1,
	EO& eo2,
	IO& io,
	vector<DMatch>& validMatches,
	vector<Vec3f>& eplines1,
	vector<Vec3f>& eplines2)
{
	vector<Point2f> pt1(matches.size());
	vector<Point2f> pt2(matches.size());

	vector<DMatch>::const_iterator iter_matches;

	Epipolar epi(eo1, eo2, io, FeatureExtractor::epi_thresh);

	float x1, y1;
	float x2, y2;

	int idx_pts = 0;

	for(iter_matches = matches.begin();
		iter_matches != matches.end();
		++iter_matches)
	{
		x1 = keypoint1[iter_matches->queryIdx].pt.x;
		y1 = keypoint1[iter_matches->queryIdx].pt.y;
		pt1.at(idx_pts) = Point2f(x1, y1);

		x2 = keypoint2[iter_matches->trainIdx].pt.x;
		y2 = keypoint2[iter_matches->trainIdx].pt.y;
		pt2.at(idx_pts) = Point2f(x2, y2);

		idx_pts++;
	}

	// Tag flag to inliers
	vector<uchar> inliers(pt1.size(), 0);

	for(idx_pts = 0; idx_pts < pt1.size() && idx_pts < pt2.size(); idx_pts++)
	{
		if(epi.checkCrossEpipolar(pt1.at(idx_pts), pt2.at(idx_pts)))
			inliers.at(idx_pts) = 'r';
		else
			continue;
	}

	// Return valid matches
	vector<uchar>::const_iterator iter_inliers = inliers.begin();
	iter_matches = matches.begin();

	for( ; iter_inliers != inliers.end(); ++iter_inliers, ++iter_matches)
	{
		if(*iter_inliers)
		{
			validMatches.push_back(*iter_matches);
		}
	}

	// Return Epipolar Lines for valid matches
	eplines1 = epi.elines1;
	eplines2 = epi.elines2;

}

void FeatureExtractor::tooMany(
		/* input */vector<DMatch>& matches, 
		/* output */vector<DMatch>& maxMatches)
{
	std::sort(matches.begin(), matches.end(), sort_by_dist());

	for(int i = 0; i < FeatureExtractor::max_matches; i++)
	{
		maxMatches.push_back(matches.at(i));
	}

	cout << "wait" << endl;

}



/*
	*************************************************************************************
	Supporting Functions of Feature Extractor
	*************************************************************************************
*/
void FeatureExtractor::setImgID(vector<DMatch>& matches, int imgID)
{
	for(int i = 0; i < matches.size(); i++)
	{
		matches[i].imgIdx = imgID;
	}
}

void FeatureExtractor::setGPID
	(vector<DMatch>& prevMatches,
	vector<DMatch>& curMatches,
	vector<int>& prevGPID,
	vector<int>& curGPID,
	vector<KeyPoint>& curKeypointA,
	vector<KeyPoint>& curKeypointB)
{
	int GPID;
	int maxGPID;
	int imgID = curMatches[0].imgIdx - 1;

	if((curMatches[0].imgIdx - prevMatches[0].imgIdx == 1) && (curMatches[0].imgIdx != 1))
	{
		vector<DMatch>::const_iterator prevMIter;
		vector<DMatch>::const_iterator curMIter;

		vector<int>::const_iterator prevGPIter, curGPIter;

		prevGPIter = max_element(prevGPID.begin(), prevGPID.end());
		maxGPID = prevGPID.at(prevGPIter - prevGPID.begin());

		curGPID.clear();

		for(curMIter = curMatches.begin();
			curMIter != curMatches.end();
			++curMIter)
		{
			for(prevMIter = prevMatches.begin();
				prevMIter != prevMatches.end();
				++prevMIter)
			{
				if(prevMIter->trainIdx == curMIter->queryIdx)
				{
					GPID = prevGPID.at(prevMIter - prevMatches.begin());
					break;
				}
			}

			if(prevMIter == prevMatches.end())
			{
				GPID = ++maxGPID;
			}
			curGPID.push_back(GPID);
		}


		bool gp_existence;

		for(int i = 0; i < curMatches.size(); i++)
		{
			gp_existence = false;

			for(int j = 0; j < prevGPID.size(); j++)
			{
				if(curGPID.at(i) == prevGPID.at(j))
				{
					gp_existence = true;
					break;
				}
			}

			if(!gp_existence)
			{
				FeatureExtractor::IOhelper.os_tp <<
					imgID + 1 << "\t" <<
					curGPID.at(i) << "\t" <<
					curKeypointA[curMatches.at(i).queryIdx].pt.x << "\t" <<
					curKeypointA[curMatches.at(i).queryIdx].pt.y << endl;
			}

		}

		imgID++;

		for(int k = 0; k < curMatches.size(); k++)
		{
			FeatureExtractor::IOhelper.os_tp <<
				imgID + 1 << "\t" <<
				curGPID.at(k) << "\t" <<
				curKeypointB[curMatches.at(k).trainIdx].pt.x << "\t" <<
				curKeypointB[curMatches.at(k).trainIdx].pt.y << endl;
		}
	}
}

void FeatureExtractor::setGPID
		(vector<DMatch>& initMatches,
		vector<int>& initGPID,
		vector<KeyPoint>& curKeypointA,
		vector<KeyPoint>& curKeypointB)
{
	int gpid;
	int imgID = initMatches[0].imgIdx - 1;

	for(int i = 0; i < initMatches.size(); i++)
	{
		gpid = i + 1;
		initGPID.push_back(gpid);
	}


	for(int j = 0; j < initMatches.size(); j++)
	{
		FeatureExtractor::IOhelper.os_tp <<
			imgID + 1 << "\t" <<
			initGPID.at(j) << "\t" <<
			curKeypointA[initMatches.at(j).queryIdx].pt.x << "\t" <<
			curKeypointA[initMatches.at(j).queryIdx].pt.y << endl;
	}

	imgID++;
	
	for(int j = 0; j < initMatches.size(); j++)
	{
		FeatureExtractor::IOhelper.os_tp <<
			imgID + 1 << "\t" <<
			initGPID.at(j) << "\t" <<
			curKeypointB[initMatches.at(j).trainIdx].pt.x << "\t" <<
			curKeypointB[initMatches.at(j).trainIdx].pt.y << endl;
	}
}

void FeatureExtractor::computeRepeatability
	(/* input */
		cv::Mat& img1,
		cv::Mat& img2,
		vector<DMatch>& matches, 
		vector<KeyPoint>& keypoint1,
		vector<KeyPoint>& keypoint2,
		/* output */
		float& repeatability, int& corrCount)
{
	vector<Point2f> pt1(matches.size());
	vector<Point2f> pt2(matches.size());

	vector<DMatch>::const_iterator iter_matches;

	cv::Mat homo;

	float x1, y1;
	float x2, y2;

	int idx_pts = 0;

	for(iter_matches = matches.begin();
		iter_matches != matches.end();
		++iter_matches)
	{
		x1 = keypoint1[iter_matches->queryIdx].pt.x;
		y1 = keypoint1[iter_matches->queryIdx].pt.y;
		pt1.at(idx_pts) = Point2f(x1, y1);

		x2 = keypoint2[iter_matches->trainIdx].pt.x;
		y2 = keypoint2[iter_matches->trainIdx].pt.y;
		pt2.at(idx_pts) = Point2f(x2, y2);

		idx_pts++;
	}

	homo = findHomography(pt1, pt2, CV_RANSAC, 3.0);

	evaluateFeatureDetector(img1, img2, homo, &keypoint1, &keypoint2, repeatability, corrCount);

}
int FeatureExtractor::dist
		(/*input*/ const cv::Point2f& a, const cv::Point2f& b)
{
	return (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y);
}

void FeatureExtractor::performANMS
		(/*input*/ 
		vector<KeyPoint>& keypoints,
		const int k,
		vector<int>& idx,
		cv::Size imgSize)
{
	int width = imgSize.width / 20;
	int height = imgSize.height / 20;
	int c_robust = 1;

	static vector<vector<int>> table;

	if(table.empty())
	{
		table.resize(width * height);

		for(int x = 0; x < width; ++x)
		{
			for(int y = 0; y < height; ++y)
			{
				int lx = x-1 < 0 ? 0 : x-1;
				int ly = y-1 < 0 ? 0 : y-1;

				int rx = x+1 >= width ? width-1 : x+1;
				int ry = y+1 >= height ? height-1 : y+1;

				for(int j = lx; j <= rx; ++j)
				{
					for(int k = ly; k <=ry; ++k)
					{
						table[y * width + x].push_back(k*width + j);
					}
				}

			}
		}
	}

	// mask1 - whether there has point in the 3 * 3 neiborhood
	// mask2 - whether the bucket[i] is empty

	vector<int> mask1, mask2;
	mask1.resize(width * height);
	mask2.resize(width * height);

	static vector<vector<int>> bucket;
	bucket.resize(width * height);

	for(size_t i = 0; i < bucket.size(); ++i)
	{
		bucket[i].clear();
	}

	std::sort(keypoints.begin(), keypoints.end(), 
		[](const KeyPoint &a, const KeyPoint &b){return a.response > b.response;});


	std::vector<std::pair<int, int>> radius;
	radius.reserve(keypoints.size());

	for(size_t i = 0; i < keypoints.size(); ++i)
	{
		int r = 0xffffff;
		int x, y;

		x = keypoints[i].pt.x / 20;
		y = keypoints[i].pt.y / 20;

		int idx = y * width + x;
		if(mask1[idx])
		{
			for(int k = 0; k < bucket[idx].size(); ++k)
			{
				int &j = bucket[idx][k];
				int temp;

				if(keypoints[i].response < c_robust * keypoints[j].response)
				{
					if((temp = dist(keypoints[i].pt, keypoints[j].pt)) < r)
						r = temp;
				}
			}
		}

		if(!mask2[idx])
		{
			for(int j = 0; j < table[idx].size(); ++j)
			{
				mask1[table[idx][j]] = 1;
			}
		}

		if(!mask2[idx])
		{
			mask2[idx] = 1;
		}

		for(int j = 0; j < table[idx].size(); ++j)
		{
			bucket[table[idx][j]].push_back(i);
		}

		radius.push_back(std::pair<int, int>(r, i));
	}

	sort(radius.begin(), radius.end(),
		[](const std::pair<int, int>& a, const std::pair<int, int>& b){return a.first > b.first;});

	idx.clear();
	idx.reserve(500);

	std::vector<KeyPoint> out_kps;
	out_kps.reserve(500);

	int size = min((size_t)k, keypoints.size());

	for(int i = 0; i < size; ++i)
	{
		out_kps.push_back(keypoints[radius[i].second]);
		idx.push_back(radius[i].second);
	}

	keypoints = out_kps;

}


//// DUMMY FUNCTIONS
//void FeatureExtractor::setCorrectMatchesFlag
//	(/*input*/ vector<vector<DMatch>>& matches1to2,
//	vector<DMatch>& corrMatches,
//	/*output*/ vector<vector<uchar>>& corrMatchesMask)
//{
//	int idxRow, idxCol;
//
//	if(matches1to2.size() == corrMatchesMask.size())
//	{
//		for(size_t i = 0; i < corrMatches.size(); i++)
//		{
//			idxRow = corrMatches[i].queryIdx;
//
//			for(size_t j = 0; j < matches1to2[idxRow].size(); j++)
//			{
//				if(matches1to2[idxRow][j].trainIdx == corrMatches[i].trainIdx)
//				{
//					corrMatchesMask[idxRow].push_back(1);
//				}
//				else
//				{
//					corrMatchesMask[idxRow].push_back(0);
//				}
//			}
//			
//		}
//	}
//}

//void FeatureExtractor::detectGridImg
//	(/*input*/
//		Ptr<cv::FeatureDetector>& detector, 
//		cv::Mat& img, vector<KeyPoint>& kp)
//{
//	int nGridRow, nGridCol;
//	nGridCol = 5;
//	nGridRow = 2;
//
//	int nCropRow, nCropCol;
//	nCropCol = img.cols / nGridCol;
//	nCropRow = img.rows / nGridRow;
//
//	cv::Rect cropROI;
//	cv::Mat cropImg;
//	vector<KeyPoint> cropkp;
//
//	for(int i = 0; i < nGridCol; i++)
//	{
//		for(int j = 0; j < nGridRow; j++)
//		{
//			cropROI = cv::Rect(nCropCol * i, nCropRow * j, nCropCol, nCropRow);
//
//			cropImg = img(cropROI).clone();
//
//			imwrite("crop.jpg", cropImg);
//
//			detector->detect(cropImg, cropkp);
//
//			kp.reserve(kp.size() + cropkp.size());
//			kp.insert(kp.end(), cropkp.begin(), cropkp.end());
//
//			cropkp.clear();
//			cropImg.release();
//		}
//	}
//
//}

cv::Mat FeatureExtractor::drawEpiLine(
		/*input*/
		cv::Mat img1,
		cv::Mat img2,
		vector<DMatch>& goodmatches, 
		vector<KeyPoint>& keypoint1,
		vector<KeyPoint>& keypoint2,
		vector<Vec3f> elines1,
		vector<Vec3f> elines2)
{
	vector<Point2f> pt1(goodmatches.size());
	vector<Point2f> pt2(goodmatches.size());

	vector<DMatch>::const_iterator iter_matches;

	float x1, y1;
	float x2, y2;

	int idx_pts = 0;

	cv::Mat imgA, imgB, imgAB;
	cv::cvtColor(img1, imgA, cv::COLOR_GRAY2RGB);
	cv::cvtColor(img2, imgB, cv::COLOR_GRAY2RGB);


	for(iter_matches = goodmatches.begin();
		iter_matches != goodmatches.end();
		++iter_matches)
	{
		x1 = keypoint1[iter_matches->queryIdx].pt.x;
		y1 = keypoint1[iter_matches->queryIdx].pt.y;
		pt1.at(idx_pts) = Point2f(x1, y1);

		x2 = keypoint2[iter_matches->trainIdx].pt.x;
		y2 = keypoint2[iter_matches->trainIdx].pt.y;
		pt2.at(idx_pts) = Point2f(x2, y2);

		idx_pts++;
	}


	for(vector<Vec3f>::const_iterator it1 = elines2.begin();
		it1 != elines2.end(); ++it1)
	{
		cv::line(imgA,
			cv::Point(0, -(*it1)[2] / (*it1)[1]),
			cv::Point(imgA.cols, 
			-((*it1)[2] + (*it1)[0] * imgA.cols) / (*it1)[1]),
			cv::Scalar(255, 0, 0));
	}

	for(int j = 0; j < pt1.size(); j++)
	{
		circle(imgA, pt1.at(j), 1, cv::Scalar(0, 250, 0), 5);
	}

	for(vector<Vec3f>::const_iterator it2 = elines1.begin();
		it2 != elines1.end(); ++it2)
	{
		cv::line(imgB,
			cv::Point(0, -(*it2)[2] / (*it2)[1]),
			cv::Point(imgB.cols, 
			-((*it2)[2] + (*it2)[0] * imgB.cols) / (*it2)[1]),
			cv::Scalar(255, 0, 0));
	}

	for(int i = 0; i < pt2.size(); i++)
	{
		circle(imgB, pt2.at(i), 1, cv::Scalar(0, 250, 0), 5);
	}

	cv::vconcat(imgA, imgB, imgAB);

	return imgAB;
}

cv::Mat FeatureExtractor::drawOpticFlow(
		/* input */
		cv::Mat img2,
		vector<DMatch>& goodmatches, 
		vector<KeyPoint>& keypoint1,
		vector<KeyPoint>& keypoint2)
{
	vector<DMatch>::const_iterator iter;
	
	cv::Mat img_res;
	cv::cvtColor(img2, img_res, cv::COLOR_GRAY2BGR);

	for(iter = goodmatches.begin();
		iter != goodmatches.end();
		++iter)
	{
		cv::Point2f pt_start, pt_end;
		cv::Point2f pt_left, pt_right;
		double angle;

		pt_start = keypoint1[iter->queryIdx].pt;
		pt_end = keypoint2[iter->trainIdx].pt;

		angle = atan2((double)pt_start.y - pt_end.y, (double)pt_start.x - pt_end.x);

		pt_right.x = (int) (pt_end.x + 15 * cos(angle + CV_PI / 4));
		pt_right.y = (int) (pt_end.y + 15 * sin(angle + CV_PI / 4));

		pt_left.x = (int) (pt_end.x + 15 * cos(angle - CV_PI / 4));
		pt_left.y = (int) (pt_end.y + 15 * sin(angle - CV_PI / 4));

		// draw main line of an arrow
		cv::line(img_res, pt_start, pt_end, Scalar(0, 0, 255), 2, 8, 0);

		// draw tip of an arrow
		cv::line(img_res, pt_right, pt_end, Scalar(0, 0, 255), 2, 8, 0);
		cv::line(img_res, pt_left, pt_end, Scalar(0, 0, 255), 2, 8, 0);
	}

	return img_res;
}

cv::Mat FeatureExtractor::drawMatchingPair(
		/* input */
		cv::Mat img1,
		cv::Mat img2,
		vector<DMatch>& goodmatches, 
		vector<KeyPoint>& keypoint1,
		vector<KeyPoint>& keypoint2)
{
	vector<DMatch>::const_iterator iter;

	cv::Mat img_res, img1_c, img2_c;

	cv::cvtColor(img1, img1_c, cv::COLOR_GRAY2BGR);
	cv::cvtColor(img2, img2_c, cv::COLOR_GRAY2BGR);

	cv::vconcat(img1_c, img2_c, img_res);

	std::random_device random_hw;
	std::mt19937 seed_b(random_hw()), seed_g(random_hw()), seed_r(random_hw());
	std::uniform_int_distribution<> range(0, 255);

	for(iter = goodmatches.begin();
		iter != goodmatches.end();
		++iter)
	{
		cv::Point2f pt_query, pt_train;
		pt_query = keypoint1[iter->queryIdx].pt;

		pt_train.x = keypoint2[iter->trainIdx].pt.x;
		pt_train.y = img1.rows + keypoint2[iter->trainIdx].pt.y;

		cv::Scalar color = Scalar(range(seed_b), range(seed_g), range(seed_r));

		cv::circle(img_res, pt_query, 5, color, 2);
		cv::circle(img_res, pt_train, 5, color, 2);

		cv::line(img_res, pt_query, pt_train, color, 1, 8, 0);
	}

	return img_res;
}


/*
	*************************************************************************************
	Main Function of Feature Extractor
	*************************************************************************************
*/
void FeatureExtractor::matchImgSeq()
{
	cv::Mat imgA, imgB;
	int iter;

	vector<int> idxA, idxB;

	vector<DMatch> curMatches;
	vector<int> curGPID;

	vector<DMatch> prevMatches;
	vector<int> prevGPID;

	vector<KeyPoint> keypointsA,keypointsB;
	cv::Mat desc_vecA, desc_vecB;

	vector<vector<DMatch>> NNmatches1, NNmatches2;
	int NNremoved1, NNremoved2;

	vector<DMatch> symMatches;

	Timer *myTimer = new Timer();

	vector<vector<DMatch>> allMatches1to2;
	vector<vector<uchar>> allCorectMatchesMask;
	vector<Point2f> recallPrecisionCurve;

	vector<Vec3f> elinesA, elinesB;

	string iter_str, iter_str_next;

	// DEBUG FILE FOR FUNDAMENTAL MATRICES
	//ofstream file_FM;
	//file_FM.open("fundamental.txt", ios::out);
	//if(!file_FM)
	//{
	//	cout << "ERROR: Cannot open the fundamental file" <<  endl;
	//}


	for(iter = 0; iter < FeatureExtractor::in_images.size()-1; iter++)
	{
		
		cout << iter << endl;

		stringstream ss;
		ss << iter;
		iter_str = ss.str();
		ss.clear();

		ss << iter+1;
		iter_str_next = ss.str();
		ss.clear();


		FeatureExtractor::IOhelper.os_log << 
			FeatureExtractor::in_images[iter] << "\t" << // Query Image
			FeatureExtractor::in_images[iter+1] << "\t"; // Train Image

		if(iter == 0)
		{
			imgA = imread
				(FeatureExtractor::IOhelper.path_imgFolder + FeatureExtractor::in_images[iter], 
				CV_LOAD_IMAGE_GRAYSCALE);
			imgB = imread(FeatureExtractor::IOhelper.path_imgFolder + FeatureExtractor::in_images[iter+1],
				CV_LOAD_IMAGE_GRAYSCALE);

			myTimer->start();

			cv::Mat mask = cv::Mat::zeros(imgA.size(), CV_8UC1);
			cv::Mat roi1(mask, cv::Rect(0, 0, imgA.cols/3, imgA.rows));
			roi1 = cv::Scalar(255, 255, 255);

			cv::Mat roi2(mask, cv::Rect(imgA.cols/3*2, 0, imgA.cols/3, imgA.rows));
			roi2 = cv::Scalar(255, 255, 255);


			detector->detect(imgA, keypointsA/*, mask*/);
			detector->detect(imgB, keypointsB/*, mask*/);


			FeatureExtractor::performANMS(keypointsA, 2000, idxA, imgA.size());
			FeatureExtractor::performANMS(keypointsB, 2000, idxB, imgB.size());


			FeatureExtractor::IOhelper.os_log <<
				myTimer->finish() << "\t"; // Detection Time

			myTimer->start();
			descriptor->compute(imgA, keypointsA, desc_vecA);
			descriptor->compute(imgB, keypointsB, desc_vecB);
			
			FeatureExtractor::IOhelper.os_log <<
				myTimer->finish() << "\t"; // Description Time


			FeatureExtractor::out_kps.push_back(keypointsA);
			FeatureExtractor::out_kps.push_back(keypointsB);

			FeatureExtractor::IOhelper.os_log <<
				keypointsA.size() << "\t" << // # of query keypoints
				keypointsB.size() << "\t"; // # of train keypoints

			cv::Mat img_kpA, img_kpB;

			drawKeypoints(imgA, keypointsA, img_kpA, Scalar(255, 0, 0), 4);
			drawKeypoints(imgB, keypointsB, img_kpB, Scalar(255, 0, 0), 4);

			cv::imwrite(FeatureExtractor::IOhelper.path_keypoints + "IMG_kp_" + iter_str + ".jpg", img_kpA);
			cv::imwrite(FeatureExtractor::IOhelper.path_keypoints + "IMG_kp_" + iter_str_next + ".jpg", img_kpB);
			
		}
		else
		{
			imgA = *(&imgB);
			keypointsA = *(&keypointsB);
			desc_vecA = *(&desc_vecB);

			imgB.release();
			keypointsB.clear();
			desc_vecB.release();

			imgB = imread(
				FeatureExtractor::IOhelper.path_imgFolder + FeatureExtractor::in_images[iter+1],
				CV_LOAD_IMAGE_GRAYSCALE);

			cv::Mat mask = cv::Mat::zeros(imgA.size(), CV_8UC1);
			cv::Mat roi1(mask, cv::Rect(0, 0, imgA.cols/3, imgA.rows));
			roi1 = cv::Scalar(255, 255, 255);

			cv::Mat roi2(mask, cv::Rect(imgA.cols/3*2, 0, imgA.cols/3, imgA.rows));
			roi2 = cv::Scalar(255, 255, 255);

			myTimer->start();
			detector->detect(imgB, keypointsB/*, mask*/);

			FeatureExtractor::performANMS(keypointsB, 2000, idxB, imgB.size());
			
			FeatureExtractor::IOhelper.os_log <<
				myTimer->finish() << "\t"; // Detection Time

			myTimer->start();
			descriptor->compute(imgB, keypointsB, desc_vecB);
			
			FeatureExtractor::IOhelper.os_log <<
				myTimer->finish() << "\t"; // Description Time

			FeatureExtractor::out_kps.push_back(keypointsB);

			FeatureExtractor::IOhelper.os_log <<
				keypointsA.size() << "\t" << // # of query keypoints
				keypointsB.size() << "\t"; // # of train keypoints

			cv::Mat img_kpB;
			drawKeypoints(imgB, keypointsB, img_kpB, Scalar(255, 0, 0), 4);
			cv::imwrite(FeatureExtractor::IOhelper.path_keypoints + "IMG_kp_" + iter_str_next + ".jpg", img_kpB);

		}

		myTimer->start();
		matcher->knnMatch(desc_vecA, desc_vecB, NNmatches1, 2);
		
		FeatureExtractor::IOhelper.os_log <<
			myTimer->finish() << "\t" << // Time for KNN1
			NNmatches1.size()  << "\t"; // # of KNN1 matches

		myTimer->start();
		matcher->knnMatch(desc_vecB, desc_vecA, NNmatches2, 2);
		
		FeatureExtractor::IOhelper.os_log <<
			myTimer->finish() << "\t" << // Time for KNN2
			NNmatches2.size()  << "\t"; // # of KNN2 matches

		myTimer->start();
		NNremoved1 = checkNNratio(NNmatches1);

		FeatureExtractor::IOhelper.os_log <<
			myTimer->finish() << "\t" << // Time for NN ratio test 1
			NNremoved1  << "\t"; // # of removed KNN1 matches

		//vector<vector<DMatch>>::iterator matchIter;


		//for(matchIter = NNmatches1.begin();
		//	matchIter != NNmatches1.end();
		//	++matchIter)
		//{
		//	if((matchIter->size()) == 2)
		//		symMatches.push_back(DMatch((*matchIter)[0].queryIdx, (*matchIter)[0].trainIdx, (*matchIter)[0].distance));
		//}



		myTimer->start();
		NNremoved2 = checkNNratio(NNmatches2);

		FeatureExtractor::IOhelper.os_log <<
			myTimer->finish() << "\t" << // Time for NN ratio test 2
			NNremoved1  << "\t"; // # of removed KNN2 matches

		myTimer->start();
		checkSymmetricity(NNmatches1, NNmatches2, symMatches);

		FeatureExtractor::IOhelper.os_log <<
			myTimer->finish() << "\t" << // Time for symmetricity check
			symMatches.size()  << "\t"; // # of symmetric matches

		/* For Precision-Recall computations */
		double TP, // True positive (# of symMatches)
			FP, // False positive (NNmatches1 - symMatches)
			FN, // False negetive (NNmatches2 - symMatches)
			TN; // True negative (# of NNremoved1)

		double precision =0.0,
			recall=0.0;

		TP = symMatches.size();
		FP = NNmatches1.size() - NNremoved1 - symMatches.size();
		FN = NNmatches2.size() - NNremoved2 - symMatches.size();
		TN = NNremoved1;

		precision =  TP / (TP + FP);
		recall = TP / (TP + FN);

		FeatureExtractor::IOhelper.os_log <<
			TP << "\t" <<
			FP << "\t" <<
			FN << "\t" <<
			TN << "\t" <<
			precision << "\t" <<
			recall << "\t";

		//cv::Mat matRes1;
		//drawMatches(imgA, keypointsA, imgB, keypointsB, symMatches, matRes1, Scalar::all(-1), Scalar::all(-1), vector<char>(), 2);
		//imwrite("matchResult_sym.jpg", matRes1);


		myTimer->start();
		
		checkEpipolar(symMatches, 
			keypointsA, 
			keypointsB, 
			FeatureExtractor::in_eop.at(iter), 
			FeatureExtractor::in_eop.at(iter+1), 
			FeatureExtractor::in_iop, 
			curMatches,
			elinesA,
			elinesB);

		FeatureExtractor::IOhelper.os_log <<
			myTimer->finish() << "\t" ; // Time for Epipolar Check

	    vector<DMatch> finalMatches;

		/*if(curMatches.size() > FeatureExtractor::max_matches)
		{
			FeatureExtractor::tooMany(curMatches, finalMatches);
		}
		else
		{
			finalMatches = curMatches;
		}*/

		//finalMatches = curMatches;

		finalMatches = curMatches;

		FeatureExtractor::IOhelper.os_log <<
			finalMatches.size() << endl ; // # of final matches


		cv::Mat img_opticflow;
		img_opticflow = FeatureExtractor::drawOpticFlow(imgB, finalMatches, keypointsA, keypointsB);
		cv::imwrite(FeatureExtractor::IOhelper.path_optical_flow + "IMG_opt_" + iter_str + ".jpg", img_opticflow);


		cv::Mat img_epipolar;
		img_epipolar = FeatureExtractor::drawEpiLine(imgA, imgB, finalMatches, keypointsA, keypointsB, elinesA, elinesB);
		cv::imwrite(FeatureExtractor::IOhelper.path_epipolar + "IMG_epi_" + iter_str + ".jpg", img_epipolar);

		cv::Mat img_matches;
		img_matches = FeatureExtractor::drawMatchingPair(imgA, imgB, finalMatches, keypointsA, keypointsB);
		cv::imwrite(FeatureExtractor::IOhelper.path_matching_pair + "IMG_match_" + iter_str + ".jpg", img_matches);


		float repeatability;
		int corrCount;
		computeRepeatability(imgA, imgB, symMatches, keypointsA, keypointsB, repeatability, corrCount);


		FeatureExtractor::IOhelper.os_log <<
		repeatability << "\t" << // repeatability to evaluate detector
		corrCount << endl; // # of correpending pairs(point to point) for repeatability


		setImgID(finalMatches, iter+1);

		FeatureExtractor::out_matches.push_back(finalMatches);


		if(finalMatches.size() < 5)
		{
			cout<< "No matches!!!" <<endl;
			// exit(0);
		}
		
		if(iter == 0)
		{
			setGPID(FeatureExtractor::out_matches[iter], curGPID, keypointsA, keypointsB);
		}
		else
		{
			setGPID(FeatureExtractor::out_matches[iter-1], FeatureExtractor::out_matches[iter], 
				FeatureExtractor::out_gpid[iter-1], curGPID, keypointsA, keypointsB);
		}

		FeatureExtractor::out_gpid.push_back(curGPID);

		NNmatches1.clear();
		NNmatches2.clear();
		symMatches.clear();
		curMatches.clear();
		finalMatches.clear();
		curGPID.clear();

	}

	FeatureExtractor::IOhelper.os_tp.close();
	FeatureExtractor::IOhelper.os_log.close();
}
