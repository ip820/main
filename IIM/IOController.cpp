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

using namespace std;
using namespace cv;
using namespace arma;

IOController::IOController(string& config)
{
	// Open the configuration file stream
	IOController::is_config.open(config, ios::in);
	if(!IOController::is_config)
	{
		cout << "ERROR : Cannot open the configuration file" << endl;
	}

	IOController::readConfig();

	cout << "SUCEESS : IOController is ready" << endl;

}

void IOController::readConfig()
{
	vector<string> buff;
	string tmp;

	// Read in the configuration file 
	while(!IOController::is_config.eof())
	{
		getline(IOController::is_config, tmp);
		if(tmp.empty())
		{
			break;
		}

		buff.push_back(tmp);
		tmp.clear();
	}

	if(buff.size() == 9)
	{
		IOController::path_img = buff.at(0);
		IOController::path_eo = buff.at(1);
		IOController::path_io = buff.at(2);
		IOController::path_tp = buff.at(3);
		IOController::path_log = buff.at(4);
		IOController::path_keypoints = buff.at(5);
		IOController::path_epipolar = buff.at(6);
		IOController::path_optical_flow = buff.at(7);
		IOController::path_matching_pair = buff.at(8);
	}

	// Open the image list file stream
	IOController::is_img.open(IOController::path_img, ios::in);
	if(!IOController::is_img)
	{
		cout << "ERROR : Cannot open the image list file" << endl;
	}

	//Open the EO list file steam
	IOController::is_eo.open(IOController::path_eo, ios::in);
	if(!IOController::is_eo)
	{
		cout << "ERROR : Cannot open the EO list file" << endl;
	}

	// Open the IO file stream
	IOController::is_io.open(IOController::path_io, ios::in);
	if(!IOController::is_io)
	{
		cout << "ERROR : Cannot open the IO list file" << endl;
	}

	// Open the TP file stream
	IOController::os_tp.open(IOController::path_tp, ios::out);
	if(!IOController::os_tp)
	{
		cout << "ERROR : Cannot open the TP file" << endl;
	}

	// Open the log file stream
	IOController::os_log.open(IOController::path_log, ios::out);
	if(!IOController::os_log)
	{
		cout << "ERROR : Cannot open the log file" << endl;
	}
}

void IOController::setImg(vector<string> &imgList)
{
	string buff;

	if(IOController::is_img)
	{
		// Read the path of input image folder
		getline(IOController::is_img, buff);
		if(buff.empty())
		{
			cout << "ERROR : Cannot read input image file folder" << endl;
		}
		else
		{
			IOController::path_imgFolder = buff;
			buff.clear();
		}

		// Read the input image file name
		while(!IOController::is_img.eof())
		{
			getline(IOController::is_img, buff);
			if(buff.empty())
			{
				cout << "ERROR : Cannot read image file name" << endl;
			}
			else
			{
				imgList.push_back(buff);
				buff.clear();
			}
		}
	}
}

void IOController::setEO(vector<EO> &eoList)
{
	size_t ptr1, ptr2;
	string buff;
	cv::Mat tmp(1, 6, CV_64F);

	int id_eo = 0;

	if(IOController::is_eo)
	{
		while(!IOController::is_eo.eof())
		{
			getline(IOController::is_eo, buff);

			if(buff.empty())
			{
				cout << "ERROR : Cannot read EO list" << endl;
			}
			else
			{
				ptr1 = 0;

				for(int i = 0; i < 6; i++)
				{
					ptr2 = buff.find("\t", ptr1);
					tmp.at<double>(0, i) = 
						stod(buff.substr(ptr1, ptr2-ptr1));
					ptr1 = ptr2 + 1;
				}

				eoList.push_back(
					EO(id_eo,
					tmp.at<double> (0, 0),
					tmp.at<double> (0, 1),
					tmp.at<double> (0, 2),
					tmp.at<double> (0, 3),
					tmp.at<double> (0, 4),
					tmp.at<double> (0, 5)));

				buff.clear();
				id_eo++;
			}
		}
	}
}

void IOController::setIO(IO &io)
{
	size_t ptr1, ptr2;
	string buff;
	cv::Mat tmp(1, 12, CV_64F);

	if(IOController::is_io)
	{
		getline(IOController::is_io, buff);

		if(buff.empty())
		{
			cout << "ERROR : Cannot read IO file" << endl;
		}
		else
		{
			ptr1 = 0;

			for(int i = 0; i < 12; i++)
			{
				ptr2 = buff.find("\t", ptr1);
				tmp.at<double>(0, i) = 
					stod(buff.substr(ptr1, ptr2-ptr1));
				ptr1 = ptr2 + 1;

			}

			// DEBUG 
			// cout << "IO" << endl << tmp << endl;

			io = IO(tmp.at<double>(0, 0),
				tmp.at<double>(0, 1),
				tmp.at<double>(0, 2),
				tmp.at<double>(0, 3),
				tmp.at<double>(0, 4),
				tmp.at<double>(0, 5),
				tmp.at<double>(0, 6),
				tmp.at<double>(0, 7),
				tmp.at<double>(0, 8),
				tmp.at<double>(0, 9),
				tmp.at<double>(0, 10),
				tmp.at<double>(0, 11));
		}
	}
}
