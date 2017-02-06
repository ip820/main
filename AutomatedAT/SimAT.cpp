/********************************************************
 * Project:			Automated AT
 * Last updated:	17 Jaunary 2011
 * Developer:		Supannee Tanathong
 ********************************************************/

#include "SimAT.h"
#include "Global.h"
#include <iostream>
#include <fstream>		// To utilize ifstream for dealing with text file input.
#include <strstream>
#include "stdio.h"

/********************************************************************
 * Description:	Construction
 ********************************************************************/
CSimAT::CSimAT(const string &_CFG_File)
{
	// Store the configuration file into the member variable.
	m_CFG_File = _CFG_File;

	// Initialize the epoch ID.
	m_EPOCH_ID = 0;

	// Initialize the number of images in the sequence.
	m_nImages = 0;

	// Initialize the sum of image points.
	m_nIP_raw = 0;

	// Initialize the number of points for the first image ID.
	// Image ID: 0 (not existence) always has 0 image point.
	m_vec_n_IP.push_back(0);	

}

/********************************************************************
 * Description:	Main function to perform Simultaneous AT
 ********************************************************************/
void CSimAT::RunSimAT()
{
	// Initialize all neccessary parameters.
	Initialization();

	// Perform Simultaneous AT estimation.
	SimATEstimation();
}

/********************************************************************
 * Description:	Call necessary functions for initialization prior to
 *				perform the simultaneous AT.
 ********************************************************************/
void CSimAT::Initialization()
{
	// Read the configuration file.
	ReadConfig();

	// Read the IO file.
	ReadIOFile();

	// Initialize the output tie point file.
	if (true == m_bOutputTP)
		InitializeTPFile();

	// Read the EO file.
	//ReadEOFile();

	// Read the IP/Tiepoints file.
	//ReadIPFile();
	//ReadIPFile2();

	// Initialize member variables prior to processing.
	//InitializeMembers();
}

/********************************************************************
 * Description:	Read the configuration file
 ********************************************************************/
void CSimAT::ReadConfig()
{
	// Open the configuration file to read.
	ifstream fileInput(m_CFG_File.c_str());

	if (!fileInput)
	{
		cout << "Error! Failed to open the configuration file." << endl;
		exit(0);
	}
	
	const int BUFFER_SIZE = 200;
	char buffer[BUFFER_SIZE+1] = {0};

	// Read the flag whether to run this process after TP extraction.
	GetLine(fileInput, buffer, BUFFER_SIZE);
	sscanf(buffer, "%d", &m_bRun);

	// Read the flag whether to out put tie points into a file.
	GetLine(fileInput, buffer, BUFFER_SIZE);
	sscanf(buffer, "%d", &m_bOutputTP);

	// Read the IO file name.
	GetLine(fileInput, buffer, BUFFER_SIZE);
	istrstream(buffer) >> m_IO_File;

	// Read the rotational matrix order.
	GetLine(fileInput, buffer, BUFFER_SIZE);
	sscanf(buffer, "%d", &ROT_ORDER);

	// Read the rotational matrix type.
	GetLine(fileInput, buffer, BUFFER_SIZE);
	sscanf(buffer, "%d", &ROT_TYPE);

	// Read the max. number of iterations for performing AT.
	GetLine(fileInput, buffer, BUFFER_SIZE);
	sscanf(buffer, "%d", &m_max_iter);

	// Read the initial number of images for starting seq. AT sim.
	GetLine(fileInput, buffer, BUFFER_SIZE);
	sscanf(buffer, "%d", &m_ninit_sim_img);

	// Read the max. number of images for seq. AT computation.
	GetLine(fileInput, buffer, BUFFER_SIZE);
	sscanf(buffer, "%d", &m_nmax_sim_img);
	
	// Read the 'Delta' stop criteria.
	GetLine(fileInput, buffer, BUFFER_SIZE);
	sscanf(buffer, "%lg", &m_dDelta);

	// Read the standard deviation of the image point errors (pixels).
	GetLine(fileInput, buffer, BUFFER_SIZE);
	sscanf(buffer, "%lf", &m_std_IP);

	// Read the standard deviation of the GPS errors (cm).
	GetLine(fileInput, buffer, BUFFER_SIZE);
	sscanf(buffer, "%lf", &m_std_GPS);

	// Read the standard deviation of the INS errors (degrees).
	GetLine(fileInput, buffer, BUFFER_SIZE);
	sscanf(buffer, "%lf", &m_std_INS);

	// Close the file after finish reading.
	fileInput.close();
}

/********************************************************************
 * Description:	Read the IO file
 ********************************************************************/
void CSimAT::ReadIOFile()
{
	// Open the IO file to read.
	ifstream fIO(m_IO_File.c_str());

	if (!fIO)
	{
		cout << "Error! Failed to open the IO file." << endl;
		exit(0);
	}
	
	const int BUFFER_SIZE = 200;
	char buffer[BUFFER_SIZE+1] = {0};

	// Read the principle point.
	GetLine(fIO, buffer, BUFFER_SIZE);
	sscanf(buffer, "%lf,%lf,%lf", &m_IO.dPPX, &m_IO.dPPY, &m_IO.dF);

	// Read the radial distortion.
	GetLine(fIO, buffer, BUFFER_SIZE);
	sscanf(buffer, "%lg,%lg,%lg", &m_RDC.dK1, &m_RDC.dK2, &m_RDC.dK3);
	
	// Read the resolution / image dimension
	GetLine(fIO, buffer, BUFFER_SIZE);
	sscanf(buffer, "%d,%d", &m_IS.iWidth, &m_IS.iHeight);

	// Read the pixel size (mm)
	GetLine(fIO, buffer, BUFFER_SIZE);
	sscanf(buffer, "%lf,%lf", &m_PS.dX, &m_PS.dY);

	// Close the file after complete reading.
	fIO.close();
}

/********************************************************************
 * Description:	Read the EO file
 *
 * NOTE:		This function is substituted by the GetNewImage()
 *				function which will store the EO parameters of 
 *				each image into the m_EO_m buffer.
 ********************************************************************/
void CSimAT::ReadEOFile()
{
	EO		eo;

	// If the file exist, read the EO data from the file.
	FILE * fEO = fopen(m_EO_File.c_str(), "r");

	if (!fEO)
	{
		cout << "Error! Failed to open the EO file." << endl;
		exit(0);
	}

	const int BUFFER_SIZE = 200;
	char buffer[BUFFER_SIZE+1] = {0};

	while(fgets(buffer, BUFFER_SIZE, fEO) != NULL)
	{
		// Skip the comment
		if (buffer[0] == '%')
			continue;

		sscanf(buffer, "%d\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n", &eo.iImg, &eo.dXc, &eo.dYc, &eo.dZc, &eo.dOmega, &eo.dPhi, &eo.dKappa);

		m_EO_m.push_back(eo);
	}

	fclose(fEO);
}

/********************************************************************
 * Description:	Read the IP/Tiepoints file
 ********************************************************************/
void CSimAT::ReadIPFile()
{
	IP	ip;
	int nImg, numIP, imgID, nIP;

	// If the file exist, read the tiepoints from the file.
	FILE * fIP = fopen(m_IP_File.c_str(), "r");

	if (!fIP)
	{
		cout << "Error! Failed to open the IP file." << endl;
		exit(0);
	}

	// Read the number of images in the sequence.
	fscanf(fIP, "No. Images : %d\n", &m_nImages);

	if (m_nImages <= 0)
	{
		cout << "Error! Number of image to process is invalid." << endl;
		exit(0);
	}

	// Image ID: 0 (not existence) always has 0 image point.
	m_vec_n_IP.push_back(0);	

	// Initialize the sum of image points.
	m_nIP_raw = 0;

	// Iterate through each image in the sequence.
	for (nImg = 1; nImg <= m_nImages; nImg++)
	{
		// Read the Image ID and number of image points in that image.
		fscanf(fIP, "Image ID : %d\n", &imgID);
		fscanf(fIP, "No. IP : %d\n", &numIP);

		// Add the number of image points into the vector.
		m_vec_n_IP.push_back(numIP);

		// Increment the sum of image points.
		m_nIP_raw += numIP;

		// Store each point into the vector.
		for (nIP = 0; nIP < numIP; nIP++)
		{
			// Obain the 'ImageID', 'ObjectID', 'X-coord', 'Y-coord' in order.
			fscanf(fIP, "%d\t%d\t%lf\t%lf\n", &ip.iImg, &ip.iObj, &ip.dX, &ip.dY);

			// Put into the list of points. 
			m_IP_i.push_back(ip);
		}
	}
	
	// Close the file after finish reading.
	fclose(fIP);
}

/********************************************************************
 * Description:	Read the IP/Tiepoints file generated from Simulator
 ********************************************************************/
void CSimAT::ReadIPFile2()
{
	IP	ip;
	int nImg, numIP, imgID, nIP;

	// If the file exist, read the tiepoints from the file.
	FILE * fIP = fopen(m_IP_File.c_str(), "r");

	if (!fIP)
	{
		cout << "Error! Failed to open the IP file." << endl;
		exit(0);
	}

	// Read the number of images in the sequence.
	fscanf(fIP, "No. Images : %d\n", &m_nImages);

	if (m_nImages <= 0)
	{
		cout << "Error! Number of image to process is invalid." << endl;
		exit(0);
	}

	// Initialize the number of image points in the m_vec_n_IP vector.
	for (nImg = 0; nImg <= m_nImages; nImg++)
		m_vec_n_IP.push_back(0);	

	// Initialize the sum of image points.
	m_nIP_raw = 0;

	// Iterate through each image in the sequence starting from Image#1.
	for (nImg = 1; nImg <= m_nImages; nImg++)
	{
		// Read the Image ID and number of image points in that image.
		fscanf(fIP, "Image ID : %d\n", &imgID);
		fscanf(fIP, "No. IP : %d\n", &numIP);

		// Increment the sum of image points.
		m_nIP_raw += numIP;

		// Store each point into the vector.
		for (nIP = 0; nIP < numIP; nIP++)
		{
			// Obain the 'ImageID', 'ObjectID', 'X-coord', 'Y-coord' in order.
			fscanf(fIP, "%d\t%d\t%lf\t%lf\n", &ip.iImg, &ip.iObj, &ip.dX, &ip.dY);

			// Put into the list of points. 
			m_IP_i.push_back(ip);

			// Increment the number of image points in the vector.
			m_vec_n_IP[ip.iImg] = m_vec_n_IP[ip.iImg] + 1;
		}
	}
	
	// Close the file after finish reading.
	fclose(fIP);
}

/********************************************************************
 * Description:	Initialize EO.
 ********************************************************************/
void CSimAT::InitializeEO()
{
	// Compute the 3D rotational matrix.
	m_RotMatrix.Compute3DRotMatrix(m_EO_m, m_nImages, ROT_ORDER, ROT_TYPE);
	
	// Compute the derivative 3D rotational matrix.
	m_RotMatrix.ComputeDerivative3DRotMatrix(m_EO_m, m_nImages, ROT_ORDER, ROT_TYPE);
}

/********************************************************************
 * Description:	Initialize ground points.
 ********************************************************************/
void CSimAT::InitializeGP()
{
/*
	// Make the list of distinct ground point ID
	m_GP.MakeGPList(m_IP_i, m_nImages);

	// Debug out the list of distinct GP.
	m_GP.DebugDistinctGP();

	// Initial approximation for GPs
	m_GP.GPInitApproximation(m_IP_f, m_IO, m_EO_m, m_nImages, m_RotMatrix, m_max_iter, m_dDelta, true);

	// Debug out the inital approximated GPs.
	m_GP.DebugInitGP(false);
*/
}

/********************************************************************
 * Description:	Initialize member variables prior to processing.
 ********************************************************************/
void CSimAT::InitializeMembers()
{
	// Convert the units of members to appropriate scale.
	// UnitConversion();

	// Initialize the photo coordinate object.
	// m_PhotoCoord.Initialization(m_IO, m_IS, m_PS);

	// Convert the units of std. deviation of the error.
    // StdUnitConversion();

	// Convert the image points from 'Pixel Coord' to 'Photo Coord'
	// m_IP_f = m_PhotoCoord.ConvertPixel2PhotoCoords(m_IP_i);

	// Initialize EO parameters.
	//InitializeEO();

}


/********************************************************************
 * Description:	Convert the units of input parameters to appropriate scale.
 ********************************************************************/
void CSimAT::UnitConversion()
{
	// Principle point - convert from mm to m
	m_IO.dPPX	=	m_IO.dPPX/1000.0;
	m_IO.dPPY	=	m_IO.dPPY/1000.0;
	m_IO.dF		=	m_IO.dF/1000.0;

	// Pixel size - convert from mm to m
	m_PS.dX		=	m_PS.dX/1000.0;
	m_PS.dY		=	m_PS.dY/1000.0;
}


/********************************************************************
 * Description:	Convert the units of std. deviation of error to appropriate scale.
 ********************************************************************/
void CSimAT::StdUnitConversion()
{
	// Std. of IP error - convert the unit from pixels to m
	PS ps = m_PhotoCoord.ConvertNumPixel2Metre(m_std_IP);
	m_std_IP = (ps.dX + ps.dY)/2;		// Average value

	// Std. of GPS error - convert the unit from cm to m
	m_std_GPS = m_std_GPS/100.0;

	// Std. of INS error - convert the unit from degree to radian
	m_std_INS = deg2rad(m_std_INS);
}

/********************************************************************
 * Description:	Wrapper function to get each line from the input file.
 ********************************************************************/
void CSimAT::GetLine(istream &in, char *buf, const int bufSize) const
{
	while(in.good())
	{
		in.getline(buf, bufSize);
		if(buf[0]!='%') break;
	}
}

/********************************************************************
 * Description:	This function will be called whenever a new image
 *				is captured. 
 ********************************************************************/
void CSimAT::GetNewImage(string image_name, EO eo)
{
	// Store the EO for future use while discard the unused image file name.
	m_EO_m.push_back(eo);

	// Increment the number of images in the sequence.
	m_nImages++;

	// Get the tie points from the KLT object.
	if (m_EO_m.size() >= 2)
	{
		GetTiePoints();

		// At this point, the TP is in the form of photo-coordinates.
	}

	// Perform the Simultaneous AT if the number of images is greater
	// than the minimum number of the required images to perform.
	if (true == m_bRun)
	{
		if (m_EO_m.size() >= m_ninit_sim_img)
			SimATEstimation();
	}
}

/********************************************************************
 * Description:	Perform Simultaneous AT estimation.
 ********************************************************************/
void CSimAT::SimATEstimation()
{
	// Set the parameters to the Sim AT object's member variables.
	m_SeqSimAT.Set(	m_nImages, 
					m_max_iter, 
					m_ninit_sim_img, 
					m_nmax_sim_img,
					m_dDelta,
					m_std_IP,
					m_std_GPS,
					m_std_INS,
					ROT_ORDER,
					ROT_TYPE);

	// Get the latest image ID.
	int latest_image_id = m_EO_m[m_nImages-1].iImg;

	// Perform the sequential AT simultaneous estimation.
	m_SeqSimAT.SeqSimEstimation(m_vec_n_IP,
								m_IP_f, 
								m_vec_n_IP, 
								m_IO, 
								m_EO_m,
								latest_image_id);

	// The results from Sim AT are its member variable.

}

/********************************************************************
 * Description:	Print out the result of the Simultaneous AT estimation.
 ********************************************************************/
void CSimAT::PrintOutResults()
{
	if (m_bRun == true)
	{
		// Debug out the results into a file named "Summary.txt".
		m_SeqSimAT.DebugSimATResults();
	}
}

/********************************************************************
 * Description:	Store the pointer to the singleton CKLTTracker object.
 ********************************************************************/
void CSimAT::RegisterKLT(CKLTTracker * pKLT)
{
	m_pKLT = pKLT;
}

/********************************************************************
 * Description:	Get the set of tie-points in the EPOCH format.
 *
 *				Epoch ID#0: m_EO_m[0].iImg, m_EO_m[1].iImg
 *				Epoch ID#1:	m_EO_m[1].iImg, m_EO_m[2].iImg
 *				Epoch ID#2:	m_EO_m[2].iImg, m_EO_m[3].iImg
 *				....
 *
 ********************************************************************/
void CSimAT::GetTiePoints()
{
	int npoints, npoints1, npoints2;	// The number of tie points.
	int	IMG_ID;							// Image ID
	int	last_element_idx;				// Last element index.
	int last_stored_GP_ID;				// Latest stored GP ID
	int	idxIMG;							// Index of Image ID
	
	// Start the first epoch by requesting the tie points for the first two images.
	if (0 == m_EPOCH_ID)
	{
		// Get the tie points from the first image in the EO vector : m_EO_m[0].iImg
		npoints1 = m_pKLT->GetTiePoints(m_EO_m[0].iImg, &m_IP_f);

		// Store the number of tie points into the vector of tie-point number.
		m_vec_n_IP.push_back(npoints1);

		// Get the tie points from the second image in the EO vector : m_EO_m[1].iImg
		npoints2 = m_pKLT->GetTiePoints(m_EO_m[1].iImg, &m_IP_f);

		// Store the number of tie points into the vector of tie-point number.
		m_vec_n_IP.push_back(npoints2);

		// Calculate the total number of tie points for EPOCH ID#0
		npoints = npoints1 + npoints2;

	}
	else
	{
		// Get the first set of EPOCH for the 'first' image.

			// Get the image ID by computing from the epoch ID.
			IMG_ID = m_EO_m[m_EPOCH_ID].iImg;

			// Get the latest stored GP ID.
			last_element_idx = m_IP_f.size() - 1;
			last_stored_GP_ID = m_IP_f[last_element_idx].iObj;

			// Get the tie points.
			npoints1 = m_pKLT->GetTiePoints(IMG_ID, &m_IP_f, last_stored_GP_ID);

			// Update the number of tie points into the vector of tie-point number.
			idxIMG = m_vec_n_IP.size() - 1;
			m_vec_n_IP[idxIMG] = m_vec_n_IP[idxIMG] + npoints1;

		// Get the second set of EPOCH for the 'second' image.

			// Get the image ID by computing from the epoch ID.
			IMG_ID = m_EO_m[m_EPOCH_ID + 1].iImg;

			// Get the tie points.
			npoints2 = m_pKLT->GetTiePoints(IMG_ID, &m_IP_f);

			// Store the number of tie points into the vector of tie-point number.
			m_vec_n_IP.push_back(npoints2);
	}

	// Calculate the total number of tie points for EPOCH.
	npoints = npoints1 + npoints2;

	// Record the number of tie points for each EPOCH
	m_vec_n_EPOCH.push_back(npoints);	

	// Increment the sum of image points.
	m_nIP_raw += npoints;

	// Increment the epoch ID.
	m_EPOCH_ID++;

	// To print out the immediate TP.
	if (true == m_bOutputTP)
	{
		FILE * fEpoch = fopen("IP_s.txt", "a+");
		// fprintf(fEpoch, "Epoch : %d\n", m_EPOCH_ID);
		fprintf(fEpoch, "Epoch : %d\n", m_EPOCH_ID+1); // To compliance with MATLAB
		fprintf(fEpoch, "No. IP : %d\n", npoints);
		int idxIP = m_nIP_raw - npoints;
		for (int i = idxIP; i < m_nIP_raw; i++)
		{
			fprintf(fEpoch, "%d\t%d\t%lf\t%lf\n", 
				m_IP_f[i].iImg, m_IP_f[i].iObj, m_IP_f[i].dX, m_IP_f[i].dY);
		}
		fclose(fEpoch);
	}
}

/********************************************************************
 * Description:	Initialize the output tie point file.
 ********************************************************************/
void CSimAT::InitializeTPFile()
{
	FILE * fEpoch = fopen("IP_s.txt", "w");
	fclose(fEpoch);
}
