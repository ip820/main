/********************************************************
 * Project:			Automated AT
 * Last updated:	26 September 2011
 * Developer:		Supannee Tanathong
 ********************************************************/

#include "KLTTracker.h"
#include "Global.h"
#include <iostream>
#include <fstream>		// To utilize ifstream for dealing with text file input.
#include <strstream>
#include "stdio.h"
#include <CRTDBG.H>		// To use _ASSERT

/********************************************************************
 * Description:	Construction
 ********************************************************************/
CKLTTracker::CKLTTracker(const string &_CFG_File)
{
	// Store the configuration file.
	m_CFG_File = _CFG_File; 

	// Initialize the GP ID.
	RUNNING_POINT_ID = 1;

	// Initialize the variables.
	imgA = NULL;	// Set image ptr to NULL
	imgB = NULL;	// Set image ptr to NULL
}

/********************************************************************
 * Description:	Destructor
 ********************************************************************/
CKLTTracker::~CKLTTracker()
{
	if (m_npoints_pattern_block)
		delete [] m_npoints_pattern_block;

	if (imgA != NULL)
		cvReleaseImage(&imgA);

	if (imgB != NULL)
		cvReleaseImage(&imgB);

}

/********************************************************************
 * Description:	Set member variables
 ********************************************************************/
void CKLTTracker::SetMembers(int n_init_num_images)
{

}

/********************************************************************
 * Description:	Wrapper function to get each line from the input file.
 ********************************************************************/
void CKLTTracker::GetLine(istream &in, char *buf, const int bufSize) const
{
	while(in.good())
	{
		in.getline(buf, bufSize);
		if(buf[0]!='%') break;
	}
}

/*******************************************************************
 * Description:	Read the KLT configuration file
 ********************************************************************/
void CKLTTracker::ReadConfig()
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

	// Read the IO file name.
	GetLine(fileInput, buffer, BUFFER_SIZE);
	istrstream(buffer) >> m_IO_File;

	// Read the scaling factor.
	GetLine(fileInput, buffer, BUFFER_SIZE);
	sscanf(buffer, "%d", &SCALE_FACTOR);

	// Read the rotational matrix order.
	GetLine(fileInput, buffer, BUFFER_SIZE);
	sscanf(buffer, "%d", &ROT_ORDER);

	// Read the rotational matrix type.
	GetLine(fileInput, buffer, BUFFER_SIZE);
	sscanf(buffer, "%d", &ROT_TYPE);

	// Read MAX_CORNERS: the number of points to track.
	GetLine(fileInput, buffer, BUFFER_SIZE);
	sscanf(buffer, "%d", &MAX_CORNERS);

	// Read MAX_BLOCK_TP: the maximum number of tie points for each block.
	GetLine(fileInput, buffer, BUFFER_SIZE);
	sscanf(buffer, "%d", &MAX_BLOCK_TP);

	// Read MINIMUM_EIGENVALUE: Minimum eigen value.
	GetLine(fileInput, buffer, BUFFER_SIZE);
	sscanf(buffer, "%lf", &MINIMUM_EIGENVALUE);

	// Read MIN_DISTANCE: Minimum distance between adjacent good features point.
	GetLine(fileInput, buffer, BUFFER_SIZE);
	sscanf(buffer, "%lf", &MIN_DISTANCE);
	
	// Read BLOCK_AUTOCORR: Window size of autocorrelation.
	GetLine(fileInput, buffer, BUFFER_SIZE);
	sscanf(buffer, "%d", &BLOCK_AUTOCORR);

	// Read WIN_SIZE_SUBPIXEL: Window size for subpixel.
	GetLine(fileInput, buffer, BUFFER_SIZE);
	sscanf(buffer, "%d", &WIN_SIZE_SUBPIXEL);

	// Read SUBPATTERN: Pattern of the overlapping P x P e.g. 5x5, 4x4, 3x3.
	GetLine(fileInput, buffer, BUFFER_SIZE);
	sscanf(buffer, "%d", &SUBPATTERN);

	// Read MAX_POINTS: Maximum points in sub region.
	GetLine(fileInput, buffer, BUFFER_SIZE);
	sscanf(buffer, "%d", &MAX_POINTS);

	// Read CRI_NUM_ITERATION: Stop Criteria - iteration.
	GetLine(fileInput, buffer, BUFFER_SIZE);
	sscanf(buffer, "%d", &CRI_NUM_ITERATION);

	// Read CRI_EPSILON_LIMIT: Stop Criteria - distance.
	GetLine(fileInput, buffer, BUFFER_SIZE);
	sscanf(buffer, "%lf", &CRI_EPSILON_LIMIT);

	// Read DEPTH_LEVELS: Pyramid depth level.
	GetLine(fileInput, buffer, BUFFER_SIZE);
	sscanf(buffer, "%d", &DEPTH_LEVELS);

	// Read FEATURE_ERROR: Tracking error.
	GetLine(fileInput, buffer, BUFFER_SIZE);
	sscanf(buffer, "%d", &FEATURE_ERROR);

	// Read if speeding up KLT by initial guessed TP will be performed.
	GetLine(fileInput, buffer, BUFFER_SIZE);
	sscanf(buffer, "%d", &CAL_INITIAL_GUESS);

	// Read if double check by CCC will be performed.
	GetLine(fileInput, buffer, BUFFER_SIZE);
	sscanf(buffer, "%d", &CCC_CHECK);

	// Read the CCC threshold for double check if CCC_CHECK flag is set.
	GetLine(fileInput, buffer, BUFFER_SIZE);
	sscanf(buffer, "%lf", &CCC_THRESHOLD);

	// Read ZA: Average terrain elevation.
	GetLine(fileInput, buffer, BUFFER_SIZE);
	sscanf(buffer, "%lf", &ZA);

	// Read the image directory.
	GetLine(fileInput, buffer, BUFFER_SIZE);
	istrstream(buffer) >> m_img_directory;

	// Read the flag whether to output the tie points.
	GetLine(fileInput, buffer, BUFFER_SIZE);
	sscanf(buffer, "%d", &m_bOutputTP);

	// Close the file after finish reading.
	fileInput.close();
}

/********************************************************************
 * Description:	Read the IO file
 * NOTE:		NOT USED
 ********************************************************************/
void CKLTTracker::ReadIOFile()
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
 * Description:	It performs all preliminary tasks prior to run the
 *				KLT tracking.
 ********************************************************************/
void CKLTTracker::Startup()
{
	// Read the KLT configuration file.
	ReadConfig();

	// Read the IO file.
	ReadIOFile();

	// Initialize the constant values required for KLT tracker.
	InitializeConstants();

	// Initialize the member variables.
	InitializeMembers();
}

/********************************************************************
 * Description:	Initialize the constant values required for KLT tracker.
 ********************************************************************/
void CKLTTracker::InitializeConstants()
{
	//START----- PERFORMANCE IMPROVEMENT SOLUTION -----

	// Adjust the parameters / image dimension
	// CAUTION: Need to handle the case when the image size is ODD number.
	m_scaledIS.iWidth = m_IS.iWidth/SCALE_FACTOR;	
	m_scaledIS.iHeight = m_IS.iHeight/SCALE_FACTOR;

	//END----- PERFORMANCE IMPROVEMENT SOLUTION -----

	// The size of 'vertical' subregion in the pattern block. 
	H_SPACE = m_scaledIS.iHeight/SUBPATTERN;

	// The size of 'horizontal' subregion in the pattern block. 
	W_SPACE = m_scaledIS.iWidth/SUBPATTERN;

	// Obtain the image center
	m_ICenter.dX = ((double)m_IS.iWidth - 1.0)/2;
	m_ICenter.dY = ((double)m_IS.iHeight - 1.0)/2;

	// Assign flag to re-read both image
	m_bReloadIMG = true;		
}

/********************************************************************
 * Description:	Initialize the member variables.
 ********************************************************************/
void CKLTTracker::InitializeMembers()
{
	// Initialize the pattern block to store the number of tie points in each block.
	m_npoints_pattern_block = new int[SUBPATTERN * SUBPATTERN];

	if (NULL == m_npoints_pattern_block)
	{
		printf("Error! Failed to allocate memory.");
		exit(0);
	}

	if (true == m_bOutputTP)
	{
		// Intialialize the TP file.
		FILE * fTP = fopen("TP.txt", "w");
		fclose(fTP);
	}
}

/********************************************************************
 * Description:	For an image, the whole area will be divided into sub-region
 *				following the defined pattern e.g. 3x3 or 4x4. 
 *				This function check if the passed-in coordinate falls
 *				into which region ID. 
 * Note:		The region ID is calculated as
 *				RegionID = iH * SUBPATTERN + iW
 ********************************************************************/
int CKLTTracker::FindSubRegionFromCoord(float y_coord, float x_coord)
{
	int iH, iW;

	// Calculate the row/column of the pattern block.
	iH = y_coord/H_SPACE;
	iW = x_coord/W_SPACE;

	iH = (iH >= SUBPATTERN)?SUBPATTERN-1:iH;
	iW = (iW >= SUBPATTERN)?SUBPATTERN-1:iW;

	// Calculate and return the subregion ID.
	return iH*SUBPATTERN + iW;
}

int CKLTTracker::FindSubRegionFromCoord(float y_coord, float x_coord, /*out*/int &iH, /*out*/int &iW)
{
	// Calculate the row/column of the pattern block.
	iH = y_coord/H_SPACE;
	iW = x_coord/W_SPACE;

	iH = (iH >= SUBPATTERN)?SUBPATTERN-1:iH;
	iW = (iW >= SUBPATTERN)?SUBPATTERN-1:iW;

	// Calculate and return the subregion ID.
	return iH*SUBPATTERN + iW;
}

/********************************************************************
 * Description:	The function returns the subregion ID in which the
 *				caller function passes in the row/column ID of  the
 *				pattern block.
 * Note:		The region ID is calculated as
 *				RegionID = iH * SUBPATTERN + iW
 ********************************************************************/
int CKLTTracker::FindSubRegionFromID(int iH, int iW)
{
	// Calculate and return the subregion ID.
	return iH*SUBPATTERN + iW;
}

/********************************************************************
 * Description:	The function returns the row/colmn ID of the passed-in
 *				region ID.
 * Note:		The region ID is calculated as
 *				RegionID = iH * SUBPATTERN + iW
 ********************************************************************/
void CKLTTracker::FindRowColFromRegionID(int regionID, /*out*/int &iH, /*out*/int &iW)
{
	iH = regionID/SUBPATTERN;
	iW = regionID%SUBPATTERN;
}

/********************************************************************
 * Description:	For an image, the whole area will be divided into sub-region
 *				following the defined pattern e.g. 3x3 or 4x4. 
 *				For the specified region ID, this function returns the 
 *				beginning x, beginning y, length x, and length y, in order. 
 ********************************************************************/
void CKLTTracker::DefineROI(int iH, int iW, 
							/*out*/ int &beginX, /*out*/ int &beginY,
							/*out*/ int &lengthX, /*out*/ int &lengthY)
{
	// Define the beginning coordinates.
	beginX = iW * W_SPACE;
	beginY = iH * H_SPACE;

	// Define the length of region in x-axis.
	if (iW < SUBPATTERN) 
		lengthX = W_SPACE;
	else
		lengthX = m_scaledIS.iWidth - beginX;

	// Define the length of region in y-axis.
	if (iH < SUBPATTERN)
		lengthY = H_SPACE;
	else
		lengthY = m_scaledIS.iHeight - beginY;

}

/********************************************************************
 * Description:	The function is to check if any block in the image
 *				pattern does not locate any tie point.
 *				If at least one block does not have tie points located,
 *				the function return false.
 ********************************************************************/
bool CKLTTracker::IsBlankRegionExisted()
{
	int i;

	for (i = 0; i < SUBPATTERN*SUBPATTERN; i++)
	{
		if (0 == m_npoints_pattern_block[i])
		{
			return true;
		}
	}
	
	return false;
}

bool CKLTTracker::IsBlankRegionExisted(int imageID, int * existing_features, int &n_existing_features)
{
	int i;
	VEC_TP::iterator iter;

	// Reset the number of tie points in the subregion (existing_features)
	memset(existing_features, 0, SUBPATTERN * SUBPATTERN * sizeof(int));

	n_existing_features = 0;

	// Iterate through m_vecTP to count the number of tie points in each subregion.
	for (iter = m_vecTP.begin(); iter != m_vecTP.end(); iter++)
	{
		// Increasing the number of tie points in the specified 'imageID'.
		if (imageID == (*iter).imageID)
		{
			existing_features[(*iter).regionID]++;
			n_existing_features++;
		}
	}
	
	// If the pattern is 1x1, always force the program to extract new features.
	if (1 == SUBPATTERN)
		return true;

	// If at least one subregion contains no tie point, return true.
	for (i = 0; i < SUBPATTERN * SUBPATTERN; i++)
	{
		if (0 == existing_features[i])
		{
			return true;
		}
	}
	
	return false;
}

/********************************************************************
 * Description:	The function is to store existing features into each block
 *				of sub-pattern according to its region ID.
 ********************************************************************/
void CKLTTracker::AssignExistingFeaturesToBlock(int imageID, 
												vector<vector<VEC_INT> > &vblock_features, 
												int * existing_features, 
												int &n_existing_features)
{
	int i;
	int iH, iW;
	VEC_TP::iterator iter;

	// Reset the number of tie points in the subregion (existing_features)
	memset(existing_features, 0, SUBPATTERN * SUBPATTERN * sizeof(int));

	n_existing_features = 0;

	// Iterate through m_vecTP to store existing features into each subregion.
	for (iter = m_vecTP.begin(); iter != m_vecTP.end(); iter++)
	{
		// Increasing the number of tie points in the specified 'imageID'.
		if (imageID == (*iter).imageID)
		{
			// Obtain the row/col ID of the feature's region ID.
			FindRowColFromRegionID((*iter).regionID, /*out*/iH, /*out*/iW);

			// Put the point ID into the sub-pattern.
			(vblock_features[iH][iW]).push_back((*iter).pointID);
			
			// Count the number of features in each region.
			existing_features[(*iter).regionID]++;
			
			// Increment the number of features.
			n_existing_features++;
		}
	}

}

/********************************************************************
 * Description:	To check if the passed-in point already existed.
 ********************************************************************/
bool CKLTTracker::IsMatchingPointsExisted(int imageID, CvPoint2D32f point)
{
	VEC_TP::iterator iter;
	double dist;

	// Iterate through the m_vecTP to validate if the point existed.
	for (iter = m_vecTP.begin(); iter != m_vecTP.end(); iter++) 
	{
		if (imageID == (*iter).imageID)
		{
			CvPoint2D32f mpcoord = (*iter).coord;

			// Check if the point already existed by
			//	calculating the Euclidian distance between point.
			dist =	(point.y - mpcoord.y) * (point.y - mpcoord.y) + 
					(point.x - mpcoord.x) * (point.x - mpcoord.x);
			
			if DBL_LT(dist, MIN_DISTANCE)
				return true;

		}
	}

	return false;
}

/********************************************************************
 * Description:	To add the signficant features to the list of features
 *				to be tracked.
 ********************************************************************/
void CKLTTracker::AddMatchedPointsForTracking(	int imageID, 
												CvPoint2D32f * cornersA, 
												int &nPoint, 
												int * cornersA_GPID)
{
	VEC_TP::iterator iter;

	// Reset the number of points to zero.
	nPoint = 0;

	// Continue searching tie points in the specified 'imageID'.
	for (iter = m_vecTP.begin(); iter != m_vecTP.end(); iter++)
	{
		if (imageID == (*iter).imageID)
		{
			CvPoint2D32f mpcoord = (*iter).coord;
			
			// Assign the tie point to the vector.
			cornersA[nPoint] = mpcoord;

			// Store the 'GPID' of the tie points in cornersA into cornersA_GPID
			cornersA_GPID[nPoint] = (*iter).pointID;

			nPoint++;			
		}
	}
}

/********************************************************************
 * Description:	This function is to calculate the initial guessed 
 *				positions of the tie points based on the collinearity
 *				equation.
 ********************************************************************/
void CKLTTracker::CalInitialGuesses(int corner_count, 
									CvPoint2D32f * cornersA, 
									/*out*/CvPoint2D32f * cornersB)
{
	// Initialize a temporaly buffer to store calculated coordinates.
	CvPoint2D32f * arrCoords;		
	arrCoords = new CvPoint2D32f[corner_count];

	// Convert the pixel coordinates to the photo coordinates.
	ConvertPix2PhotoCoords(corner_count, cornersA, arrCoords);

	// Project points in the first image to the second image using collinearity equation.
	CollinearityEquation(arrCoords, corner_count);

	// Convert the photo coordinates to the pixel coordinates.
	ConvertPhoto2PixCoords(corner_count, arrCoords, cornersB);

	// Free the memory.
	delete [] arrCoords;
}

/********************************************************************
 * Description:	This function is to convert the pixel coordinates 
 *				stored in the cornerA to the photo coordinates
 *				and store them in the arrCoords variable.
 ********************************************************************/
void CKLTTracker::ConvertPix2PhotoCoords(	int corner_count,	
											CvPoint2D32f * cornersA, 
											/*out*/CvPoint2D32f * arrCoords)
{
	CvPoint2D32f coord;

	// Calculculate the photo-coordinates for each points (pixel based)
	for (int i = 0; i < corner_count; i++)
	{
		// Obtain the points in scaled coordinate.
		coord = cornersA[i];

		// Scale to original coordinate.
		coord.x = coord.x * SCALE_FACTOR;
		coord.y = coord.y * SCALE_FACTOR;		

		// Convert to photo-coordinate and store into the output array.
		arrCoords[i].x = (coord.x - m_ICenter.dX) * m_PS.dX - m_IO.dPPX;
		arrCoords[i].y = (m_ICenter.dY - coord.y) * m_PS.dY - m_IO.dPPY;
	}
}

CvPoint2D32f CKLTTracker::ConvertPix2PhotoCoord(CvPoint2D32f inCoord) 
{
	CvPoint2D32f outCoord;

	// Scale to original coordinate.
	inCoord.x = inCoord.x * SCALE_FACTOR;
	inCoord.y = inCoord.y * SCALE_FACTOR;

	// Calculculate the photo-coordinates for passed-in pixel coord.
	outCoord.x = (inCoord.x - m_ICenter.dX) * m_PS.dX - m_IO.dPPX;
	outCoord.y = (m_ICenter.dY - inCoord.y) * m_PS.dY - m_IO.dPPY;

	return outCoord;
}

/********************************************************************
 * Description:	This function is to convert the photo coordinates 
 *				stored in the arrCoords to the pixel coordinates
 *				and store them in the cornersB.
 ********************************************************************/
void CKLTTracker::ConvertPhoto2PixCoords(	int corner_count,	
											CvPoint2D32f * arrCoords, 
											/*out*/CvPoint2D32f * cornersB)
{
	CvPoint2D32f coord;

	// Calculculate the pixel coordinates from the photo coordinates.
	for (int i = 0; i < corner_count; i++)
	{
		// Obtain the photo coordinate.
		coord = arrCoords[i];

		// Convert to pixel coordinate.
		coord.x = m_ICenter.dX + (coord.x + m_IO.dPPX) / m_PS.dX;
		coord.y = m_ICenter.dY - (coord.y + m_IO.dPPY) / m_PS.dY;

		// Scale the coordinate and store into the output array.
		cornersB[i].x = coord.x / SCALE_FACTOR;
		cornersB[i].y = coord.y / SCALE_FACTOR;
	}
}

CvPoint2D32f CKLTTracker::ConvertPhoto2PixCoord(CvPoint2D32f inCoord) 
{
	CvPoint2D32f outCoord;

	// Calculculate the photo-coordinates for passed-in pixel coord.
	outCoord.x = m_ICenter.dX + (inCoord.x + m_IO.dPPX) / m_PS.dX;
	outCoord.y = m_ICenter.dY - (inCoord.y + m_IO.dPPY) / m_PS.dY;

	// Scale the pixel coordinate.
	outCoord.x = outCoord.x / SCALE_FACTOR;
	outCoord.y = outCoord.y / SCALE_FACTOR;

	return outCoord;
}

/********************************************************************
 * Description:	This function is to map one point in the first image
 *				to the corresponding point in the second image based
 *				on the collinearity equation.
 ********************************************************************/
void CKLTTracker::CollinearityEquation(CvPoint2D32f * arrCoords, int corner_count)
{
	double	r11, r12, r13;
	double	r21, r22, r23;
	double	r31, r32, r33;
	double	T1, T2, T3;
	double	dgroundX, dgroundY;
	CvPoint2D32f coord;
	int		i;
	EO		eo;
	double	x;				// Angle		
	double	cos_x, sin_x;	// cos(Angle), sin(Angle)
	CvMat	*Rx, *Ry, *Rz;	// Rotational matrix
	CvMat	*R;				// Final matrix

	// STEP 0. Initialize the rotational matrix.
	Rx		= cvCreateMat(3, 3, CV_32FC1);
	Ry		= cvCreateMat(3 ,3, CV_32FC1);
	Rz		= cvCreateMat(3, 3, CV_32FC1);
	R		= cvCreateMat(3 ,3, CV_32FC1);

	// STEP 1. Convert the pixel coordinates in Image#1 to the ground coordinates.
	eo = m_deqEO[IMG_A];

	// Construct rotational matrix for Image#1

	if (ROT_TYPE == 1)
	{
		//		|	1		0		0		|
		// Rx =	|	0	  cos(Om) -sin(Om)	|
		//		|	0	  sin(Om) cos(Om)	|		
		
		x		=	eo.dOmega;
		cos_x	=	cos(x);
		sin_x	=	sin(x);

		cvmSet(Rx, 0, 0, 1.0);
		cvmSet(Rx, 0, 1, 0.0);
		cvmSet(Rx, 0, 2, 0.0);
		cvmSet(Rx, 1, 0, 0.0);
		cvmSet(Rx, 1, 1, cos_x);
		cvmSet(Rx, 1, 2, -sin_x);
		cvmSet(Rx, 2, 0, 0.0);
		cvmSet(Rx, 2, 1, sin_x);
		cvmSet(Rx, 2, 2, cos_x);
		
		//		|	cos(Ph)		0		sin(Ph)	|
		// Ry =	|	  0			1		   0	|
		//		|	-sin(Ph)	0		cos(Ph)	|
		
		x		=	eo.dPhi;
		cos_x	=	cos(x);
		sin_x	=	sin(x);

		cvmSet(Ry, 0, 0, cos_x);
		cvmSet(Ry, 0, 1, 0.0);
		cvmSet(Ry, 0, 2, sin_x);
		cvmSet(Ry, 1, 0, 0.0);
		cvmSet(Ry, 1, 1, 1.0);
		cvmSet(Ry, 1, 2, 0.0);
		cvmSet(Ry, 2, 0, -sin_x);
		cvmSet(Ry, 2, 1, 0.0);
		cvmSet(Ry, 2, 2, cos_x);

		//		|	cos(Ka)		-sin(Ka)	0	|
		// Rz =	|	sin(Ka)		cos(Ka)		0	|
		//		|	  0			   0		1	|
		
		x		=	eo.dKappa;	// Kappa
		cos_x	=	cos(x);
		sin_x	=	sin(x);

		cvmSet(Rz, 0, 0, cos_x);
		cvmSet(Rz, 0, 1, -sin_x);
		cvmSet(Rz, 0, 2, 0.0);
		cvmSet(Rz, 1, 0, sin_x);
		cvmSet(Rz, 1, 1, cos_x);
		cvmSet(Rz, 1, 2, 0.0);
		cvmSet(Rz, 2, 0, 0.0);
		cvmSet(Rz, 2, 1, 0.0);
		cvmSet(Rz, 2, 2, 1.0);
	}
	else // ROT_TYPE = 2
	{
		//		|	1			0		0		|
		// Rx =	|	0		 cos(Om)	sin(Om)	|
		//		|	0		 -sin(Om)	cos(Om)	|		
		
		x		=	eo.dOmega;
		cos_x	=	cos(x);
		sin_x	=	sin(x);

		cvmSet(Rx, 0, 0, 1.0);
		cvmSet(Rx, 0, 1, 0.0);
		cvmSet(Rx, 0, 2, 0.0);
		cvmSet(Rx, 1, 0, 0.0);
		cvmSet(Rx, 1, 1, cos_x);
		cvmSet(Rx, 1, 2, sin_x);
		cvmSet(Rx, 2, 0, 0.0);
		cvmSet(Rx, 2, 1, -sin_x);
		cvmSet(Rx, 2, 2, cos_x);
		
		//		|	cos(Ph)		0		-sin(Ph)|
		// Ry =	|	  0			1		   0	|
		//		|	sin(Ph)		0		cos(Ph)	|
		
		x		=	eo.dPhi;
		cos_x	=	cos(x);
		sin_x	=	sin(x);

		cvmSet(Ry, 0, 0, cos_x);
		cvmSet(Ry, 0, 1, 0.0);
		cvmSet(Ry, 0, 2, -sin_x);
		cvmSet(Ry, 1, 0, 0.0);
		cvmSet(Ry, 1, 1, 1.0);
		cvmSet(Ry, 1, 2, 0.0);
		cvmSet(Ry, 2, 0, sin_x);
		cvmSet(Ry, 2, 1, 0.0);
		cvmSet(Ry, 2, 2, cos_x);

		//		|	cos(Ka)		sin(Ka)		0	|
		// Rz =	|	-sin(Ka)	cos(Ka)		0	|
		//		|	  0			   0		1	|
		
		x		=	eo.dKappa;	// Kappa
		cos_x	=	cos(x);
		sin_x	=	sin(x);

		cvmSet(Rz, 0, 0, cos_x);
		cvmSet(Rz, 0, 1, sin_x);
		cvmSet(Rz, 0, 2, 0.0);
		cvmSet(Rz, 1, 0, -sin_x);
		cvmSet(Rz, 1, 1, cos_x);
		cvmSet(Rz, 1, 2, 0.0);
		cvmSet(Rz, 2, 0, 0.0);
		cvmSet(Rz, 2, 1, 0.0);
		cvmSet(Rz, 2, 2, 1.0);
	}

	if (ROT_ORDER == 1) // R = Rx * Ry * Rz
	{
		cvMatMul(Rx, Ry, R);	// R = Rx * Ry
		cvMatMul(R, Rz, R);		// R = (Rx * Ry) * Rz
	}
	else // R = Rz * Ry * Rx
	{
		cvMatMul(Rz, Ry, R);	// R = Rz * Ry
		cvMatMul(R, Rx, R);		// R = (Rz * Ry) * Rx
	}

	//		|	r11	r12	r13	|
	// R =	|	r21	r22	r23	|
	//		|	r31	r32	r33	|

	r11 = cvmGet(R, 0, 0);	r12 = cvmGet(R, 0, 1);	r13 = cvmGet(R, 0, 2);
	r21 = cvmGet(R, 1, 0);	r22 = cvmGet(R, 1, 1);	r23 = cvmGet(R, 1, 2);
	r31 = cvmGet(R, 2, 0);	r32 = cvmGet(R, 2, 1);	r33 = cvmGet(R, 2, 2);

	// Compute the ground coordinates.

	for (i = 0; i < corner_count; i++)
	{
		coord = arrCoords[i];

		T1 = r11*coord.x + r21*coord.y - r31*m_IO.dF;

		T2 = r12*coord.x + r22*coord.y - r32*m_IO.dF;

		T3 = r13*coord.x + r23*coord.y - r33*m_IO.dF;

		dgroundX = (ZA - eo.dZc)*T1/T3 + eo.dXc;
		dgroundY = (ZA - eo.dZc)*T2/T3 + eo.dYc;

		arrCoords[i].x = dgroundX;
		arrCoords[i].y = dgroundY;
	}

	// STEP 2. Convert the ground coordinates to the pixel coordinates in Image#2.
	eo = m_deqEO[IMG_B];

	// Construct rotational matrix for Image#2

	if (ROT_TYPE == 1)
	{
		//		|	1		0		0		|
		// Rx =	|	0	  cos(Om) -sin(Om)	|
		//		|	0	  sin(Om) cos(Om)	|		
		
		x		=	eo.dOmega;
		cos_x	=	cos(x);
		sin_x	=	sin(x);

		cvmSet(Rx, 0, 0, 1.0);
		cvmSet(Rx, 0, 1, 0.0);
		cvmSet(Rx, 0, 2, 0.0);
		cvmSet(Rx, 1, 0, 0.0);
		cvmSet(Rx, 1, 1, cos_x);
		cvmSet(Rx, 1, 2, -sin_x);
		cvmSet(Rx, 2, 0, 0.0);
		cvmSet(Rx, 2, 1, sin_x);
		cvmSet(Rx, 2, 2, cos_x);
		
		//		|	cos(Ph)		0		sin(Ph)	|
		// Ry =	|	  0			1		   0	|
		//		|	-sin(Ph)	0		cos(Ph)	|
		
		x		=	eo.dPhi;
		cos_x	=	cos(x);
		sin_x	=	sin(x);

		cvmSet(Ry, 0, 0, cos_x);
		cvmSet(Ry, 0, 1, 0.0);
		cvmSet(Ry, 0, 2, sin_x);
		cvmSet(Ry, 1, 0, 0.0);
		cvmSet(Ry, 1, 1, 1.0);
		cvmSet(Ry, 1, 2, 0.0);
		cvmSet(Ry, 2, 0, -sin_x);
		cvmSet(Ry, 2, 1, 0.0);
		cvmSet(Ry, 2, 2, cos_x);

		//		|	cos(Ka)		-sin(Ka)	0	|
		// Rz =	|	sin(Ka)		cos(Ka)		0	|
		//		|	  0			   0		1	|
		
		x		=	eo.dKappa;	// Kappa
		cos_x	=	cos(x);
		sin_x	=	sin(x);

		cvmSet(Rz, 0, 0, cos_x);
		cvmSet(Rz, 0, 1, -sin_x);
		cvmSet(Rz, 0, 2, 0.0);
		cvmSet(Rz, 1, 0, sin_x);
		cvmSet(Rz, 1, 1, cos_x);
		cvmSet(Rz, 1, 2, 0.0);
		cvmSet(Rz, 2, 0, 0.0);
		cvmSet(Rz, 2, 1, 0.0);
		cvmSet(Rz, 2, 2, 1.0);
	}
	else // ROT_TYPE = 2
	{
		//		|	1			0		0		|
		// Rx =	|	0		 cos(Om)	sin(Om)	|
		//		|	0		 -sin(Om)	cos(Om)	|		
		
		x		=	eo.dOmega;
		cos_x	=	cos(x);
		sin_x	=	sin(x);

		cvmSet(Rx, 0, 0, 1.0);
		cvmSet(Rx, 0, 1, 0.0);
		cvmSet(Rx, 0, 2, 0.0);
		cvmSet(Rx, 1, 0, 0.0);
		cvmSet(Rx, 1, 1, cos_x);
		cvmSet(Rx, 1, 2, sin_x);
		cvmSet(Rx, 2, 0, 0.0);
		cvmSet(Rx, 2, 1, -sin_x);
		cvmSet(Rx, 2, 2, cos_x);
		
		//		|	cos(Ph)		0		-sin(Ph)|
		// Ry =	|	  0			1		   0	|
		//		|	sin(Ph)		0		cos(Ph)	|
		
		x		=	eo.dPhi;
		cos_x	=	cos(x);
		sin_x	=	sin(x);

		cvmSet(Ry, 0, 0, cos_x);
		cvmSet(Ry, 0, 1, 0.0);
		cvmSet(Ry, 0, 2, -sin_x);
		cvmSet(Ry, 1, 0, 0.0);
		cvmSet(Ry, 1, 1, 1.0);
		cvmSet(Ry, 1, 2, 0.0);
		cvmSet(Ry, 2, 0, sin_x);
		cvmSet(Ry, 2, 1, 0.0);
		cvmSet(Ry, 2, 2, cos_x);

		//		|	cos(Ka)		sin(Ka)		0	|
		// Rz =	|	-sin(Ka)	cos(Ka)		0	|
		//		|	  0			   0		1	|
		
		x		=	eo.dKappa;	// Kappa
		cos_x	=	cos(x);
		sin_x	=	sin(x);

		cvmSet(Rz, 0, 0, cos_x);
		cvmSet(Rz, 0, 1, sin_x);
		cvmSet(Rz, 0, 2, 0.0);
		cvmSet(Rz, 1, 0, -sin_x);
		cvmSet(Rz, 1, 1, cos_x);
		cvmSet(Rz, 1, 2, 0.0);
		cvmSet(Rz, 2, 0, 0.0);
		cvmSet(Rz, 2, 1, 0.0);
		cvmSet(Rz, 2, 2, 1.0);
	}

	if (ROT_ORDER == 1) // R = Rx * Ry * Rz
	{
		cvMatMul(Rx, Ry, R);	// R = Rx * Ry
		cvMatMul(R, Rz, R);		// R = (Rx * Ry) * Rz
	}
	else // R = Rz * Ry * Rx
	{
		cvMatMul(Rz, Ry, R);	// R = Rz * Ry
		cvMatMul(R, Rx, R);		// R = (Rz * Ry) * Rx
	}

	//		|	r11	r12	r13	|
	// R =	|	r21	r22	r23	|
	//		|	r31	r32	r33	|

	r11 = cvmGet(R, 0, 0);	r12 = cvmGet(R, 0, 1);	r13 = cvmGet(R, 0, 2);
	r21 = cvmGet(R, 1, 0);	r22 = cvmGet(R, 1, 1);	r23 = cvmGet(R, 1, 2);
	r31 = cvmGet(R, 2, 0);	r32 = cvmGet(R, 2, 1);	r33 = cvmGet(R, 2, 2);

	// Compute the image coordinate.

	for (i = 0; i < corner_count; i++)
	{
		dgroundX = arrCoords[i].x;
		dgroundY = arrCoords[i].y;

		T1 = (dgroundX - eo.dXc)*r11 + (dgroundY - eo.dYc)*r12 + (ZA - eo.dZc)*r13;
		T2 = (dgroundX - eo.dXc)*r21 + (dgroundY - eo.dYc)*r22 + (ZA - eo.dZc)*r23;
		T3 = (dgroundX - eo.dXc)*r31 + (dgroundY - eo.dYc)*r32 + (ZA - eo.dZc)*r33;

		coord.x = - m_IO.dF*T1/T3;
		coord.y = - m_IO.dF*T2/T3;

		arrCoords[i] = coord;
	}
}

/********************************************************************
 * Description:	This function will be called whenever a new image
 *				is captured. 
 ********************************************************************/
void CKLTTracker::GetNewImage(string image_name, EO eo)
{
	// Store the new capture data to the deque which allows a maximum
	// of two sets of data at a time.	
	
	// Store the new capture data to the deque.
	m_deqEO.push_back(eo);
	m_deqIMG.push_back(image_name);

	// Maintain the deque to store a maximum of two sets of data at a time.
	if (m_deqEO.size() > MAX_TRACKING_IMG)
	{
		// Obtain the obsoleted image ID.
		int iObsoletedIMG = m_deqEO[0].iImg;
	
		// Remove the obsoleted images.
		m_deqEO.pop_front();
		m_deqIMG.pop_front();

		// Remove the obsoleted tie points from the m_vecTP vector.
		(void) RemoveObsoletedTiepoints(iObsoletedIMG);

		// Reuse previous read image pointer.
		m_bReloadIMG = false;		
	}

	// Assert to ensure the deque stores a maximum of two images at a time.
	_ASSERT( m_deqEO.size() <= MAX_TRACKING_IMG );

	// If there exist two images, perform tracking.
	if (m_deqEO.size() == MAX_TRACKING_IMG)
	{
		Track();
	}

}

/********************************************************************
 * Description:	This function will perform the tracking. 
 ********************************************************************/
void CKLTTracker::Track()
{
	IplImage* pyrA = NULL;
	IplImage* pyrB = NULL;
	IplImage* eig_image = NULL;
	IplImage* tmp_image = NULL;
	IplImage* roiA = NULL;
	CvPoint2D32f* cornersA;
	CvPoint2D32f* cornersB;
	int * cornersA_GPID;			// To store the GPID of the corresponding tie points in cornersA.
	int corner_count;				// The number of corners (features)
	int iH, iW, i, p;
	int idxRow, idxCol;				// Row and Column index of sub-pattern block.
	int xROI, yROI;					// Beginning coordinates of ROI (top-left coordinate)
	int widthROI, heightROI;		// Dimension of ROI
	int * existing_features;		// To store the number of tie points in each pattern block.
	int n_existing_features;		// To store the number of existing features.
	string imgA_file, imgB_file;	// String file name of Image A and Image B
	EO	eoA, eoB;					// EO parameters for Image A and Image B
	bool bRequireFindingGoodFeatures;	// A flag to indicate if the feature selection process will be run.
	VEC_INT vec_num_matched(9, 0);		// To store the number of matched points in each block.
	
	// Measure the computational time
	SYSTEMTIME startTime, finishTime;
	DWORD start, end;
	GetLocalTime(&startTime);
	start = GetTickCount();	

	/************** START THE KLT TRACKING **************/
	
	// Get the tracking image file names.
	imgA_file = m_deqIMG[IMG_A];
	imgB_file = m_deqIMG[IMG_B];

	// Get the corresponding EO of the tracking images.
	eoA = m_deqEO[IMG_A];
	eoB = m_deqEO[IMG_B];

		// DEBUG
		printf("****************************************************\n");
		printf("*****             KLT Image Matching           *****\n");
		printf("****************************************************\n");
		printf("Tracking image: [%d] %s\n", eoA.iImg, imgA_file.c_str());
		printf("Tracking image: [%d] %s\n", eoB.iImg, imgB_file.c_str());
		// DEBUG

	// Construct the actual path and filename of the tracking images.
	imgA_file = m_img_directory + imgA_file;
	imgB_file = m_img_directory + imgB_file;

	//START----- PERFORMANCE IMPROVEMENT SOLUTION -----

	// Load the 1st image and 2nd image (A and B) into the CV image structures.
	if (1 == SCALE_FACTOR) // Load the original image size.
	{

		if (m_bReloadIMG == true)
		{
			imgA = cvLoadImage(imgA_file.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
			imgB = cvLoadImage(imgB_file.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
		}
		else // Reuse the initialized image by swapping pointers.
		{
			// Release the existing A image.
			if (imgA != NULL)
			{
				cvReleaseImage(&imgA);
				imgA = NULL;
			}

			// Set the image A ptr to the previous allocation of image B.
			imgA = imgB;
		
			// Load the new image B.
			imgB = cvLoadImage(imgB_file.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
		}

	}
	else // Load the scaling image size.
	{
		if (m_bReloadIMG == true) // Re-read both images.
		{
			IplImage *	srcA;		// Temporary storage for Image#A
			IplImage *	srcB;		// Temporary storage for Image#B
			CvSize		img_sz;		// Image dimension

			// Load the images into the CV image structure.
			srcA = cvLoadImage(imgA_file.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
			srcB = cvLoadImage(imgB_file.c_str(), CV_LOAD_IMAGE_GRAYSCALE);

			// Construct the scaled images.
			imgA = cvCreateImage(cvSize(m_scaledIS.iWidth, m_scaledIS.iHeight), srcA->depth, srcA->nChannels);
			cvResize(srcA, imgA, CV_INTER_LINEAR);

			imgB = cvCreateImage(cvSize(m_scaledIS.iWidth, m_scaledIS.iHeight), srcB->depth, srcB->nChannels);
			cvResize(srcB, imgB, CV_INTER_LINEAR);

			// Release the temporary storages.
			if (srcA != NULL)
				cvReleaseImage(&srcA);
			if (srcB != NULL)
				cvReleaseImage(&srcB);

		}
		else // Reuse the initialized image by swapping pointers.
		{
			// Release the existing A image.
			if (imgA != NULL)
			{
				cvReleaseImage(&imgA);
				imgA = NULL;
			}

			// Set the image A ptr to the previous allocation of image B.
			imgA = imgB;

			// --- Load and scale the new image B.

			IplImage *	srcB;		// Temporary storage for Image#B
			CvSize		img_sz;		// Image dimension

			// Load the images into the CV image structure.
			srcB = cvLoadImage(imgB_file.c_str(), CV_LOAD_IMAGE_GRAYSCALE);

			// Construct the scaled images.
			imgB = cvCreateImage(cvSize(m_scaledIS.iWidth, m_scaledIS.iHeight), srcB->depth, srcB->nChannels);
			cvResize(srcB, imgB, CV_INTER_LINEAR);

			// Release the temporary storages.
			if (srcB != NULL)
				cvReleaseImage(&srcB);
		
		}
	}

	//END----- PERFORMANCE IMPROVEMENT SOLUTION -----

	//START----- EXTRACTING GOOD FEATURES -----

		// DEBUG
		int iNewPoints = 0;
		// DEBUG

	// Initialize the pattern block to store features (Point ID) locating in each block.
	vector<vector<VEC_INT> > vblock_features(SUBPATTERN, vector<VEC_INT>(SUBPATTERN));

	// Initialize the pattern block to store the number of tie points in each block.
	existing_features = new int[SUBPATTERN * SUBPATTERN];

	// Store existing features into each sub-pattern of vblock_features.
	AssignExistingFeaturesToBlock(eoA.iImg, vblock_features, existing_features, n_existing_features);

			// DEBUG
			printf("\t Number of existing points in first image is %d\n", n_existing_features);
			// DEBUG

	for (iH = 0; iH < SUBPATTERN; iH++)
	{
		for (iW = 0; iW < SUBPATTERN; iW++)
		{
			// If the number of features already reaches the MAX_CORNERS, just skip.
			if ((vblock_features[iH][iW]).size() >= MAX_CORNERS)
			{
					// DEBUG
					printf("\t\t Region[%d][%d] : %d points -- reached the maximum\n", iH, iW, (vblock_features[iH][iW]).size());
					// DEBUG

				continue;
			}

			// ----- Create ROI for sub-region (iH,iW)

			// Obtain the ROI for subpattern [iH][iW].
			DefineROI(iH, iW, xROI, yROI, widthROI, heightROI);

			// Sub-divide image A to obtain ROI for the region [iH][iW].
			cvSetImageROI(imgA, cvRect(xROI, yROI, widthROI, heightROI));

			// Create a canvas for the ROI on the image A.
			roiA = cvCreateImage(cvGetSize(imgA), imgA->depth, imgA->nChannels);

			// Copy the ROI to the recent created canvas.
			cvCopy(imgA, roiA, NULL);

			// Reset the ROI to recover the original imgA.
			cvResetImageROI(imgA);

			// ----- End of creating ROI for sub-region (iH,iW)

			// ----- Perform the good features to track for the ROI (iH,iW)

			// Obtain the size of the ROI
			CvSize img_sz = cvGetSize( roiA );

			// Perform the good feature to track in the ROI (Image A).
			eig_image = cvCreateImage( img_sz, IPL_DEPTH_32F, 1 );
			tmp_image = cvCreateImage( img_sz, IPL_DEPTH_32F, 1 );
			corner_count = MAX_CORNERS - (vblock_features[iH][iW]).size();
			cornersA = new CvPoint2D32f[ corner_count ];

			_ASSERT(cornersA != NULL);

			cvGoodFeaturesToTrack(
					roiA,				// Original image
					eig_image,			// Contains minimal eigenvalues
					tmp_image,
					cornersA,			// Contains results after run
					&corner_count,		// The number of corner
					MINIMUM_EIGENVALUE,	// Minimum eigenvalue
					MIN_DISTANCE,		// Minimum distance: multiple points within small region
					0,
					BLOCK_AUTOCORR,		// Block size for autocorrelation
					0,					// Use Shi and Tomasi
					0.04				// Used by Harris = NOT USED HERE
				);

			cvFindCornerSubPix(
					roiA,
					cornersA,
					corner_count,
					cvSize(WIN_SIZE_SUBPIXEL,WIN_SIZE_SUBPIXEL),
					cvSize(-1,-1),
					cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,CRI_NUM_ITERATION,CRI_EPSILON_LIMIT)
				);

			// Get the region ID from the coordinate.
			int regionID = 	FindSubRegionFromID(iH, iW);

			// Validate and store the extracted features.
			for (p=0; p<corner_count; p++)
			{
				// Continue to next point if either of its values is negative.
				if ( DBL_LT(cornersA[p].x, 1e-10) || DBL_LT(cornersA[p].y, 1e-10) )
					continue;

				// If the point already existed, continue with other points.
				if (IsMatchingPointsExisted(eoA.iImg, cornersA[p]))
					continue;

				// Otherwise, store the points in the matching results.
				TIE_POINT mp;
				mp.pointID = RUNNING_POINT_ID;
				mp.imageID = eoA.iImg;
				mp.regionID = regionID;
				mp.coord = cornersA[p];
				m_vecTP.push_back(mp);
				
				// Increment the world point ID
				RUNNING_POINT_ID++;

					// DEBUG
					iNewPoints++;
					// DEBUG	

				// Store the point in the subregion array as well.
				(vblock_features[iH][iW]).push_back(p);
			}

			// Clear the memory
			delete [] cornersA;
			cornersA = NULL;

			if (eig_image != NULL)
				cvReleaseImage(&eig_image);
			if (tmp_image != NULL)
				cvReleaseImage(&tmp_image);
			if (roiA != NULL)
				cvReleaseImage(&roiA);

			// DEBUG
			printf("\t\t Region[%d][%d] : %d points\n", iH, iW, (vblock_features[iH][iW]).size());
			// DEBUG

		}
	}
			// DEBUG
			printf("\t Number of new points added in first image is %d\n", iNewPoints);
			// DEBUG

	//END----- EXTRACTING GOOD FEATURES -----

	// ----- Start performing the improved KLT image matching.
	{
		// Reconstruct the "cornerA" to store the points to be tracked.
		cornersA = new CvPoint2D32f[ MAX_CORNERS * SUBPATTERN * SUBPATTERN];

		// Reconstruct the array storing the tie points' GPID.
		cornersA_GPID = new int[ MAX_CORNERS * SUBPATTERN * SUBPATTERN ];

		_ASSERT(cornersA != NULL);

		// Add the significant points in the 1st image to the list to be tracked.
		AddMatchedPointsForTracking(eoA.iImg, &cornersA[0], corner_count, &cornersA_GPID[0]);

#ifdef __LSM_DEBUGGING

			// To print out the tracking features

			char fileName[50] = {0};
			sprintf(fileName, "CornerA_%s.txt", m_deqIMG[IMG_A].c_str());

			FILE * fA = fopen(fileName, "w");
			for (i = 0; i < corner_count; i++)
			{
				fprintf(fA, "[%d]\t%f\t%f\n", cornersA_GPID[i], cornersA[i].x, cornersA[i].y);
			}
			fclose(fA);

#endif //__LSM_DEBUGGING

			///// DEBUG
			// writeFile(VEC_IMG_FILE[iPair].c_str(), VEC_IMG_FILE[iPair+1].c_str(), 0, cornersA, corner_count, matching);
			///// DEBUG

		// Construct the tracking parameters
		char * features_found = new char [ corner_count ];
		float * feature_errors = new float [ corner_count ];
		CvSize pyr_sz = cvSize( imgA->width+8, imgA->height/3 );

		pyrA = cvCreateImage( pyr_sz, IPL_DEPTH_32F, 1 );
		pyrB = cvCreateImage( pyr_sz, IPL_DEPTH_32F, 1 );

		cornersB = new CvPoint2D32f[ corner_count ];

		// CALCULATE THE INITIAL GUESS POSITION
		if (CAL_INITIAL_GUESS == 1)
			CalInitialGuesses(corner_count, cornersA, cornersB);

		// Define the tracking flag whether or not initial guessed positions exist.
		KLT_TRACKING_FLAG = (CAL_INITIAL_GUESS==0)? 0 : CV_LKFLOW_INITIAL_GUESSES;


		#ifdef __LSM_DEBUGGING
			
			// To print out the initial guessed values.

			char fileGName[100] = {0};
			sprintf(fileGName, "GUESSED_TP_%s.txt", m_deqIMG[IMG_A].c_str());

			FILE * f = fopen(fileGName, "w");
			fprintf(f, "ID\tA.x\tA.y\tGuess.x\tGuess.y\n");
			for (i = 0; i < corner_count; i++)
			{
				fprintf(f, "%d\t%lf\t%lf\t%lf\t%lf\n", i, 
					cornersA[i].x * SCALE_FACTOR, cornersA[i].y * SCALE_FACTOR, 
					cornersB[i].x * SCALE_FACTOR, cornersB[i].y * SCALE_FACTOR);
			}
			fclose(f);

		#endif //__LSM_DEBUGGING

		// Perform the pyramidal tracking.
		cvCalcOpticalFlowPyrLK(
				imgA,					// Image at time t
				imgB,					// Image at time t+1
				pyrA,					// Buffer to store input image A
				pyrB,					// Buffer to store input image B
				cornersA,				// Point in image A to be track
				cornersB,				// Point in new location
				corner_count,
				cvSize( WIN_SIZE_SUBPIXEL,WIN_SIZE_SUBPIXEL ),	// Window size
				DEPTH_LEVELS,			// Level
				features_found,			// status
				feature_errors,
				cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, CRI_NUM_ITERATION, CRI_EPSILON_LIMIT),
				KLT_TRACKING_FLAG//CV_LKFLOW_INITIAL_GUESSES//0
			);

			// DEBUG
			int iNumMatchedPoint = 0;
			// DEBUG

		for(i=0; i<corner_count; i++ ) 
		{
			if( (features_found[i]==0) || (feature_errors[i] > FEATURE_ERROR) || 
				DBL_LT(cornersB[i].x, 1e-10) || DBL_LT(cornersB[i].y, 1e-10) ||
				DBL_GT(cornersB[i].x, m_scaledIS.iWidth) || DBL_GT(cornersB[i].y, m_scaledIS.iHeight) ) 
			{

				#ifdef __LSM_DEBUGGING

					if (features_found[i]==0)
						printf("\tFeatures[%d]:(%lf,%lf) in 1st image _not found_ with error %f\n", 
								i, cornersA[i].x, cornersA[i].y, feature_errors[i]);
					
					if (feature_errors[i] > FEATURE_ERROR)
						printf("\tFeatures[%d]:(%lf,%lf) in 1st image found _with error %f_\n", 
								i, cornersA[i].x, cornersA[i].y, feature_errors[i]);
				
				#endif //__LSM_DEBUGGING

				continue;
			}

			if (CCC_CHECK == 1)
			{
				//***** START : Perform Cross Correlation Check *****//
				int		n_pixels = 0;	// The number of pixels counted for CCC computation.
				int		xt, yt;			// The x,y index for window size.
				int		valA, valB;		// The pixel value for image A and image B.
				CvScalar s;
				double	d_std_A, d_std_B, d_std_AB;
				double	d_sum_sqare_A = 0.0, d_sum_A = 0.0, d_sum_AxB = 0.0;
				double	d_sum_sqare_B = 0.0, d_sum_B = 0.0;
				double	d_ccc;

				// Looping for variance calculation
				for (yt = -WIN_SIZE_SUBPIXEL; yt < WIN_SIZE_SUBPIXEL; yt++)
				{
					for (xt = -WIN_SIZE_SUBPIXEL; xt < WIN_SIZE_SUBPIXEL; xt++)
					{
						// Validate the boundary of window.
						if ( DBL_LT(cornersA[i].y + yt, 0.0) ||
							 DBL_LT(cornersA[i].x + xt, 0.0) ||
							 DBL_GT(cornersA[i].y + yt, double(imgA->height)) ||
							 DBL_GT(cornersA[i].x + xt, double(imgA->width)) ||
							 DBL_LT(cornersB[i].y + yt, 0.0) ||
							 DBL_LT(cornersB[i].x + xt, 0.0) ||
							 DBL_GT(cornersB[i].y + yt, double(imgB->height)) ||
							 DBL_GT(cornersB[i].x + xt, double(imgB->width)) )
						{
							continue;
						}

						// Count the number of pixels for CCC computation.
						n_pixels++;

						// Get value of the image A at the matching position.
						s = cvGet2D(imgA, cornersA[i].y + yt, cornersA[i].x + xt);
						valA = s.val[0];

						// Get value of the image B at the matching position.
						s = cvGet2D(imgB, cornersB[i].y + yt, cornersB[i].x + xt);
						valB = s.val[0];

						// Variance calculation for image A.
						d_sum_sqare_A += valA * valA;
						d_sum_A += valA;

						// Variance calculation for image B.
						d_sum_sqare_B += valB * valB;
						d_sum_B += valB;

						d_sum_AxB += valA * valB;
					}
				}

				if (n_pixels == 0)
					continue;

				// Standard deviation of Image A
				d_std_A		= sqrt((n_pixels * d_sum_sqare_A - d_sum_A*d_sum_A)/n_pixels/(n_pixels-1));

				// Standard deviation of Image B
				d_std_B		= sqrt((n_pixels * d_sum_sqare_B - d_sum_B*d_sum_B)/n_pixels/(n_pixels-1));

				// Covariance between A and B
				d_std_AB	= (n_pixels*d_sum_AxB - d_sum_A*d_sum_B)/n_pixels/(n_pixels-1);

				d_ccc = d_std_AB/(d_std_A*d_std_B);			
				
				//***** END : Perform Cross Correlation Check *****//				

				// Perform validation in comparison with the pre-defined threshold.
				if DBL_LT(d_ccc, CCC_THRESHOLD)
				{
				
					#ifdef	__LSM_DEBUGGING

						printf("\tFeatures[%d]:(x%lf,y%lf) in 1st image _not match_ with ccc %lf\n", i, cornersA[i].x, cornersA[i].y, d_ccc);
					
					#endif //__LSM_DEBUGGING

					continue;
				}
			}

				// DEBUG
				iNumMatchedPoint++;
				// DEBUG


				#ifdef __LSM_DEBUGGING
				
					printf("\tFeatures[%d]:(x%lf,y%lf)||(x%lf,y%lf) accepted with error %f\n", 
							i, cornersA[i].x, cornersA[i].y, cornersB[i].x, cornersB[i].y, feature_errors[i]);
				
				#endif //__LSM_DEBUGGING

			// Add the results into the matching table.
			TIE_POINT mp;
			mp.pointID = cornersA_GPID[i];
			mp.imageID = eoB.iImg;
			mp.regionID = FindSubRegionFromCoord(cornersB[i].y, cornersB[i].x);
			mp.coord = cornersB[i];

			// Check the number of matched prior to store the TP to the matched vector.
			if ((MAX_BLOCK_TP == 0) || (vec_num_matched[mp.regionID] < MAX_BLOCK_TP))
			{
				m_vecTP.push_back(mp);

				// Increment the number of matched points.
				vec_num_matched[mp.regionID] += 1;
			}
			
		}

			// DEBUG
			printf("\t Number of matched points is %d\n", iNumMatchedPoint);
			// DEBUG

			// DEBUG
			for (i = 0; i < SUBPATTERN * SUBPATTERN; i++)
			{
				FindRowColFromRegionID(i, iH, iW); 
				printf("\t\t Region[%d][%d] : %d points\n", iH, iW, vec_num_matched[i]);
			}
			// DEBUG


		if (existing_features != NULL)
			delete [] existing_features;
		if (features_found != NULL)
			delete [] features_found;
		if (feature_errors != NULL)
			delete [] feature_errors;
		if (pyrA != NULL)
			cvReleaseImage(&pyrA);
		if (pyrB != NULL)
			cvReleaseImage(&pyrB);
		if ((iNumMatchedPoint != 0) && (cornersA != NULL))
			delete [] cornersA;
		if ((iNumMatchedPoint != 0) && (cornersB != NULL))
			delete [] cornersB;

	}

	// ----- End of performing the improved KLT image matching.

	/*************** END THE KLT TRACKING ***************/

	// Get the finishing time
	GetLocalTime(&finishTime);
	end = GetTickCount();

		// DEBUG
		printf("\t THE TOTAL TIME USED IS %d milliseconds\n", end - start);
		// DEBUG

}

/********************************************************************
 * Description:	Return the set of tie-point, for the specified IMG_ID
 *				starting from the last stored GP ID, in the form of
 *				the passed-in (by pointer) parameters. 
 *				The function stores the tie points of the specified 
 *				image ID from the m_vecTP vector (declared as VEC_TP) 
 *				into the passed-in vector which is declared as VEC_IP.
 *
 * Return:		The number of tie points stored into the vector.
 ********************************************************************/
int CKLTTracker::GetTiePoints(int IMG_ID, /*out*/VEC_IP * vec_tie_point, int last_stored_GP_ID)
{
	IP					ip;			// Image point 		
	VEC_TP::iterator	iter;		// Iterator for VEC_TP
	int					n_points;	// Total number of tie points.
	CvPoint2D32f		photoCoord;	// Photo coordinate of a point
	FILE *				f;			// Output file.

	// Initialize the number of tie points.
	n_points = 0;

	// Open the file to debug out the TP.txt
	if (true == m_bOutputTP)
	{
		f = fopen("TP.txt", "a");
	}

	// Iterate through the vector of tie points, m_vecTP.
	for (iter = m_vecTP.begin(); iter != m_vecTP.end(); iter++)
	{
		// Store the tie point with the specified image ID into the passed-in vector.
		if ( ((*iter).imageID == IMG_ID) && ((*iter).pointID > last_stored_GP_ID) )
		{
			// Construct the IP object from TP object.
			ip.iImg		= (*iter).imageID;
			ip.iObj		= (*iter).pointID;
			
			// Convert from pixel to photo coordinate.
			photoCoord = ConvertPix2PhotoCoord((*iter).coord);
			
			ip.dX		= photoCoord.x;
			ip.dY		= photoCoord.y;

			// Store the IP object into the passed-in vector.
			vec_tie_point->push_back(ip);

			// Increment the number of tie points.
			n_points++;

			// Debug out tie points into the TP.txt file.
			if (true == m_bOutputTP)
			{
				fprintf(f, "%d\t%d\t%f\t%f\n", ip.iImg, ip.iObj, 
					(*iter).coord.x * SCALE_FACTOR, (*iter).coord.y * SCALE_FACTOR);
			}

		}
	}

	// Close the TP.txt file after writing.
	if (true == m_bOutputTP)
	{
		fclose(f);
	}

	return n_points;
}


/********************************************************************
 * Description:	Remove the obsoleted tie points so that the m_vecTP 
 *				vector contains only the set of tie points extracted
 *				from the latest two images.
 ********************************************************************/
int CKLTTracker::RemoveObsoletedTiepoints(int iObsoletedIMG)
{
	int					nObsoletedTP;		// number of obsoleted TP
	VEC_TP::iterator	iter;				// iterator.

	nObsoletedTP = 0;

	// Iterate through the vector of tie points, m_vecTP.
	for (iter = m_vecTP.begin(); iter != m_vecTP.end(); )
	{
		// Remove the obsoleted tie points by checking its image ID.
		if ( (*iter).imageID <= iObsoletedIMG )
		{
			// After 'erase', it returns a pointer to the next valid iterator.
			iter = m_vecTP.erase(iter);
			
			// Count the number of obsoleted TP
			nObsoletedTP++;
		}
		else
			 iter++;
	}

	return nObsoletedTP;
}
