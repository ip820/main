/********************************************************
 * Project:			Automated AT
 * Last updated:	25 Jaunary 2011
 * Developer:		Supannee Tanathong
 ********************************************************/

#include "AerialSimulator.h"
#include "Global.h"
#include <iostream>
#include <fstream>		// To utilize ifstream for dealing with text file input.
#include <strstream>
#include "stdio.h"

/********************************************************************
 * Description:	Construction
 ********************************************************************/
CAerialSimulator::CAerialSimulator(const string &_CFG_File)
{
	m_CFG_File = _CFG_File; 

	RUNNING_INDEX = 0;			// Initialize the image index.
}

/********************************************************************
 * Description:	Set member variables
 ********************************************************************/
void CAerialSimulator::SetMembers()
{

}

/********************************************************************
 * Description:	Main function to start the Aerial section.  
 ********************************************************************/
void CAerialSimulator::StartAerialSimulator()
{
	// Initialize all neccessary parameters.
	Initialization();
}

/********************************************************************
 * Description:	Call necessary functions for initialization.
 ********************************************************************/
void CAerialSimulator::Initialization()
{
	// Read the configuration file.
	ReadConfig();

	// Read the EO file.
	ReadEOFile();

	// Read the Za file.
	if (1 == ZA_FILE_EXIST)
		ReadZaFile();
}

/********************************************************************
 * Description:	Return the period of time to capture a new image (second)
 ********************************************************************/
double CAerialSimulator::GetExposurePeriod()
{
	return CAPTURE_PERIOD;
}

/********************************************************************
 * Description:	Store the pointer to the singleton CKLTTracker object.
 ********************************************************************/
void CAerialSimulator::RegisterKLTClient(CKLTTracker * pKLT)
{
	m_pKLT = pKLT;
}

/********************************************************************
 * Description:	Store the pointer to the singleton CSimAT object.
 ********************************************************************/
void CAerialSimulator::RegisterSimAT(CSimAT * pSimAT)
{
	m_pSimAT = pSimAT;
}

/********************************************************************
 * Description:	This function is to simulated that the system has 
 *				capture a new image. It then manages to notify its
 *				registered clients that a new image is captured.
 ********************************************************************/
void CAerialSimulator::RunImageExposure()
{
	int nIMG;		// Total number of images.
	int idxIMG;		// Index of image and EO.

	// Get the total number of images.
	nIMG = m_deq_IMG_File.size();

	// Iterate through the deque of the image file and eo.	
	for (idxIMG = 0; idxIMG < nIMG; idxIMG++)
	{
		// Assume that a new image is captured.
		if (1 == ZA_FILE_EXIST)
			OnExposureEventFired(m_deq_IMG_File[idxIMG], m_deq_EO[idxIMG], m_deq_Za[idxIMG]);		
		else
			OnExposureEventFired(m_deq_IMG_File[idxIMG], m_deq_EO[idxIMG]);		
	}

}

/********************************************************************
 * Description:	Handle the process once the exposure event fired.
 ********************************************************************/
void CAerialSimulator::OnExposureEventFired(string image_name, EO eo)
{
	// Notify the KLT tracker that a new image is captured.
	m_pKLT->GetNewImage(image_name, eo);

	// Notify the Simultaneous AT that a new image is captured.
	m_pSimAT->GetNewImage(image_name, eo);
}

void CAerialSimulator::OnExposureEventFired(string image_name, EO eo, double za)
{
	// Notify the KLT tracker that a new image is captured.
	m_pKLT->GetNewImage(image_name, eo, za);

	// Notify the Simultaneous AT that a new image is captured.
	m_pSimAT->GetNewImage(image_name, eo);
}

/********************************************************************
 * Description:	Wrapper function to get each line from the input file.
 ********************************************************************/
void CAerialSimulator::GetLine(istream &in, char *buf, const int bufSize) const
{
	while(in.good())
	{
		in.getline(buf, bufSize);
		if(buf[0]!='%') break;
	}
}

/*******************************************************************
 * Description:	Read the configuration file for the aerial section.
 ********************************************************************/
void CAerialSimulator::ReadConfig()
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

	// Read the time period to capture a new image(second)
	GetLine(fileInput, buffer, BUFFER_SIZE);
	sscanf(buffer, "%lf", &CAPTURE_PERIOD);

	// Read the number of images in the sequence.
	GetLine(fileInput, buffer, BUFFER_SIZE);
	sscanf(buffer, "%d", &TOTAL_IMAGE);

	// Read the EO file name.
	GetLine(fileInput, buffer, BUFFER_SIZE);
	istrstream(buffer) >> m_EO_File;

	// Read the image directory.
	GetLine(fileInput, buffer, BUFFER_SIZE);
	istrstream(buffer) >> m_img_directory;

	// Read the image file names.
	for (int i=0; i<TOTAL_IMAGE; i++)
	{
		string img_file;

		GetLine(fileInput, buffer, BUFFER_SIZE);
		istrstream(buffer) >> img_file;	
		
		// Store the image file name into the deque.
		m_deq_IMG_File.push_back(img_file);
	}

	// Read if Za file used.
	GetLine(fileInput, buffer, BUFFER_SIZE);
	sscanf(buffer, "%d", &ZA_FILE_EXIST);

	if (1 == ZA_FILE_EXIST)
	{
		// Read the Za file name.
		GetLine(fileInput, buffer, BUFFER_SIZE);
		istrstream(buffer) >> m_Za_File;
	}

	// Close the file after finish reading.
	fileInput.close();
}

/********************************************************************
 * Description:	Read the EO file
 ********************************************************************/
void CAerialSimulator::ReadEOFile()
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
	
		m_deq_EO.push_back(eo);
	}

	fclose(fEO);
}

/********************************************************************
 * Description:	Read the Za file
 ********************************************************************/
void CAerialSimulator::ReadZaFile()
{
	double		za;

	// If the file exist, read the EO data from the file.
	FILE * fZa = fopen(m_Za_File.c_str(), "r");

	if (!fZa)
	{
		cout << "Error! Failed to open the Za file." << endl;
		exit(0);
	}

	const int BUFFER_SIZE = 200;
	char buffer[BUFFER_SIZE+1] = {0};

	while(fgets(buffer, BUFFER_SIZE, fZa) != NULL)
	{
		// Skip the comment
		if (buffer[0] == '%')
			continue;

		sscanf(buffer, "%lf\n", &za);
	
		m_deq_Za.push_back(za);
	}

	fclose(fZa);
}
