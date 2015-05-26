/********************************************************
 * Project:			Automated AT
 * Last updated:	17 Jaunary 2011
 * Developer:		Supannee Tanathong
 ********************************************************/

#include <stdio.h>
#include <windows.h>
#include "SimAT.h"				// To use the CSimAT object.
#include "AerialSimulator.h"	// To use the CAerialSimulator object.
#include "KLTTracker.h"			// To use the CKLTTracker object.

void ShowUsage();

void main(int argc, char *argv[])
{
	// 1. Check the input parameters prior to process.
	if (argc < 4)
	{
		ShowUsage();
		return;
	}

	// 2. Initialize the three main components to build the Automated AT system.
	try
	{
		// I Aerial Section
		CAerialSimulator AerialSimulator(argv[1]);
		AerialSimulator.StartAerialSimulator();

		// II Image Matching
		CKLTTracker KLT(argv[2]);
		KLT.Startup();

		// III Simultaneous AT
		CSimAT SimAT(argv[3]);
		SimAT.Initialization();

		// 3. Register the clients of the Aerial simulator.
		AerialSimulator.RegisterKLTClient(&KLT);
		AerialSimulator.RegisterSimAT(&SimAT);

		// 4. Register the KLT object to the Simultaneous AT object.
		SimAT.RegisterKLT(&KLT);

		// 5. Start the Aerial section to capture images then the 
		//	  'whole process' will be automatically started.
		AerialSimulator.RunImageExposure();
	
		// 6. Finally, print out the result.
		SimAT.PrintOutResults();
	}
	catch(...)
	{
		// This is to avoid all exceptions that might be thrown.
	}

}

/*
 * Print out the help message.
 */
void ShowUsage()
{
	printf("Automated Aerial Triangulation System\n");
	printf("     Usage:     AutomatedAT <Aerial-config-file> <KLT-config-file> <AT-config-file>\n");
	printf("   Example:     AutomatedAT AerialConfig.txt KLTConfig.txt SimATConfig.txt\n");
}
