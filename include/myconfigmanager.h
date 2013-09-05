#ifndef __MYCONFIGMANAGER_H
#define __MYCONFIGMANAGER_H
#include <iostream>
#include <stdlib.h>
#include "SimpleIniConfigReader.h"

using namespace LogConfigTime;
using namespace std;

class MyConfigManager
{
	// This method is called by init of the base class to read the configuration values.
    virtual bool readConfiguration(const char *filename, const int argc, const char **argv)
	{
		SimpleIniConfigReader *SIreader = new SimpleIniConfigReader(filename,argc,argv);
		ConfigReader *reader = SIreader;

		showImage = reader->getBoolValue("main","showImage", argc, argv);
		serverPort = reader->getIntValue("main","serverPort", argc, argv);
		camSourceFilename = reader->getStringValue("main","camSourceFilename", argc, argv);
		logFileName = reader->getStringValue("main","logFileName", argc, argv);
		usePs3eye = reader->getBoolValue("main","usePs3eye", argc, argv);
		sendMatImage = reader->getBoolValue("main","sendMatImage", argc, argv);
		camID = reader->getIntValue("main","camID", argc, argv);
		showResponseOnCout = reader->getBoolValue("main","showResponseOnCout", argc, argv);
		camIntrinsicParamsFileName = reader->getStringValue("main","camIntrinsicParamsFileName");

		delete SIreader;
		return true;
	}

public:
    void init(const char *filename, int argc, char **argv)
	{
		readConfiguration(filename,(const int)argc,(const char **)argv);
	}

	// --- Settings
	bool showImage;
	bool sendMatImage;
	bool usePs3eye;
	int camID;	// ID of camera, of <0 for filename
	std::string camSourceFilename;	// If !usePs3eye, may be filename
	std::string logFileName;
	int serverPort;
	bool showResponseOnCout;
	std::string camIntrinsicParamsFileName;

};

#endif

