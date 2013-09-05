#ifndef __MYPHONESERVER_H
#define __MYPHONESERVER_H
#include "PhoneServer.h"
#include "CameraLocalProxy.h"
#include "myconfigmanager.h"
#include "TextMessage.h"

// For DetectionCollector
#include "DetectionResultExporterBase.h"
#include "MarkerBase.h"
#include "MarkerCC2Tracker.h"

class MyPhoneServer : public PhoneServer
{
	/** Number of images captured.
		Incremented upon taking a picture.
		Reported in measurement log.
	*/
	int imageNumber;
	CameraLocalProxy *camProxy;

	class TimeMeasurementCodeDefs
	{
	public:
		// This class may use ID-s >20
		const static int ImageCapture			= 21;
		const static int JpegCompression		= 22;
		const static int ShowImage				= 23;

		static void setnames(TimeMeasurement *measurement)
		{
			measurement->setMeasurementName("CamClient internal time measurements");

			measurement->setname(ImageCapture,"CamClient-ImageCapture");
			measurement->setname(JpegCompression,"CamClient-JpegCompression");
			measurement->setname(ShowImage,"CamClient-ShowImage");
		}
	};

	class DetectionCollector : public TwoColorCircleMarker::DetectionResultExporterBase
	{
	public:
		Vector<Point2d> pointVect;
		// Allows retrieval of current timestamp and
		//	camera transformations
		// Warning! Uses default lastImageTakenTimestamp
		CameraProxy *cameraProxy;

		virtual void writeResult(TwoColorCircleMarker::MarkerBase *marker)
		{
			//Ray ray = cameraProxy->pointImg2World(marker->center);
			long long timestamp = cameraProxy->lastImageTakenTimestamp;
			// Write results to stdout
			cout << "-- New marker" << endl
				 << "- Image coordinates: " << marker->center.x << "/" << marker->center.y << endl
//				 << "- Ray: " << ray << endl
				 << "- Timestamp: " << timestamp << endl
				 << "- IsCenterValid: " << marker->isCenterValid << endl;
			pointVect.push_back(cv::Point2d(marker->center.x,marker->center.y));
		}
		void ShowLocations(Mat *frame)
		{
			for(unsigned int i=0; i<pointVect.size(); i++)
			{
				Point2d p = pointVect[i];
				circle(*frame,p,3,Scalar(255,255,255));
			}
		}
	};

	static const char *imageWindowName;

	DetectionCollector *detectionCollector;
	TwoColorCircleMarker::MarkerCC2Tracker *tracker;

	JsonMessage *createImageMessageFromMat(Mat *image, long long timestamp);

public:
	/** @warning Do not forget to initialize! */
	MyConfigManager configManager;

	MyPhoneServer()
	{
		imageNumber=0;
		TimeMeasurementCodeDefs::setnames(&timeMeasurement);
	}

	~MyPhoneServer()
	{
		if (camProxy)
		{
			delete camProxy;
			camProxy = NULL;
		}
		if (detectionCollector)
		{
			delete detectionCollector;
			detectionCollector = NULL;
		}
	}

	void init(char *inifilename, int argc, char **argv);

	/** Ping callback */
	virtual JsonMessage *PingCallback(PingMessage *msg);

	/** TakePicture callback */
	virtual JsonMessage *TakePictureCallback(TakePictureMessage *msg);

	/** SendPosition callback */
	virtual JsonMessage *SendPositionCallback(SendPositionMessage *msg);

	/** SendLog callback */
	virtual JsonMessage *SendLogCallback(SendlogMessage *msg);

	/** Text callback */
	virtual JsonMessage *TextCallback(TextMessage *textMessage);

};




#endif