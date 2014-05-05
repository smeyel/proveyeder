#include "MyPhoneServer.h"

// For showing an image with imshow
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "MeasurementLogMessage.h"
#include "MatImageMessage.h"
#include "JpegMessage.h"
#include "StdOutLogger.h"

// Marker detection and tracking
#include "chessboarddetector.h"
#include "camera.h"


const char *MyPhoneServer::imageWindowName = "JPEG";

void MyPhoneServer::init(char *inifilename, int argc, char **argv)
{
	// Init config, allows overrides (including the ini file name)
	configManager.init(inifilename, argc, argv);

	if (configManager.camID>=0)
	{
		camProxy = new CameraLocalProxy(
			configManager.usePs3eye ? VIDEOINPUTTYPE_PS3EYE : VIDEOINPUTTYPE_GENERIC,
			configManager.camID);
	}
	else
	{
		// Use given filename to create file based camera
		camProxy = new CameraLocalProxy(configManager.camSourceFilename.data());
	}
	camProxy->camera->cameraID=0;
	camProxy->camera->isStationary=false;
	camProxy->camera->loadCalibrationData(configManager.camIntrinsicParamsFileName.data());

	// Init logger (singleton)
	Logger *logger = new StdoutLogger();
	//logger->SetLogLevel(Logger::LOGLEVEL_INFO);
	logger->SetLogLevel(Logger::LOGLEVEL_ERROR);
	Logger::getInstance()->Log(Logger::LOGLEVEL_VERBOSE,"CamClient","CamClient started\n");
    time_t t = time(0);   // get time now
    struct tm * now = localtime( & t );
	Logger::getInstance()->Log(Logger::LOGLEVEL_INFO,"CamClient","Current time: %d-%d-%d, %d:%d:%d\n",
		(now->tm_year + 1900),(now->tm_mon + 1),now->tm_mday,now->tm_hour,now->tm_min,now->tm_sec );
	Logger::getInstance()->Log(Logger::LOGLEVEL_VERBOSE,"CamClient","Ini file: %s\n",inifilename);

	// Initialize socket communication and server socket
	InitServer(configManager.serverPort);

	if (configManager.showImage)
	{
		cv::namedWindow(imageWindowName, CV_WINDOW_AUTOSIZE);
	}

	// Init marker detection and tracking
	const Size dsize(640,480);	// TODO: should always correspond to the real frame size!
	detectionCollector = new DetectionCollector();
	tracker = new TwoColorCircleMarker::MarkerCC2Tracker();
	tracker->setResultExporter(detectionCollector);
	tracker->init(inifilename,true,dsize.width,dsize.height);
	detectionCollector->cameraProxy = camProxy;	// Now it can get current timestamp and camera transformations

	blobTracker = new BlobTracker();
	blobTracker->init(camProxy);
}

JsonMessage *MyPhoneServer::PingCallback(PingMessage *msg)
{
	PingMessage *ping = new PingMessage();
	return ping;
}

// w.r.t configManager.sendMatImage, creates new MatImageMessage or JpegMessage from image
JsonMessage *MyPhoneServer::createImageMessageFromMat(Mat *image, long long timestamp)
{
	JsonMessage *answer = NULL;
	if (configManager.sendMatImage)
	{
		// Sending images in openCV Mat format
		MatImageMessage *matAnswer = new MatImageMessage();
		matAnswer->timestamp = timestamp;
		matAnswer->Encode(image);

		matAnswer->log();
		if (configManager.showResponseOnCout)
			cout << "{ \"type\":\"MatImage\", \"timestamp\":\"" << matAnswer->timestamp << "\", \"size\":\"" << matAnswer->size << "\" }#" << endl;

		// Sending the answer and the JPEG encoded picture
		answer = matAnswer;
	}
	else	// Sending JPEG compressed images
	{
		JpegMessage *jpegAnswer = new JpegMessage();
		jpegAnswer->timestamp = timestamp;

		// JPEG compression
		timeMeasurement.start(TimeMeasurementCodeDefs::JpegCompression);
		jpegAnswer->Encode(image);
		timeMeasurement.finish(TimeMeasurementCodeDefs::JpegCompression);

		// Assembly of the answer
		if (configManager.showResponseOnCout)
			cout << "{ \"type\":\"JPEG\", \"timestamp\":\"" << jpegAnswer->timestamp << "\", \"size\":\"" << jpegAnswer->size << "\" }#" << endl;

		// Sending the answer and the JPEG encoded picture
		answer = jpegAnswer;

		// Showing the image after sending it, so that it causes smaller delay...
		if (configManager.showImage)
		{
			timeMeasurement.start(TimeMeasurementCodeDefs::ShowImage);
			cv::Mat show = cv::imdecode(cv::Mat(jpegAnswer->data),CV_LOAD_IMAGE_COLOR); 
			cv::imshow(imageWindowName,show);
			int key = cv::waitKey(25);	// TODO: needs some kind of delay to show!!!
			timeMeasurement.finish(TimeMeasurementCodeDefs::ShowImage);
		}
	}
	return answer;
}


JsonMessage *MyPhoneServer::TakePictureCallback(TakePictureMessage *msg)
{
	timeMeasurement.start(TimeMeasurementCodeDefs::ImageCapture);
	bool isSuccessful = camProxy->CaptureImage(msg->desiredtimestamp);
	//detectionCollector->ShowLocations(camProxy->lastImageTaken);	// ADDS INDICATORS!
	timeMeasurement.finish(TimeMeasurementCodeDefs::ImageCapture);

	if (!isSuccessful)
	{
		// Send message that there are no more frames
		// (Possibly because an AVI file is used as input and we have reached its end.)
		TextMessage *errorMsg = new TextMessage();
		errorMsg->copyToContent("Error: no more input images");
		return errorMsg;
	}

	JsonMessage *answer = createImageMessageFromMat(
		camProxy->lastImageTaken,
		camProxy->lastImageTakenTimestamp);

	imageNumber++;
	return answer;
}

JsonMessage *MyPhoneServer::SendPositionCallback(SendPositionMessage *msg)
{
	// Take picture
	timeMeasurement.start(TimeMeasurementCodeDefs::ImageCapture);
	bool isSuccessful = camProxy->CaptureImage(msg->desiredtimestamp);
	timeMeasurement.finish(TimeMeasurementCodeDefs::ImageCapture);

	if (!isSuccessful)
	{
		// Send message that there are no more frames
		// (Possibly because an AVI file is used as input and we have reached its end.)
		TextMessage *errorMsg = new TextMessage();
		errorMsg->copyToContent("Error: no more input images");
		return errorMsg;
	}

	// Detect marker
	detectionCollector->pointVect.clear();
	tracker->processFrame(*(camProxy->lastImageTaken),camProxy->camera->cameraID,(float)imageNumber);
	/*	if (camProxy->camera->getIsTSet())
	{
		tracker->processFrame(*(camProxy->lastImageTaken),camProxy->camera->cameraID,imageNumber);
	}
	else
	{
		cout << "ERROR: Cannot track marker! Camera not calibrated!" << endl;
		Logger::getInstance()->Log(Logger::LOGLEVEL_ERROR,"CamClient","Cannot execute SendPosition, camera not calibrated!");
	}*/
	//detectionCollector->ShowLocations(camProxy->lastImageTaken);	// ADDS INDICATORS!

	// Assemble position response (overriding send procedure, increases handle time with communication overhead!)
	TextMessage *textMsg = new TextMessage();
	if (detectionCollector->pointVect.size()>0)
	{
		char buff[100];
		Point2d first = detectionCollector->pointVect[0];
		sprintf(buff,"n=%d, first x=%lf y=%lf",
			detectionCollector->pointVect.size(),
			first.x, first.y);
		textMsg->copyToContent(buff);
	}
	else
	{
		textMsg->copyToContent("NONE");
	}
	Send(textMsg);

	delete textMsg;
	textMsg = NULL;

	// Sending image if asked for
	JsonMessage *answer = NULL;
	if (msg->sendImage)
	{
		JsonMessage *answer = createImageMessageFromMat(
			camProxy->lastImageTaken,
			camProxy->lastImageTakenTimestamp);
		Send(answer);
	}

	imageNumber++;
	return answer;
}

JsonMessage *MyPhoneServer::SendLogCallback(SendlogMessage *msg)
{
	MeasurementLogMessage *answer = new MeasurementLogMessage();

	// Create the log
	std::ostringstream measurementLog;
	time_t t = time(0);   // get time now
    struct tm * now = localtime( & t );
	measurementLog << "--- New results at " << (now->tm_year + 1900) << "-" << (now->tm_mon + 1) << "-" << now->tm_mday
		<< " " << now->tm_hour << ":" << now->tm_min << ":" << now->tm_sec << endl;
	measurementLog << "Log file: " << configManager.logFileName << endl;
	measurementLog << "--- Main loop time measurement results:" << endl;
	timeMeasurement.showresults(&measurementLog);
	measurementLog << "--- Further details:" << endl;
	measurementLog << "Number of captured images: " << imageNumber << endl;
	measurementLog << "--- End of results" << endl;
	std::string measurementLogString = measurementLog.str();

	// Send
	answer->timestamp = timeMeasurement.getTimeStamp();
	answer->size = measurementLogString.length();
	const char *ptr = measurementLogString.c_str();
	for(int i=0; i<answer->size; i++)
	{
		answer->data.push_back(ptr[i]);
	}
	return answer;
}

JsonMessage *MyPhoneServer::TextCallback(TextMessage *textMessage)
{
	cout << "Received TEXT message with text=" << textMessage->content << endl;
	if (!strcmp(textMessage->content,"CALIBRATE"))
	{
		cout << "Calibration command received... trying for 50 frames..." << endl;
		if (camProxy->CaptureUntilCalibrated(50))
		{
			cout << "Calibration successful." << endl;
			textMessage->copyToContent("OK");
		}
		else
		{
			cout << "Calibration FAILED." << endl;
			textMessage->copyToContent("FAILED");
		}
	}
	else if (!strcmp(textMessage->content,"DETECT"))
	{
		cout << "Marker detection command received..." << endl;
		camProxy->CaptureImage(0);
		tracker->processFrame(*(camProxy->lastImageTaken),camProxy->camera->cameraID,0.0);
	}
	else if (!strcmp(textMessage->content,"QUIT"))
	{
		cout << "QUIT command received..." << endl;
		Disconnect();
		exit(0);
	}

	return textMessage;
}

JsonMessage *MyPhoneServer::PropertyCallback(PropertyMessage *propertyMessage)
{
	cout << "Received PROPERTY message: key=" << propertyMessage->key << ", value=" << propertyMessage->value << endl;
	if (!strcmp(propertyMessage->key,"gain"))
	{
		camProxy->SetNormalizedGain(propertyMessage->getIntValue());
	}
	else if (!strcmp(propertyMessage->key,"exposure"))
	{
		camProxy->SetNormalizedExposure(propertyMessage->getIntValue());
	}
	else if (!strcmp(propertyMessage->key,"whitebalance"))
	{
		int r,g,b;
		sscanf(propertyMessage->value,"%d/%d/%d",&r,&g,&b);
		camProxy->SetNormalizedWhiteBalance(r,g,b);
	}
	else if (!strcmp(propertyMessage->key, "blobcolor"))
	{
		int h,s,v;
		sscanf(propertyMessage->value, "%d/%d/%d", &h, &s, &v);
		blobTracker->setColorBound(Vec3d(h, s, v));
	}

	return NULL;
}



JsonMessage *MyPhoneServer::StreamPositionCallback(StreamPositionMessage *streamPositionMessage)
{
	if (!camProxy->camera->getIsTSet())
	{
		TextMessage *tm = new TextMessage();
		tm->copyToContent("You have to calibrate your camera!");
		Send(tm);
		delete tm;
		tm = NULL;

		return NULL;
	}

	if (streamPositionMessage->streaming == 1)
	{
		SOCKET sock = getSocket();
		HANDLE thread = CreateThread(NULL, 0, StaticStreamThreadStart, (void*)sock, 0, NULL);
		BlobTracker::getInstance()->streaming = 1;
	}
	else
	{
		BlobTracker::getInstance()->streaming = 0;
	}
	
	return NULL;
}

DWORD MyPhoneServer::StaticStreamThreadStart(void* param)
{
	SOCKET mySock = (SOCKET)param;

	while (BlobTracker::getInstance()->streaming == 1)
	{
		bool isSuccessful = BlobTracker::getInstance()->camProxy->CaptureImage(0);

		if (!isSuccessful)
		{
			TextMessage *errorMsg = new TextMessage();
			errorMsg->copyToContent("Error: no more input images");
			return 0;
		}

		Point p = BlobTracker::getInstance()->processFrame(*(BlobTracker::getInstance()->camProxy->lastImageTaken));

		if (&p != NULL)
		{
			Ray r = BlobTracker::getInstance()->getBlobDirection(p);
			r.show("");

			TextMessage *rayData = new TextMessage();

			char buff[100];
			sprintf(buff, "A:%lf, %lf, %lf, %lf, B:%lf, %lf, %lf, %lf",
				r.A.val[0], r.A.val[1], r.A.val[2], r.A.val[3],
				r.B.val[0], r.B.val[1], r.B.val[2], r.B.val[3]);
			rayData->copyToContent(buff);

			char buffer[200];
			rayData->writeJson(buffer);
			int len = strlen(buffer);

			PlatformSpecifics::getInstance()->send(mySock, buffer, len, 0);
			rayData->writeAux(mySock);
		}
	}

	return 0;
}