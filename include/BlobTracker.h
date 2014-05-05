#ifndef __BLOBTRACKER_H
#define __BLOBTRACKER_H

#include <fstream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <stdlib.h>
#include "CameraProxy.h"


using namespace cv;
using namespace std;

class BlobTracker
{
public:
	static BlobTracker *instance;
	static BlobTracker *getInstance(void);

	CameraProxy* camProxy;
	Mat* hsvImg;

	Vec<double, 4> mLowerBound;
	Vec<double, 4> mUpperBound;

	int streaming;

	BlobTracker();

	void init(CameraProxy* camproxy);
	Point processFrame(Mat& src);
	void setColorBound(Vec3d hsv);
	Ray getBlobDirection(Point blobCenter);
	bool isCircle(vector<Point> contour, Point blobCenter);
};


#endif