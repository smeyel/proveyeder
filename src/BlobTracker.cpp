#include "BlobTracker.h"
#include <Windows.h>
// For showing an image with imshow
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "JpegMessage.h"

// Marker detection and tracking
#include "chessboarddetector.h"
#include "camera.h"

BlobTracker* BlobTracker::instance = NULL;

BlobTracker::BlobTracker()
{
	instance = this;
}

void BlobTracker::init(CameraProxy* _camProxy)
{
	camProxy = _camProxy;
	Size cameraResolution = camProxy->camera->cameraResolution;
	hsvImg = new Mat(cameraResolution.height, cameraResolution.width, CV_8UC3);

	mLowerBound = Vec<double, 4>();
	mUpperBound = Vec<double, 4>();
}

Point BlobTracker::processFrame(Mat& img)
{
	vector< vector<Point> > mContours;
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	
	cvtColor(img, *hsvImg, CV_BGR2HSV_FULL);

	inRange(*hsvImg, mLowerBound, mUpperBound, *hsvImg);
	dilate(*hsvImg, *hsvImg, Mat());
	findContours(*hsvImg, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

	int maxContour = 0;
	double maxArea = 0;

	for (int i = 0; i < contours.size(); i++)
	{
		double area = contourArea(contours.at(i));
		if (area > maxArea)
		{
			maxArea = area;
			maxContour = i;
		}
	}

	if (contours.size() > 0)
	{
		Moments mu = moments(contours.at(maxContour));
		Point blobCenter = Point(mu.m10 / mu.m00, mu.m01 / mu.m00);
		vector<Point> contourPoints = contours.at(maxContour);
		bool blobIsCircle = isCircle(contourPoints, blobCenter);

		if (blobIsCircle)
		{
			return blobCenter;
		}
		else
		{
			return NULL;
		}
	}
	else
	{
		return NULL;
	}

}

void BlobTracker::setColorBound(Vec3d hsv)
{
	Vec<int, 4> mColorRadius = Vec<int, 4>(10, 50, 70, 0);

	double minH = (hsv[0] >= mColorRadius.val[0]) ? hsv[0] - mColorRadius.val[0] : 0;
	double maxH = (hsv[0] + mColorRadius.val[0] <= 255) ? hsv[0] + mColorRadius.val[0] : 255;

	mLowerBound.val[0] = minH;
	mUpperBound.val[0] = maxH;

	mLowerBound.val[1] = hsv[1] - mColorRadius.val[1];
	mUpperBound.val[1] = hsv[1] + mColorRadius.val[1];

	mLowerBound.val[2] = hsv[2] - mColorRadius.val[2];
	mUpperBound.val[2] = hsv[2] + mColorRadius.val[2];

	mLowerBound.val[3] = 0;
	mUpperBound.val[3] = 255;
}

Ray BlobTracker::getBlobDirection(Point blobCenter)
{
	return camProxy->camera->pointImg2World(blobCenter);
}

bool BlobTracker::isCircle(vector<Point> contour, Point blobCenter)
{
	double minDistance;
	double epsilon = 0.0;

	vector<double> contourDistancesFromCenter = vector<double>();

	for (int j = 0; j < contour.size(); j++)
	{
		Point contourPoint = contour.at(j);

		double a = ((contourPoint.x - blobCenter.x)*(contourPoint.x - blobCenter.x));
		double b = ((contourPoint.y - blobCenter.y)*(contourPoint.y - blobCenter.y));
		double currentDistance = sqrt(a + b);

		contourDistancesFromCenter.push_back(currentDistance);

		if (j == 0)
			minDistance = currentDistance;
		else
		{
			if (minDistance > currentDistance)
				minDistance = currentDistance;
		}
	}

	epsilon = (2.0 * minDistance) / 3.0;

	for (int j = 0; j < contourDistancesFromCenter.size(); j++)
	{
		if (!((contourDistancesFromCenter.at(j) + epsilon > minDistance) && (contourDistancesFromCenter.at(j) - epsilon < minDistance)))
		{
			// this blob is not a circle
			return false;
		}
	}

	return true;
}

BlobTracker* BlobTracker::getInstance()
{
	return instance;
}