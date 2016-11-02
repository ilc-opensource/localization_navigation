/*
 * License: 3-clause BSD. See LICENSE file in root directory.
 * Copyright(c) 2015-2016 Intel Corporation. All Rights Reserved.
 * */

#include <iostream>
#include "cORBExtractor.h"

using namespace std;
using namespace cv;

cORBExtractor::cORBExtractor(Mat &image)
{
	MAX_POINTS = 4096;

	Ptr<ORB> orb = ORB::create(MAX_POINTS);
	orb->detect(image, keypoints);
	if (keypoints.size() < 1)
	{
		cout << "No keypoint detected!" << endl;
		return;
	}
	orb->compute(image, keypoints, descriptor);
}

cORBExtractor::cORBExtractor()
{
	MAX_POINTS = 4096;
}

cORBExtractor::~cORBExtractor()
{
	keypoints.clear();
	descriptor.release();
}

void cORBExtractor::run(Mat &image)
{
	keypoints.clear();
	descriptor.release();

	Ptr<ORB> orb = ORB::create(MAX_POINTS);
	Ptr<FastFeatureDetector> fast = FastFeatureDetector::create(10);
	fast->detect(image, keypoints);
	if (keypoints.size() < 1)
	{
		cout << "No keypoint detected!" << endl;
		return;
	}
	orb->compute(image, keypoints, descriptor);
}
