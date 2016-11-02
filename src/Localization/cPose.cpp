/*
 * License: 3-clause BSD. See LICENSE file in root directory.
 * Copyright(c) 2015-2016 Intel Corporation. All Rights Reserved.
 * */

#include "cPose.h"
#include "cORBExtractor.h"
#include "cBasic.h"


#define BRUTEFORCEMATE
#ifdef BRUTEFORCEMATE
#include "opencv2/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"

#endif

using namespace std;
using namespace cv;

cPose::cPose()
{
	max_ratio = 0.9;
	TMatrix = Eigen::Matrix4f::Identity();
}

cPose::~cPose()
{
}

void cPose::setKeyFrame(Mat &_key_rgb, Mat &_key_depth)
{
	if (!_key_depth.data || !_key_depth.data)
		return;

	key_rgb = _key_rgb;
	key_depth = _key_depth;
}

void cPose::setQueryFrame(Mat& _query_rgb, Mat& _query_depth)
{
	if (!_query_rgb.data || !_query_depth.data)
		return;

	query_rgb = _query_rgb;
	query_depth = _query_depth;
}

void cPose::setIntrinsic(float fx, float fy, float u0, float v0)
{
	mIntrinsic.release();

	mIntrinsic.create(Size(3, 3), CV_64FC1);
	for (int i = 0; i < 3;i++)
		for (int j = 0; j < 3; j++)
			mIntrinsic.at<double>(i, j) = 0.0;

	mIntrinsic.at<double>(0, 0) = fx;
	mIntrinsic.at<double>(1, 1) = fy;
	mIntrinsic.at<double>(0, 2) = u0;
	mIntrinsic.at<double>(1, 2) = v0;
	mIntrinsic.at<double>(2, 2) = 1.0;
}

pcl::PointXYZ cPose::Point2DTo3D(float ptx, float pty, uint16_t idensity)
{
	pcl::PointXYZ pt;
	pt.z = idensity / 1000.0f;
	pt.x = (ptx - mIntrinsic.at<double>(0, 2)) * pt.z / mIntrinsic.at<double>(0, 0);
	pt.y = (pty - mIntrinsic.at<double>(1, 2)) * pt.z / mIntrinsic.at<double>(1, 1);
	return pt;
}

int cPose::getInlierNum()
{
	return inlier;
}

int cPose::getMatchNum()
{
	return orbMatches;
}

int cPose::getValidMatch()
{
	return validMatches;
}

Mat cPose::getPose()
{
	return P;
}

Eigen::MatrixXf cPose::getTMatrix()
{
	return TMatrix;
}

void cPose::ransacPnP()
{
	cORBExtractor feature1(query_rgb);
	cORBExtractor feature2(key_rgb);

	ransacPnP(feature1.keypoints, feature1.descriptor, feature2.keypoints, feature2.descriptor);
}

void cPose::ransacPnP(vector<KeyPoint> &kp1, Mat &desc1, vector<KeyPoint> &kp2, Mat &desc2)
{
#ifndef BRUTEFORCEMATE
	desc1.convertTo(desc1, CV_32F);
	desc2.convertTo(desc2, CV_32F);
	FlannBasedMatcher matcher;
	vector<vector<DMatch>> knnMatches;
	vector<DMatch> matches;

	// query和keyframe之间的特征匹配
	matcher.add(vector<Mat>(1, desc2));
	matcher.train();
	matcher.knnMatch(desc1, knnMatches, 2);
	for (auto it = knnMatches.begin(); it != knnMatches.end(); ++it)
	{
		if (it->size() == 0)
			continue;

		DMatch bestMatch = (*it)[0];
		DMatch betterMatch = (*it)[1];

		float ratio = bestMatch.distance / betterMatch.distance;
		if (ratio < max_ratio)
		{
			matches.push_back(bestMatch);
		}
	}
#else	
	Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");

	vector<DMatch> matchesRaw;
	vector<DMatch > matches;
	matcher->match(desc1, desc2, matchesRaw);
	double max_dist = 0; double min_dist = 100;
	//-- Quick calculation of max and min distances between keypoints
	for (int i = 0; i < matchesRaw.size(); i++)
	{
		double dist = matchesRaw[i].distance;
		if (dist < min_dist) min_dist = dist;
		if (dist > max_dist) max_dist = dist;
	}
	//printf("-- Max dist : %f \n", max_dist);
	//printf("-- Min dist : %f \n", min_dist);
	for (int i = 0; i < matchesRaw.size(); i++)
	{
		if (matchesRaw[i].distance < 0.4*max_dist)
		{
			matches.push_back(matchesRaw[i]);
		}
	}
#endif
	
	
	
	orbMatches = matches.size();

	vector<Point2f> point2ds;
	vector<Point3f> point3ds;

	for (auto it = matches.begin(); it != matches.end();)
	{
		int trainId = it->trainIdx;
		int queryId = it->queryIdx;
		unsigned short idensity = key_depth.at<unsigned short>(kp2[trainId].pt.y, kp2[trainId].pt.x);
		pcl::PointXYZ ptSrc = Point2DTo3D(kp2[trainId].pt.x, kp2[trainId].pt.y, idensity);
		Point3f tmp_3D;
		tmp_3D.x = ptSrc.x; tmp_3D.y = ptSrc.y; tmp_3D.z = ptSrc.z;
		Point2f tmp_2D;
		tmp_2D.x = kp1[queryId].pt.x; tmp_2D.y = kp1[queryId].pt.y;
		if (tmp_3D.z<0.0000001 || tmp_3D.z>10)
		{
			it = matches.erase(it);
			continue;
		}
		else
			it++;
		point2ds.push_back(tmp_2D);
		point3ds.push_back(tmp_3D);
	}
	validMatches = matches.size();
	double dist[4] = { 0, 0, 0, 0 };
	Mat distCoffes(1, 4, CV_64FC1, dist);
	Mat rvec(3, 1, CV_64FC1);
	Mat tvec(3, 1, CV_64FC1);
	Mat Rout(3, 3, CV_64FC1);
	vector<int> inliers;
	if (point2ds.size() <= 3 || point3ds.size() <= 3)
	{
		inlier = 0;
	}
	else
	{
		solvePnPRansac(point3ds, point2ds, mIntrinsic, distCoffes, rvec, tvec, false, 100, 8.0f, 0.99f, inliers, CV_EPNP);
	}
	Rodrigues(rvec, Rout);
	P.release();
	P.create(Size(4, 4), CV_64FC1);

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			TMatrix(i, j) = Rout.at<double>(i, j);
		}
	}

	TMatrix(0, 3) = tvec.at<double>(0, 0);
	TMatrix(1, 3) = tvec.at<double>(1, 0);
	TMatrix(2, 3) = tvec.at<double>(2, 0);
	TMatrix(3, 0) = 0;
	TMatrix(3, 1) = 0;
	TMatrix(3, 2) = 0;
	TMatrix(3, 3) = 1;

	TMatrix = TMatrix.inverse();

	// get inliers
	inlier = inliers.size();
}
