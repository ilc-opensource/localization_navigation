/*
 * License: 3-clause BSD. See LICENSE file in root directory.
 * Copyright(c) 2015-2016 Intel Corporation. All Rights Reserved.
 * */

#include <Eigen/Dense>
#include <vector>
#include "opencv2/opencv.hpp"
#include "cCommon.h"
class cPose
{
private:
	// 待求解的帧
	cv::Mat query_rgb;
	cv::Mat query_depth;
	// keyframe
	cv::Mat key_rgb;
	cv::Mat key_depth;
	// pose
	cv::Mat P;
	Eigen::MatrixXf TMatrix;
	int inlier;
	// matcher
	float max_ratio;

	cv::Mat mIntrinsic;

	int validMatches;
	int orbMatches;

	pcl::PointXYZ Point2DTo3D(float ptx, float pty, uint16_t idensity);

public:
	cPose();
	~cPose();

public:
	// 设置query图像
	void setQueryFrame(cv::Mat &_query_rgb, cv::Mat &_query_depth);
	// 设置keyframe
	void setKeyFrame(cv::Mat &_key_rgb, cv::Mat &_key_depth);
	// 求解pnp，使用ransac
	void ransacPnP();
	// 直接利用特征点
	void ransacPnP(std::vector<cv::KeyPoint> &kp1, cv::Mat &desc1, std::vector<cv::KeyPoint> &kp2, cv::Mat &desc2);
	// 设置内参
	void setIntrinsic(float fx, float fy, float u0, float v0);
	//
	int getInlierNum();
	int getMatchNum();
	int getValidMatch();

	//
	cv::Mat getPose();
	Eigen::MatrixXf getTMatrix();
};
