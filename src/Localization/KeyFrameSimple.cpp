/*
 * License: 3-clause BSD. See LICENSE file in root directory.
 * Copyright(c) 2015-2016 Intel Corporation. All Rights Reserved.
 * */

#include "KeyFrameSimple.h"

void KeyFrameSimple::LoadKeyFrameList(std::vector<KeyFrameSimple> &keyFrameList, std::string xml)
{
	cv::FileStorage fs(xml, cv::FileStorage::READ);
	int numberOfKeyFrame;
	fs["NumberOfKeyFrame"] >> numberOfKeyFrame;
	keyFrameList.clear();
	keyFrameList.resize(numberOfKeyFrame);
	cv::FileNode fn = fs["KeyFrameList"];
	int i = 0;
	for (auto it = fn.begin(); it != fn.end() && i < numberOfKeyFrame; it++, i++)
	{
		*it >> keyFrameList[i];
	}
}

void KeyFrameSimple::read(const cv::FileNode &fn)
{
	fn["RGB"] >> rgbImageName;
	fn["Depth"] >> depthImageName;
	cv::Mat pos;
	fn["RobotPos"] >> pos;
	robotPos = Eigen::Matrix4f::Identity();
	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++)
			robotPos(i, j) = pos.at<float>(i, j);
	cv::FileNode node = fn["HashKey"];
	hashKey.clear();
	for (auto it = node.begin(); it != node.end(); it++)
		hashKey.push_back(*it);
}
