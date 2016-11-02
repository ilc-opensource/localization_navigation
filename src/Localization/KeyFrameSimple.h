/*
 * License: 3-clause BSD. See LICENSE file in root directory.
 * Copyright(c) 2015-2016 Intel Corporation. All Rights Reserved.
 * */

#ifndef KEYFRAMESIMPLE_H_
#define KEYFRAMESIMPLE_H_

#include "cCommon.h"

class KeyFrameSimple
{
public:
	KeyFrameSimple() {}
	static void LoadKeyFrameList(std::vector<KeyFrameSimple>&, std::string xml);
	void read(const cv::FileNode&);
	std::string rgbImageName, depthImageName;
	std::vector<int> hashKey;
	Eigen::MatrixXf robotPos;
};

static void read(const cv::FileNode& fn, KeyFrameSimple& kf, const KeyFrameSimple& default_value = KeyFrameSimple())
{
	if (fn.empty())
		kf = default_value;
	else
		kf.read(fn);
}

#endif
