/*
 * License: 3-clause BSD. See LICENSE file in root directory.
 * Copyright(c) 2015-2016 Intel Corporation. All Rights Reserved.
 * */

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <vector>


class cORBExtractor {
public:
	std::vector<cv::KeyPoint> keypoints;
	cv::Mat descriptor;
	int MAX_POINTS;

public:
	cORBExtractor(cv::Mat &image);
	cORBExtractor();
	~cORBExtractor();

	void run(cv::Mat &image);
};
