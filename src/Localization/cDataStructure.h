/*
 * License: 3-clause BSD. See LICENSE file in root directory.
 * Copyright(c) 2015-2016 Intel Corporation. All Rights Reserved.
 * */

#ifndef CDATASTRUCTURE_H_
#define CDATASTRUCTURE_H_

#include <string>
// localiztion output data structure;
struct cItem
{
	double score = -1; // score from vocabulary tree
	int index = -1; // index of keyframe candidate
	int inlier = -1; // inlier number between query image and keyframe candidate
	int orbMatches = -1; //matched ORB feature number between query image and keyframe candidate
	int validMatches = -1; //siftmatches with depth value. if depth value < 0 or depth value > 3.0, it is invalid
	std::string imageName = ""; // the name of keyfame candidate
	float relativePose[4][4]; // relative pose between query image and keyframe candidate
	float absolutePose[4][4]; // absolute pose of query image
	bool isFound = false; // is the query image localized?
};

#endif
