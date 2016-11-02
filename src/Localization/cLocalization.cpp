/*
 * License: 3-clause BSD. See LICENSE file in root directory.
 * Copyright(c) 2015-2016 Intel Corporation. All Rights Reserved.
 * */

#include "cLocalization.h"

#include "cBasic.h"

#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/core/sparse_optimizer.h>

using namespace std;
using namespace cv;


void cLocalization::LoadKeyFrameList(string xmlPath)
{
	g2o::SparseOptimizer *optimizer = new g2o::SparseOptimizer;
	optimizer->load(g2oPath.c_str());

	KeyFrameSimple::LoadKeyFrameList(keyframeSimpleList, xmlPath);

	for (auto it = keyframeSimpleList.begin(); it != keyframeSimpleList.end(); it++)
	{
		g2o::OptimizableGraph::Vertex* v = optimizer->vertex(it - keyframeSimpleList.begin());
		if (v == NULL) continue;
		g2o::Vector7d _estimate;
		v->getEstimateData(&_estimate(0));
		Eigen::Isometry3d i3d = g2o::internal::fromVectorQT(_estimate);
		Eigen::Matrix4f tmpPose = cBasic<>::Isometry3dToMatrixXf(i3d);
		it->robotPos = keyframeSimpleList.begin()->robotPos.inverse()*tmpPose;
	}
	delete optimizer;
}

cLocalization::cLocalization(string g2o, string xmlPath)
{
	g2oPath = g2o;
	LoadKeyFrameList(xmlPath);
}

cLocalization::~cLocalization()
{
}

Eigen::Matrix4f cLocalization::GetAbsoluteCamPose(string candidateName, const Eigen::Matrix4f &relativePose, float camScale)
{
	Eigen::Matrix4f absolutePose = Eigen::Matrix4f::Identity();
	for (auto it = keyframeSimpleList.begin(); it != keyframeSimpleList.end(); it++)
	{
		if (it->rgbImageName == candidateName)
		{
			absolutePose = it->robotPos*relativePose;
			if (camScale != 1.0)
			{
				for (int i = 0; i < 3;i++)
				for (int j = 0; j < 3; j++)
					absolutePose(i, j) = absolutePose(i, j)*camScale;
			}
			return absolutePose;
		}
	}
	return absolutePose;
}

Eigen::Matrix4f cLocalization::GetKeyframePose(string candidateName, float camScale)
{
	Eigen::Matrix4f Pose = Eigen::Matrix4f::Identity();
	for (auto it = keyframeSimpleList.begin(); it != keyframeSimpleList.end(); it++)
	{
		if (it->rgbImageName == candidateName)
		{
			Pose = it->robotPos;
			if (camScale != 1.0)
			{
				for (int i = 0; i < 3; i++)
				for (int j = 0; j < 3; j++)
					Pose(i, j) = Pose(i, j)*camScale;
			}

			return Pose;
		}
	}
	return Pose;
}
