/*
 * License: 3-clause BSD. See LICENSE file in root directory.
 * Copyright(c) 2015-2016 Intel Corporation. All Rights Reserved.
 * */

#ifndef CVISUALIZATION_H_
#define CVISUALIZATION_H_

#include "cBasic.h"
#include <string>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>

class cVisualization
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cVisualization(std::string ModelPath, std::string campath);

	void Show();
	void Show(const Eigen::Matrix4f &camPose);
	void Show(const Eigen::Matrix4f &camPose, const Eigen::Matrix4f &KeyframePose);
	void Save(cv::Mat rgb1, cv::Mat depth1, cv::Mat rgb2, cv::Mat depth2, std::string savepath, const Eigen::Matrix4f &TMatrix);
	bool gen2DMap(std::string filename);

protected:
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr image2PointCloud(cv::Mat rgb, cv::Mat depth);

private:
	/*pcl::visualization::PCLVisualizer mViewer;*/
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr mObjectModel;
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr mCamModel;
	pcl::PolygonMesh mCamMesh;
	Eigen::Matrix4f mCamPose;
	Eigen::Matrix4f mKeyframePose;
};

#endif
