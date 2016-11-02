/*
 * License: 3-clause BSD. See LICENSE file in root directory.
 * Copyright(c) 2015-2016 Intel Corporation. All Rights Reserved.
 * */

#ifndef RANSAC_H_
#define RANSAC_H_

#include "cCommon.h"
#include "cTransformationType.h"
#include "cBasic.h"
#include <pcl/common/transforms.h>

// this file is written by huwei, please ask him for explanation.
template <typename TPoint = pcl::PointXYZRGBA>
class cRansac
{
public:
	cRansac(typename pcl::PointCloud<TPoint>::Ptr pointCloud1,
		typename pcl::PointCloud<TPoint>::Ptr pointCloud2,
		TransformationType::TransFormat type = TransformationType::rigid,
		float _threshold = 0.05, float _confidence = 0.99, int _maxIterNum = 1024)
		:maxIterNum(_maxIterNum), confidence(_confidence), threshold(_threshold)
	{
		switch (type)
		{
		case TransformationType::rigid:
			ransacMain(pointCloud1, pointCloud2, TransformationType::rigidEval<TPoint>, 3);
			break;
		}
	}

	void operator()(typename pcl::PointCloud<TPoint>::Ptr pointCloud1,
		typename pcl::PointCloud<TPoint>::Ptr pointCloud2,
		TransformationType::TransFormat type = TransformationType::rigid,
		float _threshold = 0.05, float _confidence = 0.99, int _maxIterNum = 1024) const
	{
		switch (type)
		{
		case TransformationType::rigid:
			ransacMain(pointCloud1, pointCloud2, TransformationType::rigidEval<TPoint>, 3);
			break;
		}
	}

	Eigen::MatrixXf GetTransforMat()
	{
		return transforMat;
	}

	vector<int> GetInliersIndex()
	{
		return inliersIndex;
	}

	int GetInliersNum()
	{
		return inliersNum;
	}

private:
	float confidence, threshold;
	int maxIterNum, inliersNum;
	Eigen::MatrixXf transforMat;
	vector<int> inliersIndex;
	using funcEval = void(*)(typename pcl::PointCloud<TPoint>::Ptr, const vector<int>&,
		typename pcl::PointCloud<TPoint>::Ptr, Eigen::MatrixXf&);

	void ransacMain(typename pcl::PointCloud<TPoint>::Ptr pointCloud1,
		typename pcl::PointCloud<TPoint>::Ptr pointCloud2, funcEval func, int sampleNum)
	{
		inliersNum = 0;
		inliersIndex.clear();
		if (pointCloud1->size() < sampleNum || pointCloud2->size() < sampleNum)
		{
			//cerr << "no enough points for ransac!" << endl;
			transforMat = Eigen::Matrix4f::Identity();
			return;
		}
		int iterNum = 0;
		cv::RNG rng;
		while (iterNum <= maxIterNum)
		{
			// 生成随机抽样点
			vector<int> sample;
			for (int i = 0; i < sampleNum; i++)
			{
				int ind = rng.uniform(0.0, (double)(*pointCloud1).points.size());
				while (find(sample.begin(), sample.end(), ind) != sample.end())
				{
					ind = rng.uniform(0.0, (double)(*pointCloud1).points.size());
				}
				sample.push_back(ind);
			}
			Eigen::MatrixXf transTemp;
			func(pointCloud1, sample, pointCloud2, transTemp);

			// 计算内点概率
			int inliersNumTemp = 0;
			vector<int> inlierIndexTemp;
			typename pcl::PointCloud<TPoint>::Ptr pointCloudTemp(new pcl::PointCloud<TPoint>);
			pcl::transformPointCloud(*pointCloud1, *pointCloudTemp, transTemp);
			for (int i = 0; i < (*pointCloud1).points.size(); i++)
			{
				Eigen::RowVectorXf v1 = cBasic<TPoint>::PointToRowVector((*pointCloudTemp).points[i]),
					v2 = cBasic<TPoint>::PointToRowVector((*pointCloud2).points[i]);
				if ((v2 - v1).norm() < threshold)
				{
					inliersNumTemp++;
					inlierIndexTemp.push_back(i);
				}
			}

			if (inliersNumTemp>inliersNum)
			{
				inliersIndex.assign(inlierIndexTemp.begin(), inlierIndexTemp.end());
				inliersNum = inliersNumTemp;
				float inliersRate = (float)inliersNum / pointCloud1->points.size();
				maxIterNum = (abs(log(1 - confidence) / log(1 - pow(inliersRate, sampleNum))) > maxIterNum) ?
				maxIterNum : abs(log(1 - confidence) / log(1 - pow(inliersRate, sampleNum)));
			}
			iterNum++;
		}

		func(pointCloud1, inliersIndex, pointCloud2, transforMat);
	}

};
#endif
