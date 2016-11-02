/*
 * License: 3-clause BSD. See LICENSE file in root directory.
 * Copyright(c) 2015-2016 Intel Corporation. All Rights Reserved.
 * */

#ifndef CBASIC_H_
#define CBASIC_H_
#include "cCommon.h"


const float zMax = 3.0;
const float mCam_cx = 319.5;
const float mCam_cy = 239.5;
const float mCam_fx = 620.0;
const float mCam_fy = 620.0;
const float mCam_scale = 1000;

template <typename TPoint = pcl::PointXYZRGBA>
class cBasic
{
public:
	cBasic()
	{

	}
	//convert TPoint to RowVectorXf format
	static Eigen::RowVectorXf PointToRowVector(TPoint& point)
	{
		Eigen::RowVectorXf result(3);
		result << point.x, point.y, point.z;
		return result;
	}
	//convert Isometry3d to MatrixXf format
	static Eigen::MatrixXf Isometry3dToMatrixXf(Eigen::Isometry3d& transformation)
	{
		Eigen::MatrixXf T(4, 4);
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				T(i, j) = transformation(i, j);
			}
		}
		T.block<1, 4>(3, 0) << 0, 0, 0, 1;
		return T;
	}
private:

};


#endif
