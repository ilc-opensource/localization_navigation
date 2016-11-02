/*
 * License: 3-clause BSD. See LICENSE file in root directory.
 * Copyright(c) 2015-2016 Intel Corporation. All Rights Reserved.
 * */

#include <pcl/pcl_config.h>
#if PCL_VERSION_COMPARE(<, 1, 7, 2)
#include <pcl/io/vtk_lib_io.h>
#endif
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include "cVisualization.h"

using namespace std;
using namespace cv;

cVisualization::cVisualization(string modelpath, string campath)
{
	mCamPose = Eigen::Matrix4f::Identity();
	mKeyframePose = Eigen::Matrix4f::Identity();
	mCamModel.reset(new pcl::PointCloud<pcl::PointXYZRGBA>());
	mObjectModel.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);

	pcl::io::loadPLYFile<pcl::PointXYZRGBA>(modelpath, *mObjectModel);
#if PCL_VERSION_COMPARE(>=, 1, 7, 2)
	pcl::io::loadPLYFile(campath, mCamMesh);
#else
	pcl::io::loadPolygonFile(campath, mCamMesh);
#endif
	pcl::fromPCLPointCloud2(mCamMesh.cloud, *mCamModel);
}

void cVisualization::Show()
{
	pcl::visualization::PCLVisualizer mViewer = pcl::visualization::PCLVisualizer("3D Viewer");

	mViewer.setBackgroundColor(0, 0, 0);
	mViewer.addCoordinateSystem(2.0, 0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(mObjectModel);
	mViewer.addPointCloud<pcl::PointXYZRGBA>(mObjectModel, rgb, "sample");
	mViewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample");

	while (!mViewer.wasStopped())
	{
		mViewer.spinOnce(100);
	}
}

void cVisualization::Show(const Eigen::Matrix4f &camPose)
{
	mCamPose = camPose;
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr mCamModelTransformed(new pcl::PointCloud<pcl::PointXYZRGBA>());
	pcl::transformPointCloud(*mCamModel, *mCamModelTransformed, mCamPose);
	pcl::toPCLPointCloud2(*mCamModelTransformed, mCamMesh.cloud);

	pcl::visualization::PCLVisualizer mViewer = pcl::visualization::PCLVisualizer("3D Viewer");
	//mViewer.setCameraClipDistances(9.6, 18.3);
	//mViewer.setCameraPosition(-0.3, -12.8, -1.33, -0.15, 0.64, -0.13, -0.21, -0.08, 0.97); // 按下 h 键，得到帮助信息，再根据帮助信息按下 c 键，得到当前camera的参数
	///** \brief Set the camera pose given by position, viewpoint and up vector
	//* \param[in] pos_x the x coordinate of the camera location
	//* \param[in] pos_y the y coordinate of the camera location
	//* \param[in] pos_z the z coordinate of the camera location
	//* \param[in] view_x the x component of the view point of the camera
	//* \param[in] view_y the y component of the view point of the camera
	//* \param[in] view_z the z component of the view point of the camera
	//* \param[in] up_x the x component of the view up direction of the camera
	//* \param[in] up_y the y component of the view up direction of the camera
	//* \param[in] up_z the y component of the view up direction of the camera
	//* \param[in] viewport the viewport to modify camera of (0 modifies all cameras)
	//*/
	//mViewer.setSize(1920, 1080);
	mViewer.setBackgroundColor(0, 0, 0);
	mViewer.addPolygonMesh(mCamMesh);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(mObjectModel);
	mViewer.addPointCloud<pcl::PointXYZRGBA>(mObjectModel, rgb, "sample");
	mViewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample");

	while (!mViewer.wasStopped())
	{
		mViewer.spinOnce(100);
	}
}

void cVisualization::Show(const Eigen::Matrix4f &camPose, const Eigen::Matrix4f &keyframePose)
{
	mCamPose = camPose;
	mKeyframePose = keyframePose;
	pcl::PolygonMesh mKeyframeMesh = mCamMesh;

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr mCamModelTransformed(new pcl::PointCloud<pcl::PointXYZRGBA>());
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr mKeyframeTransformed(new pcl::PointCloud<pcl::PointXYZRGBA>());
	pcl::transformPointCloud(*mCamModel, *mCamModelTransformed, mCamPose);
	pcl::transformPointCloud(*mCamModel, *mKeyframeTransformed, mKeyframePose);

	for (auto it = mKeyframeTransformed->begin(); it != mKeyframeTransformed->end(); it++)
	{
		it->r = 0; it->g = 255; it->b = 0;
	}
	pcl::toPCLPointCloud2(*mCamModelTransformed, mCamMesh.cloud);
	pcl::toPCLPointCloud2(*mKeyframeTransformed, mKeyframeMesh.cloud);

	pcl::visualization::PCLVisualizer mViewer = pcl::visualization::PCLVisualizer("3D Viewer");
	mViewer.setBackgroundColor(0, 0, 0);
	mViewer.addPolygonMesh(mKeyframeMesh,"keyframe");
	mViewer.addPolygonMesh(mCamMesh,"camera");
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(mObjectModel);
	mViewer.addPointCloud<pcl::PointXYZRGBA>(mObjectModel, rgb, "sample");
	mViewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample");

	while (!mViewer.wasStopped())
	{
		mViewer.spinOnce(100);
	}
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cVisualization::image2PointCloud(Mat rgb, Mat depth)
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

	for (int m = 0; m < depth.rows; m++)
	{
	    for (int n = 0; n < depth.cols; n++)
	    {
		    // 获取深度图中(m, n)处的值
		    unsigned short d = depth.ptr<unsigned short>(m)[n];
		    if (d == 0)
			    continue;
		    pcl::PointXYZRGBA p;

		    p.z = double(d) / mCam_scale;
		    p.x = (n - mCam_cx) * p.z / mCam_fx;
		    p.y = (m - mCam_cy) * p.z / mCam_fy;

		    p.b = rgb.ptr<uchar>(m)[n * 3 + 0];
		    p.g = rgb.ptr<uchar>(m)[n * 3 + 1];
		    p.r = rgb.ptr<uchar>(m)[n * 3 + 2];

		    cloud->points.push_back(p);
	    }
	}

	cloud->height = 1;
	cloud->width = cloud->points.size();
	cloud->is_dense = false;

	return cloud;
}

void cVisualization::Save(Mat rgb1, Mat depth1, Mat rgb2, Mat depth2, string savepath, const Eigen::Matrix4f &TMatrix)
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud1 = image2PointCloud(rgb1, depth1);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud2 = image2PointCloud(rgb2, depth2);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGBA>());
	pcl::transformPointCloud(*cloud2, *output, TMatrix);
	*output += *cloud1;
	pcl::io::savePCDFile(savepath, *output);
}

bool cVisualization::gen2DMap(string filename)
{
	if (filename.length() == 0)
	{
		cerr << "Confirm your 2DMap save path" << endl;
		return false;
	}

	cout << "Generating 2D map ..." << endl;
	clock_t time_begin = clock();

	pcl::PointCloud<pcl::PointXYZ>::Ptr mPointCloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr mPointCloud0;
	Eigen::Matrix3f covariance_matrix;
	Eigen::Vector4f xyz_centroid;
	Eigen::Matrix3f evecs;
	Eigen::Vector3f evals;
	Eigen::Matrix4f P;

	mPointCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*mObjectModel, *mPointCloud);
	mPointCloud0.reset(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*mObjectModel, *mPointCloud0);

	pcl::compute3DCentroid(*mPointCloud, xyz_centroid);
	pcl::computeCovarianceMatrix(*mPointCloud, xyz_centroid, covariance_matrix);
	pcl::eigen33(covariance_matrix, evecs, evals);

	P = Eigen::Matrix4f::Identity();
	for (int i = 0; i < 3; ++i)
	{
		P(i, 0) = evecs(0, i);
		P(i, 1) = evecs(1, i);
		P(i, 2) = evecs(2, i);
	}

	Eigen::Vector4f T = -P * xyz_centroid;
	P(0, 3) = T(0);
	P(1, 3) = T(1);
	P(2, 3) = T(2);

	Eigen::Matrix4f P1 = Eigen::Matrix4f::Identity();
	P1(0, 0) = P1(2, 2) = 0;
	P1(0, 2) = 1;
	P1(2, 0) = -1;
	P = P1 * P;
	pcl::transformPointCloud(*mObjectModel, *mObjectModel, P);
	printf("%f %f %f %f\n%f %f %f %f\n%f %f %f %f\n%f %f %f %f\n\n", P(0, 0), P(0, 1), P(0, 2), P(0, 3), P(1, 0), P(1, 1), P(1, 2), P(1, 3),
		P(2, 0), P(2, 1), P(2, 2), P(2, 3), P(3, 0), P(3, 1), P(3, 2), P(3, 3));
	pcl::copyPointCloud(*mObjectModel, *mPointCloud);
	vector<cv::Point2f> points2d;

	float minX = 1000;
	float minY = 1000;
	float minHeight = 1000;
	float maxHeight = -1000;

	int index = 0;
	for (auto it = mPointCloud->begin(), it2 = mPointCloud0->begin(); it != mPointCloud->end(); ++it,++it2)
	{
		float y = it->x;
		float z = it->y;
		//add by gp
		float height = it->z;

		if (index++<5)
		    printf("%f,%f,%f,  %f,%f,%f\n", it2->x,it2->y,it2->z,it->x, it->y, it->z);
		
		if (height > maxHeight) maxHeight = height;
		if (height < minHeight) minHeight = height;

		if (minX > y)  minX = y;
		if (minY > z)  minY = z;

		//cv::Point2f tmp(y, z);

		//points2d.push_back(tmp);
	}
	printf("%f,%f\n", minHeight, maxHeight);

	for (auto it = mPointCloud->begin(); it != mPointCloud->end(); ++it)
	{
		float y = it->x;
		float z = it->y;
		//add by gp
		float height = it->z;
		//if (height > maxHeight) maxHeight = height;
		//if (height < minHeight) minHeight = height;

		//if (minX > y)  minX = y;
		//if (minY > z)  minY = z;
		if ((fabs(height - minHeight) < 0.3) || (fabs(height - maxHeight) < 0.3))
			continue;

		cv::Point2f tmp(y, z);

		points2d.push_back(tmp);
	}

	float height = -1;
	float width = -1;

	for (auto it = points2d.begin(); it != points2d.end(); ++it)
	{
		it->x -= minX;
		it->x *= 100;
		if (width < it->x)  width = it->x;

		it->y -= minY;
		it->y *= 100;
		if (height < it->y)  height = it->y;
	}

	int Height = (int)height + 1;
	int Width = (int)width + 1;

	cv::Mat _2DMap(Height, Width, CV_8UC3, Scalar(255, 255, 255));

	for (auto it = points2d.begin(); it != points2d.end(); ++it)
	{
		int row = _2DMap.rows - it->y - 1;
		int col = it->x;
		uchar* ptr = _2DMap.ptr<uchar>(row);
		ptr[3 * col] = 0;
		ptr[3 * col + 1] = 0;
		ptr[3 * col + 2] = 0;
	}

	cv::imwrite(filename, _2DMap);

	clock_t time_end = clock();
	double duration = (time_end - time_begin) * 1000.0 / CLOCKS_PER_SEC;
	cout << "Generate 2DMap success, saved as: "  << filename << endl;
	cout << "2DMap Generation Time: " << duration << " ms" << endl;

	cout << "Write related parameters to file..." << endl;
	FILE *fout = fopen("project_paras_auto.txt", "w");
	if (NULL == fout)
	{
		cout << "Write parameters failed" << endl;
		return false;
	}

	fprintf(fout, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %d %d %f %f\n", P(0, 0), P(0, 1), P(0, 2), P(0, 3), P(1, 0), P(1, 1), P(1, 2), P(1, 3),
		P(2, 0), P(2, 1), P(2, 2), P(2, 3), P(3, 0), P(3, 1), P(3, 2), P(3, 3), Height, Width, minX, minY);

	fclose(fout);

	cout << "Write parameters success, saved as: project_paras.txt" << endl;

	return true;
}
