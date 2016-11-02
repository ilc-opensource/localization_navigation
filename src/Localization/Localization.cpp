/*
 * License: 3-clause BSD. See LICENSE file in root directory.
 * Copyright(c) 2015-2016 Intel Corporation. All Rights Reserved.
 * */

#include "Localization.h"
#include "cPose.h"
#include "cVisualization.h"
#include "cLocalization.h"
#include "ORBVocTree.h"

using namespace std;
using namespace cv;

namespace {

void PrintHTMLHeader(FILE *f, int num_nns)
{
	fprintf(f, "<html>\n"
		"<header>\n"
		"<title>Vocabulary tree results</title>\n"
		"</header>\n"
		"<body>\n"
		"<h1>Vocabulary tree results</h1>\n"
		"<hr>\n\n");

	fprintf(f, "<table border=2 align=center style=font-size:25pt>\n<tr>\n<th>Query image</th>\n");
	for (int i = 0; i < num_nns; i++) {
		fprintf(f, "<th>Match %d</th>\n", i + 1);
	}
	fprintf(f, "</tr>\n");
}

void PrintHTMLRow(FILE *f, string query_in,
	vector<cItem> items, int num_nns, string rgb_db,
	const vector<string> &db_images, int MostInlier, int MostOrbMatches, int MostValidMatches, float distance, int query_orb_num)
{
	fprintf(f, "<tr align=center>\n<td><img src=\"%s\"</td>\n", query_in.c_str());

	for (int i = 0; i < num_nns; i++) {
		string db_all = rgb_db + "/" + db_images[items[i].index];
		fprintf(f, "<td><img src=\"%s\"</td>\n", db_all.c_str());
	}

	fprintf(f, "</tr>\n<tr align=center style=font-size:25pt>\n");
	int last_slash = query_in.find_last_of("/");
	int last_dot = query_in.find_last_of(".");
	string query_name = query_in.substr(last_slash + 1, last_dot - last_slash - 1);
	fprintf(f, "<td> %s, Inliers: %d / %d, ORBMatches: %d, dist=%f, query_sift=%d </td>\n", query_name.c_str(), MostInlier, MostValidMatches, MostOrbMatches, distance, query_orb_num);
	for (int i = 0; i < num_nns; i++)
		fprintf(f, "<td>Score: %0.5f, Inlier: %d / %d, ORBMatches: %d</td>\n", items[i].score, items[i].inlier, items[i].validMatches, items[i].orbMatches);

	fprintf(f, "</tr>\n");
}

void PrintHTMLFooter(FILE *f)
{
	fprintf(f, "</tr>\n"
		"</table>\n"
		"<hr>\n"
		"</body>\n"
		"</html>\n");
}

};

class Localization::Impl
{
public:
	Impl() :
		candidate_num(5), curr_most_inlier(-1), curr_best_index(-1)
	{}

	~Impl() = default;
	Impl(const Impl &) = delete;
	Impl &operator=(const Impl &) = delete;

	bool init(const char *dataset_path);
	void setIntrinsics(float fx, float fy, float ppx, float ppy);
	void search(Mat rgb, Mat depth, cItem &output, bool fast = true);
	void show(bool isShow, bool isHTML);

private:
	string voctree;			// 生成的voctree
	string keyframeRGB;		// vslam选出来的keyframe
	string keyframeD;		// vslam选出来的keyframe
	string keyframePose;	// vslam的结果
	string model;			// vslam的结果
	string cam;				// 相机三维模型
	string xml;				// vslam的结果
	string orbDescKpStored; // keyframe\rgb的orb keypoints和descriptors

	vector<string> vRGBList;
	vector<string> vDList;

	shared_ptr<cLocalization> loc;
	ORBVocTree orb_tree;
	vector<cItem> query;

	const int candidate_num;
	Mat curr_query_rgb;
	int curr_most_inlier;
	int curr_best_index;
	cItem curr_output;

	cPose cpose;
	vector<vector<KeyPoint>> orbKeypointsVector;
	vector<Mat> orbDescVector;
};

Localization::Localization() : impl(new Impl()) {}
Localization::~Localization() {}

bool Localization::init(const char *dataset_path)
{
	return impl->init(dataset_path);
}

void Localization::setIntrinsics(float fx, float fy, float ppx, float ppy)
{
	impl->setIntrinsics(fx, fy, ppx, ppy);
}

void Localization::search(Mat rgb, Mat depth, cItem &output, bool fast)
{
	impl->search(rgb, depth, output, fast);
}

void Localization::show(bool isShow, bool isHTML)
{
	impl->show(isShow, isHTML);
}

bool Localization::Impl::init(const char *dataset_path)
{
	string data_folder(dataset_path);
	voctree = data_folder + "/keyframe/orb_db_yml.gz";
	keyframeRGB = data_folder + "/keyframe/rgb";
	keyframeD = data_folder + "/keyframe/depth";
	keyframePose = data_folder + "/keyframe/AfterOptimize.g2o";
	model = data_folder + "/keyframe/g2o_linkbreak_.ply";
	cam = data_folder + "/keyframe/cam.ply";
	xml = data_folder + "/keyframe/KeyFrame.xml";
	orbDescKpStored = data_folder + "/keyframe/orbDescKpStored.xml";

	FileStorage fs(orbDescKpStored, FileStorage::READ);
	int frameCount = (int)fs["frameCount"];
	int tImageID;
	FileNode features = fs["features"];
	FileNodeIterator it = features.begin(), it_end = features.end();
	for (; it != it_end; ++it)
	{
		(*it)["imageId"] >> tImageID;
		vector<KeyPoint> tKp;
		read((*it)["keypoint"], tKp);
		cv::Mat tDesc;
		(*it)["descriptor"] >> tDesc;
		orbKeypointsVector.push_back(tKp);
		orbDescVector.push_back(tDesc);
	}
	fs.release();

	getSortedFileList(keyframeRGB, vRGBList);
	getSortedFileList(keyframeD, vDList);
	if (vRGBList.size() == 0)
		return false;

	orb_tree.loadVocdb(voctree);

	cpose.setIntrinsic(620, 620, 320, 240);

	loc.reset(new cLocalization(keyframePose, xml));

	curr_query_rgb.release();
	curr_most_inlier = -1;
	curr_best_index = -1;

	return true;
}

void Localization::Impl::setIntrinsics(float fx, float fy, float ppx, float ppy)
{
	cpose.setIntrinsic(fx, fy, ppx, ppy);
}

void Localization::Impl::search(Mat query_rgb, Mat query_d, cItem &output, bool fast)
{
	// find five most similar images of query image.
	const int threshold_inlier = 30;

	curr_query_rgb.release();
	curr_most_inlier = -1;
	curr_best_index = -1;
	query.clear();

	curr_query_rgb = query_rgb.clone();

	vector<KeyPoint> kp;
	Mat desc;
	orb_tree.detectORBDesc(query_rgb, desc, kp);
	
	
    cpose.setQueryFrame(query_rgb, query_d);
	vector<Query> query_out;
	orb_tree.querydb(desc, candidate_num, query_out);
	for (int i = 0; i < candidate_num; i++)
	{
		cItem tmp;
		tmp.index = query_out[i].id;
		tmp.score = query_out[i].score;
		query.push_back(tmp);
	}

	vector<int> inliers(candidate_num, -1);
	for (int i = 0; i < candidate_num; i++)
	{
		string candidate_rgb_s = keyframeRGB + "/" + vRGBList[query[i].index];
		string candidate_depth_s = keyframeD + "/" + vDList[query[i].index];
		Mat candidate_rgb = imread(candidate_rgb_s, IMREAD_COLOR);
		Mat candidate_depth = imread(candidate_depth_s, IMREAD_ANYDEPTH);
		if (!candidate_rgb.data || !candidate_depth.data)
			continue;

		cpose.setKeyFrame(candidate_rgb, candidate_depth);
		cpose.ransacPnP(kp, desc, orbKeypointsVector[query[i].index], orbDescVector[query[i].index]);

		query[i].imageName = vRGBList[query[i].index];
		query[i].inlier = cpose.getInlierNum();
		query[i].orbMatches = cpose.getMatchNum();
		query[i].validMatches = cpose.getValidMatch();
		Eigen::Matrix4f tmp = cpose.getTMatrix();
		for (int m = 0; m < 4; m++)
			for (int n = 0; n < 4; n++)
				query[i].relativePose[m][n] = tmp(m, n);

		if (query[i].inlier > threshold_inlier)
		{
			inliers[i] = query[i].inlier;
			if (fast)
				break;
		}
	}

	int max_inlier = -1;
	for (int i = 0; i < inliers.size(); i++)
	{
		if (inliers[i] > max_inlier)
		{
			curr_best_index = i;
			curr_most_inlier = inliers[i];
			max_inlier = curr_most_inlier;
		}
	}

	if (curr_best_index != -1)
	{
		output.imageName = query[curr_best_index].imageName;
		output.index = query[curr_best_index].index;
		output.inlier = query[curr_best_index].inlier;
		Eigen::Matrix4f tmp2;
		for (int m = 0; m < 4; m++)
			for (int n = 0; n < 4; n++)
				tmp2(m, n) = query[curr_best_index].relativePose[m][n];

		Eigen::Matrix4f tmp3 = loc->GetAbsoluteCamPose(query[curr_best_index].imageName, tmp2, 1.0);
		for (int m = 0; m < 4; m++)
			for (int n = 0; n < 4; n++)
				output.absolutePose[m][n] = tmp3(m, n);

		for (int m = 0; m < 4; m++)
			for (int n = 0; n < 4; n++)
				output.relativePose[m][n] = query[curr_best_index].relativePose[m][n];

		output.score = query[curr_best_index].score;
		output.orbMatches = query[curr_best_index].orbMatches;
		output.validMatches = query[curr_best_index].validMatches;
		output.isFound = true;

		curr_output = output;
	}
	else
	{
		curr_output.isFound = output.isFound = false;
		cerr << "Localization failed! try again!" << endl;
	}

	return;
}

void Localization::Impl::show(bool isShow, bool isHTML)
{
	FILE *f_html = NULL;
	if (isHTML)
	{
		char *output_html = "./results.html";
		f_html = fopen(output_html, "w");
		PrintHTMLHeader(f_html, candidate_num);
	}
	if (curr_best_index != -1)
	{
		if (isHTML)
		{
			imwrite("query_image.jpg", curr_query_rgb);
			PrintHTMLRow(f_html, "query_image.jpg", query, candidate_num, keyframeRGB, vRGBList,
				curr_most_inlier, curr_output.orbMatches, curr_output.validMatches, 0, 0);
			PrintHTMLFooter(f_html);
			fclose(f_html);
		}
		if (isShow)
		{
			Eigen::Matrix4f tmp4;
			for (int m = 0; m<4; m++)
				for (int n = 0; n<4; n++)
					tmp4(m, n) = query[curr_best_index].relativePose[m][n];
			Eigen::Matrix4f mCamPose = loc->GetAbsoluteCamPose(query[curr_best_index].imageName, tmp4, 0.2);
			cVisualization vis(model, cam);
			vis.Show(mCamPose);
		}
	}
	else
	{
		if (isHTML)
		{
			imwrite("query_image.jpg", curr_query_rgb);
			PrintHTMLRow(f_html, "query_image.jpg", query, candidate_num, keyframeRGB, vRGBList, 0, 0, 0, 0, 0);
			PrintHTMLFooter(f_html);
			fclose(f_html);
		}
	}
}
