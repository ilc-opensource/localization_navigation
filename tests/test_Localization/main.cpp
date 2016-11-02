/*
 * License: 3-clause BSD. See LICENSE file in root directory.
 * Copyright(c) 2015-2016 Intel Corporation. All Rights Reserved.
 * */

#include <iostream>
#include <ctime>
#include <opencv2/opencv.hpp>
#include <boost/format.hpp>
#include "Localization.h"

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
	if (argc != 3)
	{
		cerr << "Usage:\n"
				"test_Localization data_folder query_image_index\n"
				<< endl;
		exit(EXIT_FAILURE);
	}
	string data_folder(argv[1]);
	int query_index = atoi(argv[2]);

	string query_rgb_path = boost::str(boost::format("%s/query/rgb/frame_%05d_RGBImg.jpg") % data_folder % query_index);
	string query_depth_path = boost::str(boost::format("%s/query/depth/frame_%05d_DepthDataConvertedSmooth.png") % data_folder % query_index);

	Mat queryRGB = imread(query_rgb_path, IMREAD_COLOR);
	Mat queryD = imread(query_depth_path, IMREAD_ANYDEPTH);
	if (!queryRGB.data || !queryD.data)
	{
		cerr << "error reading input frame pair!" << endl;
		return EXIT_FAILURE;
	}

	Localization loc;
	//Initial the configure
	loc.init(data_folder.c_str());

	cItem output;
	clock_t time_begin, time_end;

	time_begin = clock();
	//Localize the query picture
	loc.search(queryRGB, queryD, output);
	cout << output.inlier << " inliners found" << endl;
	time_end = clock();
	double duration = (time_end - time_begin) * 1000.0 / CLOCKS_PER_SEC;
	cout << "Total Time: " << duration << " ms" << endl;
	//Show the result
	if (output.isFound)
		loc.show(true, true);

	return EXIT_SUCCESS;
}
