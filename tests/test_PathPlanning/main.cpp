/*
 * License: 3-clause BSD. See LICENSE file in root directory.
 * Copyright(c) 2015-2016 Intel Corporation. All Rights Reserved.
 * */

#include <ctime>
#include <iostream>
#include "PathPlanning.hpp"

using namespace std;
using namespace cv;

int main(int argc, char** argv)
{
	if (argc != 6) {
		cout << "Usage:\n"
			"This example takes an image and searches a path between\n"
			"2 points user-defined. Pure white RGB(255,255,255) is a\n"
			"\"walkable\" pixel and any other color is a wall.\n"
			"Produces an image solution named path.jpg in the\n"
			"working directory.\n"
			"\n"
			"Run with 5 arguments:\n"
			"test_PathPlanning map_image x1 y1 x2 y2\n"
			"x1 y1 the coordinates of the start pixel\n"
			"x2 y2 the coordinates of the goal pixel\n"
			"e.g. test_PathPlanning Map.png 100 200 300 400"
			<< endl;
		exit(EXIT_FAILURE);
	}
	char* mapPath = argv[1];
	int startx = atoi(argv[2]);
	int starty = atoi(argv[3]);
	int endx = atoi(argv[4]);
	int endy = atoi(argv[5]);
	float safeSpace = 3;
	float robotRadius = 5;

	Mat image = imread(mapPath, IMREAD_GRAYSCALE);
	vector<Point> output;
	clock_t time_begin = clock();
	PathPlan pathplaner;
	//Initial the configure
	pathplaner.init(image, safeSpace, robotRadius);
	//path planning
	pathplaner.pathplanning(startx, starty, endx, endy, output);
	clock_t time_end = clock();
	double duration = (time_end - time_begin) / (double)CLOCKS_PER_SEC;
	std::cout << "total time: " << duration << " seconds" << std::endl;
	//show the path
	for (auto p : output) {
		circle(image, p, 1, (255, 0, 0));
	}
	imwrite("path.jpg", image);
	return 0;
}
