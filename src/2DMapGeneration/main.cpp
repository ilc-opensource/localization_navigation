/*
 * License: 3-clause BSD. See LICENSE file in root directory.
 * Copyright(c) 2015-2016 Intel Corporation. All Rights Reserved.
 * */

#include <iostream>
#include "cVisualization.h"

using namespace std;

int main(int argc, char* argv[])
{
	if (argc != 3) {
		cerr << "Usage:\n"
			"2DMapGeneration g2o_linkbreak_.ply cam.ply\n"
			<< endl;
		exit(EXIT_FAILURE);
	}

	string model(argv[1]);
	string cam(argv[2]);

	cVisualization vis(model, cam);

	vis.gen2DMap("map2d.png");
	vis.Show();

	return 0;
}
