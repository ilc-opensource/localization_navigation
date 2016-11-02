/*
 * License: 3-clause BSD. See LICENSE file in root directory.
 * Copyright(c) 2015-2016 Intel Corporation. All Rights Reserved.
 * */
#include <iostream>
#include <string>
#include "ORBVocTree.h"

using namespace std;
using namespace cv;


int main(int argc, char **argv)
{
	if (argc != 2)
	{
		cerr << "Usage:\n"
			"  TreeGeneration RGB_images_directory\n"
			<< endl;
		exit(EXIT_FAILURE);
	}
	string imagePath(argv[1]);
	vector<string> imgPathDict;
	string orbTree("orb_db_yml.gz");
	string orbDescKpStored = "orbDescKpStored.xml";
	ORBVocTree orb_case;
	
	imgPathDict.push_back(imagePath);
	orb_case.buildVocDict(imgPathDict);
	
	orb_case.loadImagesWithExsitingTree(imagePath, orbDescKpStored);
	//orb_case.loadImages(imagePath, orbDescKpStored);
	orb_case.saveVocdb(orbTree);

	return 0;
}
