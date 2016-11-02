#include <cmath>
#include <ctime>
#include <opencv2/highgui/highgui.hpp>
#include "ORBVocTree.h"

#if WIN32
#include <io.h>
#else
#include <unistd.h>
#include <dirent.h>
#endif

using namespace DBoW2;
using namespace std;
using namespace cv;

#define BIGTREE

#ifdef BIGTREE
ORBVocTree::ORBVocTree() : weight(TF_IDF), score(L1_NORM), K(10), L(6), max_feature(4096)
{
	orb = ORB::create(max_feature);
}
#else
ORBVocTree::ORBVocTree() : weight(TF_IDF), score(L1_NORM), K(9), L(3), max_feature(4096)
{
	orb = ORB::create(max_feature);
}
#endif
ORBVocTree::~ORBVocTree()
{
}

void ORBVocTree::loadImages(string image_path, string orbDescFile)
{
	vector<vector<TDescriptor>> descriptors;
	vector<string> image_files;

	getSortedFileList(image_path, image_files);

	auto image_count = image_files.size();

	if (image_count == 0)
		return;

	printf("Detect orb descriptor for each image\n");
	clock_t time_begin = clock();

	FileStorage fs(orbDescFile, FileStorage::WRITE);
	fs << "frameCount" << (int)image_count;
	fs << "features" << "[:";
	for (unsigned i = 0; i < image_count; ++i)
	{
		string filename = image_path + "/" + image_files[i];
		Mat img = imread(filename);
		if (!img.data)
			continue;

		vector<TDescriptor> orb_desc;
		Mat desc;
		vector<KeyPoint> kp;
		detectORBDesc(img, desc, kp);
		convertDesc(desc, orb_desc);
		descriptors.push_back(orb_desc);

		fs << "{:";
		fs << "imageID" << (int)i;
		fs << "keypoint" << kp;
		fs << "descriptor" << desc;
		fs << "}";
	}
	fs << "]";//features
	fs.release();

	clock_t time_end = clock();
	double duration = (time_end - time_begin) / (double)CLOCKS_PER_SEC;
	printf("Descriptor Detection Time: %.3f s\n\n", duration);

	auto desc_count = descriptors.size();

	int val = (int)ceil(log10f(float(desc_count))) - 1;
	if (val > L)  L = val;

	printf("Create OrbTree and Database\n");
	printf("K = %d, L = %d, image_descriptors = %lu\n", K, L, descriptors.size());

	printf("1. Create OrbTree\n");
	time_begin = clock();

	voc_orb.reset(new ORBVocabulary(K, L, weight, score));
	voc_orb->create(descriptors);
	voc_orb->save("voc.txt");

	time_end = clock();
	duration = (time_end - time_begin) / (double)CLOCKS_PER_SEC;
	printf("OrbTree Generation Time: %.3f s\n", duration);

	printf("2. Create Database\n");
	time_begin = clock();

	db_orb.reset(new ORBDatabase());
	db_orb->setVocabulary(*voc_orb, false, 0);

	for (unsigned i = 0; i < desc_count; ++i)
	{
		db_orb->add(descriptors[i]);
	}

	time_end = clock();
	duration = (time_end - time_begin) / (double)CLOCKS_PER_SEC;
	printf("Databse Generation Time: %.3f s\n\n", duration);
}

void ORBVocTree::loadImagesWithExsitingTree(string image_path, string orbDescFile)
{
	vector<vector<TDescriptor>> descriptors;
	vector<string> image_files;

	getSortedFileList(image_path, image_files);

	auto image_count = image_files.size();

	if (image_count == 0)
		return;

	printf("Detect orb descriptor for each image\n");
	clock_t time_begin = clock();

	FileStorage fs(orbDescFile, FileStorage::WRITE);
	fs << "frameCount" << (int)image_count;
	fs << "features" << "[:";
	for (unsigned i = 0; i < image_count; ++i)
	{
		string filename = image_path + "/" + image_files[i];
		Mat img = imread(filename);
		if (!img.data)
			continue;

		vector<TDescriptor> orb_desc;
		Mat desc;
		vector<KeyPoint> kp;
		detectORBDesc(img, desc, kp);
		convertDesc(desc, orb_desc);
		descriptors.push_back(orb_desc);

		fs << "{:";
		fs << "imageID" << (int)i;
		fs << "keypoint" << kp;
		fs << "descriptor" << desc;
		fs << "}";
	}
	fs << "]";//features
	fs.release();

	clock_t time_end = clock();
	double duration = (time_end - time_begin) / (double)CLOCKS_PER_SEC;
	printf("Descriptor Detection Time: %.3f s\n\n", duration);
	auto desc_count = descriptors.size();

	/*	
	int val = (int)ceil(log10f(float(desc_count))) - 1;
	if (val > L)  L = val;
	printf("Create OrbTree and Database\n");
	printf("K = %d, L = %d, image_descriptors = %lu\n", K, L, descriptors.size());
	printf("1. Create OrbTree\n");
	time_begin = clock();
	voc_orb.reset(new ORBVocabulary(K, L, weight, score));
	voc_orb->create(descriptors);
	voc_orb->save("voc.txt");
	time_end = clock();
	duration = (time_end - time_begin) / (double)CLOCKS_PER_SEC;
	printf("OrbTree Generation Time: %.3f s\n", duration);
	*/
	//voc_orb->load("vocDictTree.txt");
	voc_orb->loadFromTextFile("vocDictTreetxt.txt");
	//voc_orb->loadFromTextFile("D:\\work\\dataset\\ORBvoc.txt");

	printf("2. Create Database\n");
	time_begin = clock();

	db_orb.reset(new ORBDatabase());
	db_orb->setVocabulary(*voc_orb, false, 0);

	for (unsigned i = 0; i < desc_count; ++i)
	{
		db_orb->add(descriptors[i]);
	}

	time_end = clock();
	duration = (time_end - time_begin) / (double)CLOCKS_PER_SEC;
	printf("Databse Generation Time: %.3f s\n\n", duration);
}

void ORBVocTree::buildVocDict(vector<string> image_path_Dict)
{
	vector<vector<TDescriptor>> descriptorsDict;
	printf("Detect orb descriptor for each image\n");
	clock_t time_begin = clock();
	for (int n = 0; n < image_path_Dict.size(); n++)
	{
		vector<string> image_files_Dict;
		getSortedFileList(image_path_Dict[n], image_files_Dict);
		auto image_count_dict = image_files_Dict.size();
		if (image_count_dict == 0)
			continue;
		for (unsigned i = 0; i < image_count_dict; ++i)
		{
			string filename = image_path_Dict[n] + "/" + image_files_Dict[i];
			Mat img = imread(filename);
			if (!img.data)
				continue;

			vector<TDescriptor> orb_desc;
			Mat desc;
			vector<KeyPoint> kp;
			detectORBDesc(img, desc, kp);
			convertDesc(desc, orb_desc);
			descriptorsDict.push_back(orb_desc);
		}
	}
	clock_t time_end = clock();
	double duration = (time_end - time_begin) / (double)CLOCKS_PER_SEC;
	printf("Descriptor Detection Time: %.3f s\n\n", duration);

	auto desc_count = descriptorsDict.size();

	int val = (int)ceil(log10f(float(desc_count))) - 1;
	if (val > L)  L = val;

	printf("1. Create Orb Vocabulary Tree\n");
	printf("K = %d, L = %d, image_descriptors = %lu\n", K, L, descriptorsDict.size());
	time_begin = clock();
	voc_orb.reset(new ORBVocabulary(K, L, weight, score));
	voc_orb->create(descriptorsDict);
	time_end = clock();
	duration = (time_end - time_begin) / (double)CLOCKS_PER_SEC;
	printf("OrbTree Generation Time: %.3f s\n\n", duration);
	voc_orb->save("vocDictTree.txt");
	voc_orb->saveToTextFile("vocDictTreetxt.txt");
}

void ORBVocTree::querydb(Mat query_desc, int ret_num, vector<Query> &query_out)
{
	vector<TDescriptor> orb_desc;
	QueryResults ret;

	convertDesc(query_desc, orb_desc);
	db_orb->query(orb_desc, ret, ret_num);

	for (int i = 0; i < ret_num; i++)
	{
		Query tmp;
		tmp.id = ret[i].Id;
		tmp.score = ret[i].Score;
		query_out.push_back(tmp);
	}
}

void ORBVocTree::loadVoctree(string tree_path)
{
	voc_orb.reset(new ORBVocabulary(tree_path.c_str()));
}

void ORBVocTree::loadVocdb(string db_path)
{
	db_orb.reset(new ORBDatabase());
	db_orb->load(db_path);
}

void ORBVocTree::saveVoctree(string tree_path)
{
	voc_orb->save(tree_path);
}

void ORBVocTree::saveVocdb(string db_path)
{
	db_orb->save(db_path);
}


inline bool cmpFile_orb(std::string const &arg_a, std::string const &arg_b)
{
	return arg_a.size() < arg_b.size() || (arg_a.size() == arg_b.size() && arg_a < arg_b);
}

void getFilesHelper(std::string search_path, std::vector<std::string>& files)
{
#ifdef WIN32
	intptr_t handler;
	_finddata_t fileInfo;

	if (search_path.empty())
		return;

	handler = _findfirst((search_path + "/*").c_str(), &fileInfo);

	if(handler != -1L)
	{
		while(_findnext(handler, &fileInfo) == 0)
		{
			if ((fileInfo.attrib & _A_SUBDIR) == 16 && strcmp(fileInfo.name, ".."))  // A directory
			{
			}
			else if (!(fileInfo.attrib & _A_SUBDIR))  // A file
			{
				files.push_back(fileInfo.name);
			}
		}
	}
	_findclose(handler);
#else
	DIR *dir;
	struct dirent *entry;

	if((dir = opendir(search_path.c_str())) != NULL)
	{
		while((entry = readdir(dir)))
		{
			if(entry->d_type == 8)  // A file
			{
				files.push_back(entry->d_name);
			}
			else if(entry->d_name[0] != '.' && entry->d_type == 4)  // A directory
			{
			}
		}
	}
	closedir(dir);
#endif
}

void getSortedFileList(std::string search_path, std::vector<std::string>& files)
{
	getFilesHelper(search_path, files);
	sort(files.begin(), files.end(), cmpFile_orb);
}
