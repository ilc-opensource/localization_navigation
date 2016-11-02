#include <vector>
#include <string>
#include <memory>

#include <DBoW2.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#if WIN32

#pragma warning (disable : 4251)

#if defined(DBoW2_EXPORTS)
#define DBOW2API __declspec(dllexport)
#else
#define DBOW2API __declspec(dllimport)
#endif

#else

#define DBOW2API

#endif

struct Query
{
	int id;
	double score;
};


typedef DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB> ORBVocabulary;
typedef DBoW2::TemplatedDatabase<DBoW2::FORB::TDescriptor, DBoW2::FORB> ORBDatabase;

class DBOW2API ORBVocTree
{
public:
	typedef DBoW2::FORB::TDescriptor TDescriptor;

	ORBVocTree();
	~ORBVocTree();

	void loadImages(std::string image_path, std::string orbDescFile);
	void loadImagesWithExsitingTree(std::string image_path, std::string orbDescFile);
	void buildVocDict(std::vector<std::string> image_path_Dict);

	void querydb(cv::Mat query_desc, int ret_num, std::vector<Query> &query_out);

	void loadVoctree(std::string tree_path);
	void loadVocdb(std::string db_path);
	void saveVoctree(std::string voc_path);
	void saveVocdb(std::string db_path);

	void detectORBDesc(cv::Mat img, cv::Mat &desc, std::vector<cv::KeyPoint> &keypoint)
	{
		cv::Mat mask;
		orb->detectAndCompute(img, mask, keypoint, desc);
	}

private:
	void convertDesc(cv::Mat desc, std::vector<TDescriptor> &orb_desc)
	{
		orb_desc.reserve(desc.rows);
		for (int i = 0; i < desc.rows; i++)
			orb_desc.push_back(desc.row(i));
	}

	DBoW2::WeightingType weight;
	DBoW2::ScoringType score;

	std::shared_ptr<ORBVocabulary> voc_orb;
	std::shared_ptr<ORBDatabase> db_orb;

	int K;
	int L;

	cv::Ptr<cv::ORB> orb;
	int max_feature;
};

DBOW2API void getSortedFileList(std::string search_path, std::vector<std::string> &files);
