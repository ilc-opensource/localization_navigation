/*
 * License: 3-clause BSD. See LICENSE file in root directory.
 * Copyright(c) 2015-2016 Intel Corporation. All Rights Reserved.
 * */

#pragma once

#include <memory>
#include <opencv2/core/core.hpp>
#include "cDataStructure.h"

#if WIN32

#pragma warning (disable : 4251)

#if defined(Localization_EXPORTS)
#define LOCALIZATIONAPI __declspec(dllexport)
#else
#define LOCALIZATIONAPI __declspec(dllimport)
#endif

#else

#define LOCALIZATIONAPI

#endif

class LOCALIZATIONAPI Localization final
{
public:
	Localization();
	~Localization();
	Localization(const Localization &) = delete;
	Localization &operator=(const Localization &) = delete;

	/* load necessary data files and do initial configs */
	bool init(const char *dataset_path);
	/* set camera intrinsics */
	void setIntrinsics(float fx, float fy, float ppx, float ppy);
	/* localize input frame pair, return information in output */
	void search(cv::Mat query_rgb, cv::Mat query_d, cItem &output, bool fast = true);
	/* display the frame input to search in 3D location and HTML, call after search */
	void show(bool isShow, bool isHTML);

private:
	class Impl;
	std::unique_ptr<Impl> impl;
};
