/*
 * License: 3-clause BSD. See LICENSE file in root directory.
 * Copyright(c) 2015-2016 Intel Corporation. All Rights Reserved.
 * */

#pragma once

#include <vector>
#include <opencv2/opencv.hpp>
#include "PathFinder.h"
#include "AStar.h"

#if WIN32

#if defined(PathPlanning_EXPORTS)
#define PATHPLANNINGAPI __declspec(dllexport)
#else
#define PATHPLANNINGAPI __declspec(dllimport)
#endif

#else

#define PATHPLANNINGAPI

#endif

class Square : public AStarNode
{
public:

	Square()
	{}

	~Square()
	{}

	void setType(const bool type)
	{
		m_type = type;
	}

	bool getType() const
	{
		return m_type;
	}

	// A diagonal move has a sqrt(2) cost, 1 otherwise
	float localDistanceTo(AStarNode* node) const
	{
		if (node->getX() != m_x && node->getY() != m_y)
			return 1.41421356237f;
		else
			return 1.0f;
	}

	float distanceTo(AStarNode* node) const
	{
		int newX = m_x - node->getX(), newY = m_y - node->getY();
		return sqrtf(static_cast<float>(newX*newX + newY*newY));
	}

private:
	// To tell wether a pixel is "walkable" or not
	bool m_type;
};

class PATHPLANNINGAPI PathPlan final
{
public:
	PathPlan();
	~PathPlan();
	PathPlan(const PathPlan &) = delete;
	PathPlan &operator=(const PathPlan &) = delete;
	
	void init(cv::Mat map, float safeSpace, float robotRadius);
	void release();
	bool pathplanning(int startx, int starty, int endx, int endy, std::vector<cv::Point> &output);

private:
	cv::Mat map2D, map2D_erode;
	int mapWidth, mapHeight;
	Square **squares;
	int erodeSize;

};