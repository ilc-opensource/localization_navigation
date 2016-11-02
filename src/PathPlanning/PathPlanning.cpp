/*
 * License: 3-clause BSD. See LICENSE file in root directory.
 * Copyright(c) 2015-2016 Intel Corporation. All Rights Reserved.
 * */

#include <opencv2/opencv.hpp>
#include <chrono>
#include <iostream>
#include <ctime>
#include "PathPlanning.hpp"
using namespace cv;

/*
Create our own class deriving AStarNode.
This is needed for applying A* Algorithm to our squares.
*/

PathPlan::PathPlan() 
{
	squares = NULL;
}
PathPlan::~PathPlan() 
{ 
	release(); 
}

void PathPlan::init(cv::Mat map, float safeSpace, float robotRadius)
{
	clock_t time1 = clock();

	int width = map.cols;
	int height = map.rows;

	mapWidth = width;
	mapHeight = height;

	map2D = map.clone();
	Mat element;
	erodeSize = (robotRadius + safeSpace);             //腐蚀(膨胀)尺寸，使用腐蚀还是膨胀，根据地图中障碍物用黑色(白色)定
	element = getStructuringElement(MORPH_ELLIPSE, Size(2 * erodeSize + 1, 2 * erodeSize + 1), Point(erodeSize, erodeSize));
	erode(map, map2D_erode, element);
	/* 	Now let's create the nodes, one for each pixel
	the SFML stuff is pretty self-explanatory	 */
	squares = new Square*[width];
	for (unsigned short int x = 0; x < width; ++x)
	{
		squares[x] = new Square[height];
		for (unsigned short int y = 0; y < height; ++y)
		{
			squares[x][y].setPosition(x, y);
			squares[x][y].setType(map2D_erode.at<uchar>(y, x) == 255 ? true : false);
		}
	}
	/*
	And setup the relations between nodes. In this case,
	we want to create a link between pixels that are
	next to each other (so that a pixel has 8 neighbours)
	*/
	int newX, newY;
	Square *aChild;
	for (int x = 0; x < width; ++x)
		for (int y = 0; y < height; ++y) // traverse all squares
		{
			for (int i = -1; i < 2; ++i)
			{
				newX = squares[x][y].getX() + i;
				for (int j = -1; j < 2; ++j) // for all squares in this 3*3 square
				{
					newY = squares[x][y].getY() + j;
					if (newX > -1 && newX < width && newY > -1 && newY < height) // be sure not to go outside the limits
					{
						aChild = &(squares[newX][newY]);
						if (aChild->getType() && (newX != x || newY != y)) // only take free squares and not the one we are examining
							squares[x][y].addChild(aChild, squares[x][y].localDistanceTo(aChild));
					}
				}
			}
		}
	clock_t time2 = clock();
	double timegraph = (time2 - time1) / (double)CLOCKS_PER_SEC;
	std::cout << "graph building time:" << timegraph << " seconds" << std::endl;
}
void PathPlan::release()
{
	clock_t time3 = clock();
	if (squares)
	{
		for (unsigned short int x = 0; x < mapWidth; ++x)
			delete[] squares[x];
		delete[] squares;
		squares = NULL;
	}

	clock_t time4 = clock();
	double timeDelete = (time4 - time3) / (double)CLOCKS_PER_SEC;
	std::cout << "timeDelete:" << timeDelete << " seconds" << std::endl;
}

bool PathPlan::pathplanning(int startx, int starty, int endx, int endy, std::vector<cv::Point> &output)
{	
	clock_t time2 = clock();
	
	// Create the PathFinding stuff and the SFML image to load the image
	PathFinder<Square> p;
	std::vector<Square*> path;	

	if (squares[startx][starty].getType() == false)
	{
		int startx_final, starty_final;
		bool isFindxy = false;
		for (int radius = 1; radius < 2 * erodeSize; radius++)
		{
			int left = max(startx - radius, 0);
			int right = min(startx + radius, mapWidth);
			int up = max(starty - radius, 0);
			int down = min(starty + radius, mapHeight);
			for (int w = left; w < right; w++)
			{
				for (int h = up; h < down; h++)
				{
					float distance = sqrtf((float(startx) - float(w))*(float(startx) - float(w)) + (float(starty) - float(h))*(float(starty) - float(h)));
					if (distance < float(radius) && squares[w][h].getType())
					{
						startx_final = w;
						starty_final = h;
						isFindxy = true;
						break;
					}
				}
				if (isFindxy)
					break;
			}
			if (isFindxy)
				break;
		}
		//startx_final = startx + int(1.5f*float(startx_final - startx));
		//starty_final = starty + int(1.5f*float(starty_final - starty));
		//for (int i = -3; i <= 3; i++)
		//for (int j = -3; j <= 3; j++)
		//{
		//	map2D.at<unsigned char>(i + starty, j + startx) = 0;
		//	map2D.at<unsigned char>(i + starty_final, j + startx_final) = 0;
		//}
		int lengthx = abs(startx_final - startx);

		//int length = int(sqrtf((float(startx) - float(startx_final))*(float(startx) - float(startx_final)) + (float(starty) - float(starty_final))*(float(starty) - float(starty_final))));
		float k = (float(starty_final) - float(starty)) / (float(startx_final) - float(startx));
		for (int i = 0; i <= lengthx; i++)
		{
			Point tmp;
			if (startx_final>startx)
				tmp.x = startx + i;
			else
				tmp.x = startx - i;
			if (starty_final>starty)
				tmp.y = starty + int(k*i);
			else
				tmp.y = starty - int(k*i);

			output.push_back(tmp);
		}
		startx = startx_final;
		starty = starty_final;
	}
	p.setStart(squares[startx][starty]);
	p.setGoal(squares[endx][endy]);
	bool isFound = p.findPath<AStar>(path);
	Point tmp;
	for (const auto& square : path)
	{
		//image.at<uchar>(square->getY(), square->getX()) = 0;
		tmp.x = square->getX(); tmp.y = square->getY();
		output.push_back(tmp);
	}
	clock_t time3 = clock();
	double timePath = (time3 - time2) / (double)CLOCKS_PER_SEC;
	std::cout << "timePath:" << timePath << " seconds" << std::endl;
	
	return isFound;
}
