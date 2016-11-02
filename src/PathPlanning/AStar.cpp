/*
 * License: 3-clause BSD. See LICENSE file in root directory.
 * Copyright(c) 2015-2016 Intel Corporation. All Rights Reserved.
 * */

#include "AStar.h"
#include<functional>

AStar::AStar()
{}

AStar::~AStar()
{}

bool AStar::getPath(AStarNode* start, AStarNode* goal, std::vector<AStarNode*>& path)
{
	AStarNode *currentNode, *childNode;
	float f, g, h;


	open.clear();
	closed.clear();
	std::make_heap(open.begin(), open.end(), CompareNodes2());
	pushOpen(start);

	while(!open.empty())
	{



		std::sort(open.begin(), open.end(), CompareNodes());
		//std::make_heap(open.begin(), open.end(), CompareNodes2());

		currentNode = open.front(); // pop n node from open for which f is minimal
		popOpen(currentNode);

		currentNode->setClosed(true);
		closed.push_back(currentNode);




		if(currentNode == goal)
		{
			reconstructPath(currentNode, path);
			return true;
		}

		for(const auto& children : currentNode->getChildren() )// for each successor n' of n
		{
			childNode = static_cast<AStarNode*>(children.first);
			g = currentNode->getG() + children.second; // stance from start + distance between the two nodes
			if( (childNode->isOpen() || childNode->isClosed()) && childNode->getG() <  g) // n' is already in opend or closed with a lower cost g(n')
				continue; // consider next successor

			h = distanceBetween(childNode, goal);
			f = g + h; // compute f(n')
			childNode->setF(f);
			childNode->setG(g);
			childNode->setH(h);
			childNode->setParent(currentNode);

			if(childNode->isClosed())
				childNode->setClosed(false);
			if(!childNode->isOpen())
				pushOpen(childNode);
		}
	}


	return false;
}

void AStar::pushOpen(AStarNode* node)
{
	open.push_back(node);
	std::sort(open.begin(), open.end(), CompareNodes());
	std::push_heap(open.begin(), open.end(), CompareNodes2());
	node->setOpen(true);
}

void AStar::popOpen(AStarNode* node)
{
	std::pop_heap(open.begin(), open.end(), CompareNodes2());
	open.pop_back();
	node->setOpen(false);
	open.end();
}

void AStar::releaseNodes()
{
	for(const auto& node : open)
		node->release();
	for(const auto& node : closed)
		node->release();
}

void AStar::clear()
{
	releaseNodes();
	open.clear();
	closed.clear();
}
