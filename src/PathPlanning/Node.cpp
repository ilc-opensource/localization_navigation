/*
 * License: 3-clause BSD. See LICENSE file in root directory.
 * Copyright(c) 2015-2016 Intel Corporation. All Rights Reserved.
 * */

#include "Node.h"

Node::Node() :
	m_parent(nullptr)
{}

Node::~Node()
{}

Node* Node::getParent() const
{
	return m_parent;
}

std::vector<std::pair<Node*, float>>& Node::getChildren()
{
	return m_children;
}

void Node::addChild(Node* child, float distance)
{
	m_children.push_back(std::make_pair(child,distance));
}

void Node::clearChildren()
{
	m_children.clear();
}

void Node::setParent(Node* parent)
{
	m_parent = parent;
}
