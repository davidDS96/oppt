/**
 * Copyright 2017
 * 
 * This file is part of On-line POMDP Planning Toolkit (OPPT).
 * OPPT is free software: you can redistribute it and/or modify it under the terms of the 
 * GNU General Public License published by the Free Software Foundation, 
 * either version 2 of the License, or (at your option) any later version.
 * 
 * OPPT is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License along with OPPT. 
 * If not, see http://www.gnu.org/licenses/.
 */
#ifndef _RRT_TREE_HPP_
#define _RRT_TREE_HPP_
#include "oppt/opptCore/core.hpp"

namespace oppt
{

class Tree;

class Node
{
public:
    Node(Tree *tree, 
	 const Node* parent, 
	 const VectorFloat& values);
    
    ~Node();

    void addChild(const size_t &idx);

    const Node* getParent() const;

    VectorFloat getValues() const;
    
    void setRobotState(const RobotStateSharedPtr &robotState);
    
    const RobotStateSharedPtr getRobotState() const;

private:
    Tree *tree_;
    
    const Node* parent_;

    const VectorFloat values_;

    std::vector<size_t> children_;
    
    RobotStateSharedPtr robotState_ = nullptr;

};

typedef std::function<const FloatType(const FloatType*, 
                                      const FloatType*)> DistanceFunction;


class Tree
{
public:
    Tree();
    ~Tree();
    
    void reset();
    
    const Node* allocNode(const Node* parent, const RobotStateSharedPtr& state);
    
    size_t makeRoot(const RobotStateSharedPtr &values);
    
    const Node* nearestNeighbour(const RobotStateSharedPtr &q);
    
    const FloatType size() const;  
    
    void setDistanceFunction(DistanceFunction &distanceFunction);

private:
    std::vector<Node*> nodes_;
    
    size_t allocated_ = 0;
    
    DistanceFunction distanceFunction_ = nullptr;

};
}

#endif
