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
#include "include/BoxObstacle.hpp"
#include "oppt/opptCore/geometric/Box.hpp"
#include <iostream>

using std::cout;
using std::endl;

using std::min;
using std::max;

using namespace fcl;

namespace oppt
{

BoxObstacle::BoxObstacle(std::string name,
                         Matrixdf& pose,
                         VectorFloat& dimensions):
    ObstacleImpl(name)
{
    geometry_ = std::make_shared<geometric::Box>(name, dimensions, pose);    
    createCollisionObject();
}

ObstacleSharedPtr BoxObstacle::clone() const
{
    VectorFloat dimensions = static_cast<geometric::Box*>(geometry_.get())->getDimensions(); 
    VectorFloat color = getColor();
    auto pose = geometry_->getPose();
    ObstacleSharedPtr clonedObstacle = std::make_shared<BoxObstacle>(name_,
                                       pose,
                                       dimensions);
    clonedObstacle->setStatic(static_);
    clonedObstacle->setEnabled(enabled_);
    clonedObstacle->setColor(color);
    return clonedObstacle;
}

void BoxObstacle::createCollisionObject()
{
    Matrixdf pose = geometry_->getPose();
    fcl::Matrix3f trans_matrix(pose(0, 0), pose(0, 1), pose(0, 2),
                               pose(1, 0), pose(1, 1), pose(1, 2),
                               pose(2, 0), pose(2, 1), pose(2, 2));
    fcl::Vec3f trans_vec(pose(0, 3), pose(1, 3), pose(2, 3));
    fcl::Transform3f transform(trans_matrix, trans_vec);
    CollisionGeometrySharedPtr collisionGeometry = geometry_->getCollisionGeometry();
    collisionObject_ = CollisionObjectSharedPtr(new fcl::CollisionObject(collisionGeometry, transform));    
}

}
