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
#include "include/SphereObstacle.hpp"
#include "oppt/opptCore/geometric/Sphere.hpp"
#include <iostream>

using std::cout;
using std::endl;

using namespace fcl;

namespace oppt
{

SphereObstacle::SphereObstacle(std::string name,
                               Matrixdf& pose,
                               FloatType radius):
    ObstacleImpl(name),
    radius_(radius)
{
    geometry_ = std::make_shared<geometric::Sphere>(name, radius, pose);    
    createCollisionObject();
}

ObstacleSharedPtr SphereObstacle::clone() const
{    
    auto pose = geometry_->getPose();
    VectorFloat color = getColor();    
    ObstacleSharedPtr clonedObstacle = std::make_shared<SphereObstacle>(name_,
                                       pose,
                                       radius_);
    clonedObstacle->setStatic(static_);
    clonedObstacle->setEnabled(enabled_);
    clonedObstacle->setColor(color);
    return clonedObstacle;
}

void SphereObstacle::createCollisionObject()
{
    Matrixdf pose = geometry_->getPose();
    fcl::Vec3f trans(pose(0, 3), pose(1, 3), pose(2, 3));
    fcl::Matrix3f rot(1.0, 0.0, 0.0,
                      0.0, 1.0, 0.0,
                      0.0, 0.0, 1.0);
    fcl::Transform3f transf(rot, trans);
    CollisionGeometrySharedPtr collisionGeometry = geometry_->getCollisionGeometry();
    collisionObject_ = CollisionObjectSharedPtr(new fcl::CollisionObject(collisionGeometry, transf));
}

}
