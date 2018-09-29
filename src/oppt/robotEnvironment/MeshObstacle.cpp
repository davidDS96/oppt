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
#include "include/MeshObstacle.hpp"
#include "oppt/opptCore/geometric/Mesh.hpp"
#include <iostream>

using std::cout;
using std::endl;

using std::min;
using std::max;

using namespace fcl;

namespace oppt
{
MeshObstacle::MeshObstacle(std::string name,
                           VectorString& meshFiles,
                           Matrixdf& pose,
                           VectorFloat& scale,
                           bool loadFromFile):
    ObstacleImpl(name),
    meshFiles_(meshFiles),
    identity_transform_(),
    scale_({1.0, 1.0, 1.0})
{
    geometry_ = std::make_shared<geometric::Mesh>(name, meshFiles[0], pose, scale);    
    createCollisionObject();
}

ObstacleSharedPtr MeshObstacle::clone() const
{
    VectorString clonedMeshFiles = meshFiles_;
    VectorFloat clonedScale = scale_;    
    auto pose = geometry_->getPose();
    VectorFloat color = getColor();
    ObstacleSharedPtr clonedObstacle = std::make_shared<MeshObstacle>(name_,
                                       clonedMeshFiles,
                                       pose,
                                       clonedScale,
                                       true);
    clonedObstacle->setStatic(static_);
    clonedObstacle->setEnabled(enabled_);
    clonedObstacle->setColor(color);
    return clonedObstacle;
}

void MeshObstacle::removeVertices(std::vector<VectorFloat>& vertices)
{
    createCollisionObject();
}

void MeshObstacle::createCollisionObject()
{
    Matrixdf pose = geometry_->getPose();
    fcl::Vec3f translation(pose(0, 3), pose(1, 3), pose(2, 3));
    fcl::Matrix3f rotation(pose(0, 0), pose(0, 1), pose(0, 2),
                           pose(1, 0), pose(1, 1), pose(1, 2),
                           pose(2, 0), pose(2, 1), pose(2, 2));
    fcl::Transform3f transf(rotation, translation);
    CollisionGeometrySharedPtr collisionGeometry = geometry_->getCollisionGeometry();
    collisionObject_ = CollisionObjectSharedPtr(new fcl::CollisionObject(collisionGeometry, transf));
}

}
