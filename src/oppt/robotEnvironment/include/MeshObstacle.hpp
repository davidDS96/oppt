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
#ifndef MESH_OBSTACLE_HPP_
#define MESH_OBSTACLE_HPP_
#include "Obstacle.hpp"
#include "oppt/opptCore/geometric/Mesh.hpp"

namespace oppt
{

class MeshObstacle: public ObstacleImpl
{
public:
    MeshObstacle(std::string name,
                 VectorString& meshFiles,
                 Matrixdf& pose,
                 VectorFloat& scale,
                 bool loadFromFile = true);

    virtual ObstacleSharedPtr clone() const override;

    virtual void createCollisionObject() override;

    void removeVertices(std::vector<VectorFloat>& vertices);

private:
    fcl::Transform3f identity_transform_;

    VectorString meshFiles_;

    VectorFloat scale_;
};

}

#endif /* OBSTACLE_HPP_ */
