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
/** @file ManipulatorState.hpp
 *
 * Defines the RockSampleState class, which represents a state of the RockSample problem.
 */
#ifndef SPHERE_OBSTACLE_HPP_
#define SPHERE_OBSTACLE_HPP_
#include "Obstacle.hpp"

namespace oppt
{

class SphereObstacle: public ObstacleImpl
{
public:
    SphereObstacle(std::string name,
                   Matrixdf& pose,
                   FloatType radius);

    virtual ObstacleSharedPtr clone() const override;

    virtual void createCollisionObject() override;

private:
    FloatType radius_;
};

}

#endif /* OBSTACLE_HPP_ */
