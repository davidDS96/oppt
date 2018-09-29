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
#ifndef BOX_OBSTACLE_HPP_
#define BOX_OBSTACLE_HPP_
#include "Obstacle.hpp"

namespace oppt
{

class BoxObstacle: public ObstacleImpl
{
public:
    BoxObstacle(std::string name,
                Matrixdf &pose,
                VectorFloat &dimensions);
    
    virtual ObstacleSharedPtr clone() const override;

    virtual void createCollisionObject() override;

};

}

#endif /* OBSTACLE_HPP_ */
