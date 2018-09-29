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
#include "include/Terrain.hpp"

namespace oppt
{

TerrainImpl::TerrainImpl(const std::string name,
                         const FloatType traversalCost,
                         const FloatType velocityDamping,
                         bool traversable,
                         bool observable):
    Terrain(traversable, observable),
    name_(name),
    traversalCost_(traversalCost),
    velocityDamping_(velocityDamping)
{

}

TerrainSharedPtr TerrainImpl::clone() const
{
    TerrainSharedPtr clonedTerrain = std::make_shared<TerrainImpl>(name_, 
								   traversalCost_, 
								   velocityDamping_, 
								   traversable_, 
								   observable_);
    clonedTerrain->setColor(color_);
    return clonedTerrain;
}

const std::string TerrainImpl::getName() const
{
    return name_;
}

const FloatType TerrainImpl::getTraversalCost() const
{
    return traversalCost_;
}

const FloatType TerrainImpl::getVelocityDamping() const
{
    return velocityDamping_;
}

}
