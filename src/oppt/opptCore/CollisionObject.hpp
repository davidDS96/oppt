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
#ifndef _OPPT_COLLISION_OBJECT_HPP_
#define _OPPT_COLLISION_OBJECT_HPP_
#include "typedefs.hpp"

namespace oppt {
    /**
 * Wrapper class around a oppt::CollisionObjectSharedPtr
 */
class OpptCollisionObject
{
public:
    /**
     * @brief Construct from a oppt::CollisionObjectSharedPtr and a name for the collision object
     */
    OpptCollisionObject(CollisionObjectSharedPtr& collisionObject, const std::string& name);
    
    /**
     * @brief Get the underlying oppt::CollisionObjectSharedPtr
     */
    CollisionObjectSharedPtr getCollisionObject() const;
    
    /**
     * @brief Get the name of this oppt::OpptCollisionObject
     */
    std::string getName() const;
    
    /**
     * @brief Check if the collision objects provided in otherCollisionObjects collide with this collision object
     * @param otherCollisionObjects std::vector of oppt::OpptCollisionObjectSharedPtr for which the collision check is performed
     * @return true iff at least one of the other collision objects collide with this collision object
     */
    CollisionReportSharedPtr collides(const CollisionRequestSharedPtr &collisionRequest) const;

private:
    CollisionObjectSharedPtr collisionObject_;

    std::string name_;
};
}

#endif