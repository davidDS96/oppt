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
#ifndef __OPPT_COLLISION_REPORT_HPP__
#define __OPPT_COLLISION_REPORT_HPP__
#include "typedefs.hpp"

namespace oppt
{

/**
 * Datastructure for collision queries
 */
class CollisionReport
{
public:

    /**
     * @brief Default constructor
     */
    CollisionReport() {}

    /**
     * @brief Boolean flag that determines if a collision was detected
     */
    bool collides = false;

    /**
     * @brief Vector of obstacle names the robot collided with
     */
    VectorString collidingObstacles;

    /**
     * @brief The index of the robot body that collided with an obstacle
     */
    unsigned int collidingBodyIndex = 0;

    /**
     * @brief Name of the robot body that collided with an obstacle
     */
    std::string collidingBody;
};

/**
 * Datastructure that represents a discrete collision report
 */
class DiscreteCollisionReport: public CollisionReport
{
public:
    /**
     * @brief Default constructor
     */
    DiscreteCollisionReport(): CollisionReport() {}

    /**
     * @brief Vector of oppt::OpptCollisionObject for which collision checking is performed
     */
    VectorCollisionObjects robotCollisionObjects;

    /**
     * @brief Flag that determines if contact information should be generated during the collision query
     */
    bool enableContact = false;

    /**
     * @brief oppt::VectorFloat containing the contact position (this vector is empty if enableContact is set to false)
     */
    std::vector<VectorFloat> contactPositions;

    /**
     * @brief oppt::VectorFloat containing the contact normal (this vector is empty if enableContact is set to false)
     */
    std::vector<VectorFloat> contactNormals;

    VectorFloat penetrationDepths;
    
    std::vector<std::pair<std::string, std::string>> collisionPairs;
};

/**
 * Datastructure that represents a continuous collision report
 */
class ContinuousCollisionReport: public CollisionReport
{
public:
    /**
     * @brief Default constructor
     */
    ContinuousCollisionReport(): CollisionReport() {}
    
    unsigned int numIterations = 10;

    /**
     * oppt::VectorFCLTransform3f that contains the goal transformations of the robot collision objects
     */
    VectorFCLTransform3f tfGoal;
};

/**
 * Datastructure that represents an extended collision report
 */
class ExtendedCollisionReport: public CollisionReport
{
public:
    /**
     * @brief Default constructor
     */
    ExtendedCollisionReport():
        CollisionReport() {

    }

    /**
     * @brief Contains the oppt::VectorCollisionObjects at the start state of the robot
     */
    VectorCollisionObjects robotCollisionObjectsStart;

    /**
     * @brief Contains the oppt::VectorCollisionObjects at the goal state of the robot
     */
    VectorCollisionObjects robotCollisionObjectsGoal;

    /**
     * @brief Time (between 0 and 1) of contact
     */
    FloatType timeOfContact;

    std::string contactBodyName;

};

}

#endif
