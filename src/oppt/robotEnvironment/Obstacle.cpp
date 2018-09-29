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
#include "include/Obstacle.hpp"
#include "oppt/opptCore/geometric/Geometry.hpp"
#include "oppt/opptCore/CollisionObject.hpp"
#include <iostream>

using std::cout;
using std::endl;

using std::min;
using std::max;

using namespace fcl;

namespace oppt
{

bool defaultCollisionFunction(fcl::CollisionObject* o1, fcl::CollisionObject* o2, void* cdata)
{
    CollisionData* cdata_ = static_cast<CollisionData*>(cdata);
    const fcl::CollisionRequest& request = cdata_->request;
    fcl::CollisionResult& result = cdata_->result;

    if (cdata_->done) return true;
    size_t currentNumContacts = result.numContacts();

    fcl::collide(o1, o2, request, result);

    if (result.numContacts() > currentNumContacts) {
        cdata_->o1.push_back(o1);
        cdata_->o2.push_back(o2);
        if (!(request.enable_contact)) {
            cdata_->done = true;
        } else {
            if (!request.enable_cost && (result.numContacts() >= request.num_max_contacts)) {
                cdata_->done = true;
            }
        }
    }

    return cdata_->done;
}

ObstacleImpl::ObstacleImpl(std::string& name):
    oppt::Obstacle(name)
{

}

std::string ObstacleImpl::toSDFString() const
{
    std::string serString = "";
    Matrixdf pose = geometry_->getPose();
    Matrix3f rotMatrix(pose.block<3, 3>(0, 0));
    Vector3f eulerAngles = math::rotationMatrixToEulerAngles(rotMatrix);
    //serString += "<sdf version='1.6'>";
    serString += "<model name=\"" + name_ + "\">";
    serString += "<pose frame=''>";
    serString += std::to_string(pose(0, 3)) + " ";
    serString += std::to_string(pose(1, 3)) + " ";
    serString += std::to_string(pose(2, 3)) + " ";
    serString += std::to_string(eulerAngles[0]) + " ";
    serString += std::to_string(eulerAngles[1]) + " ";
    serString += std::to_string(eulerAngles[2]);
    serString += "</pose>";
    serString += "<static>";
    if (static_) {
        serString += "1";
    } else {
        serString += "0";
    }
    serString += "</static>";
    serString += "<link name=\"" + name_ + "_link\">";
    if (enabled_) {
        serString += "<collision name='";
        serString += name_ + "_collision'>";
        serString += geometry_->toSDFString();
        serString += "</collision>";
    }

    serString += "<visual name='";
    serString += name_ + "_visual'>";
    serString += geometry_->toSDFString();
    serString += "</visual>";
    serString += "</link>";
    serString += "</model>";
    //serString += "</sdf>";
    return serString;
}

bool ObstacleImpl::collides(const std::vector<oppt::CollisionObjectSharedPtr>& collisionObjects,
                            unsigned int& collidingBodyIndex) const
{
    for (size_t i = 0; i != collisionObjects.size(); i++) {
        fcl::CollisionRequest request;
        fcl::CollisionResult result;
        fcl::collide(collisionObjects[i].get(),
                     collisionObject_.get(),
                     request,
                     result);
        if (result.isCollision()) {
            collidingBodyIndex = i;
            return true;
        }
    }

    return false;
}

bool ObstacleImpl::collides(const std::vector<oppt::ObstacleSharedPtr>& otherObstacles) const
{
    std::vector<oppt::CollisionObjectSharedPtr> collisionObjects(otherObstacles.size(), nullptr);
    for (size_t i = 0; i != otherObstacles.size(); ++i) {
        collisionObjects[i] = otherObstacles[i]->getCollisionObject();
    }

    unsigned int bodyIdx = 0;
    return collides(collisionObjects, bodyIdx);

    /**for (size_t i = 0; i != otherObstacles.size(); i++) {
        if (collisionObject_->getAABB().overlap(otherObstacles[i]->getCollisionObject()->getAABB())) {
            return true;
        }
    }

    return false;*/
}

bool ObstacleImpl::collides(const VectorFloat& point) const
{
    Vec3f p_vec(point[0], point[1], point[2]);
    return collisionObject_->getAABB().contain(p_vec);
}

FloatType ObstacleImpl::distance(const std::vector<oppt::CollisionObjectSharedPtr>& collisionObjects) const
{
    FloatType min_distance = std::numeric_limits<FloatType>::infinity();
    for (size_t i = 0; i != collisionObjects.size(); i++) {
        fcl::DistanceRequest request;
        fcl::DistanceResult result;
        fcl::distance(collisionObjects[i].get(),
                      collisionObject_.get(),
                      request,
                      result);
        if (result.min_distance < min_distance)
            min_distance = result.min_distance;

    }

    return min_distance;
}

bool ObstacleImpl::collidesContinuous(const oppt::CollisionObjectSharedPtr& collisionObjectStart,
                                      const oppt::CollisionObjectSharedPtr& collisionObjectGoal) const
{
    fcl::ContinuousCollisionRequest request(20,
                                            0.0001,
                                            CCDM_LINEAR,
                                            GST_LIBCCD,
                                            CCDC_NAIVE);
    fcl::ContinuousCollisionResult result;
    fcl::continuousCollide(collisionObjectStart.get(),
                           collisionObjectGoal->getTransform(),
                           collisionObject_.get(),
                           collisionObject_->getTransform(),
                           request,
                           result);
    return result.is_collide;
}

}
