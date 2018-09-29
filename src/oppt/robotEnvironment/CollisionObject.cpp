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
#include "oppt/opptCore/CollisionObject.hpp"
#include "oppt/opptCore/CollisionRequest.hpp"
#include "oppt/opptCore/CollisionReport.hpp"

using std::cout;
using std::endl;

namespace oppt
{

OpptCollisionObject::OpptCollisionObject(CollisionObjectSharedPtr& collisionObject, const std::string& name):
    collisionObject_(collisionObject), name_(name)
{

}

CollisionObjectSharedPtr OpptCollisionObject::getCollisionObject() const
{
    return collisionObject_;
}

std::string OpptCollisionObject::getName() const
{
    return name_;
}

CollisionReportSharedPtr OpptCollisionObject::collides(const CollisionRequestSharedPtr &collisionRequest) const
{
    bool collides = false;
    CollisionReportSharedPtr collisionReport(new DiscreteCollisionReport());
    collisionReport->collides = false;
    for (size_t i = 0; i != collisionRequest->collisionObjects.size(); i++) {
        fcl::CollisionRequest request;
        fcl::CollisionResult result;
        request.enable_contact = collisionRequest->enableContact;
        request.num_max_contacts = collisionRequest->maxNumContacts;
        fcl::collide(collisionObject_.get(),
                     collisionRequest->collisionObjects[i]->getCollisionObject().get(),
                     request,
                     result);
        if (result.isCollision()) {
            collisionReport->collides = true;
            if (request.enable_contact) {
                std::vector<fcl::Contact> contacts;
                result.getContacts(contacts);
                auto dC = static_cast<DiscreteCollisionReport *>(collisionReport.get());
                dC->contactPositions =
                    std::vector<VectorFloat>(contacts.size());
                dC->contactNormals =
                    std::vector<VectorFloat>(contacts.size());
                dC->penetrationDepths = VectorFloat(contacts.size());
                for (size_t j = 0; j != contacts.size(); ++j) {
                    dC->contactPositions[j] =
                        VectorFloat({contacts[j].pos[0], contacts[j].pos[1], contacts[j].pos[2]});
                    dC->contactNormals[j] =
                        VectorFloat({contacts[j].normal[0], contacts[j].normal[1], contacts[j].normal[2]});
                    dC->penetrationDepths[j] = contacts[j].penetration_depth;
                }
            }

            break;
        }
    }

    return collisionReport;
}

}
