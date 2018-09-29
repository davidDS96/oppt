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
#include "include/SceneImpl.hpp"
#include "include/Obstacle.hpp"
#include "oppt/opptCore/geometric/Geometry.hpp"
#include "oppt/opptCore/CollisionObject.hpp"
#include <iostream>

namespace oppt
{

SceneImpl::SceneImpl():
    Scene()
{
    sceneCollisionManager_->clear();
    sceneCollisionManager_->setup();
    robotCollisionManager_->clear();
    robotCollisionManager_->setup();

}

SceneImpl::~SceneImpl()
{
    sceneCollisionManager_->clear();
    robotCollisionManager_->clear();
    sceneCollisionManager_.reset();
    robotCollisionManager_.reset();
}

SceneSharedPtr SceneImpl::clone() const
{
    SceneSharedPtr clonedScene = std::make_shared<SceneImpl>();
    for (auto & obstacle : obstacles_) {
        auto clonedObstacle = obstacle->clone();
        clonedScene->addObstacle(clonedObstacle);
    }

    return clonedScene;
}

void SceneImpl::addObstacle(oppt::ObstacleSharedPtr& obstacle, bool executeCallback)
{
    ObstacleSharedPtr obst = getObstacle(obstacle->getName());
    if (obst)
        removeObstacle(obst->getName());

    obstacles_.push_back(obstacle);
    if (obstacle->isEnabled()) {
        obstaclesInManager_.push_back(obstacle->getName());
        sceneCollisionManager_->registerObject(obstacle->getCollisionObject().get());
        sceneCollisionManager_->update();
        collisionObjectMap_[obstacle->getCollisionObject().get()] = obstacle->getName();
    }

    if (executeCallback) {
        if (addObstacleCallback_) {
            addObstacleCallback_(obstacle);
            return;
        }
    }
}

void SceneImpl::addObstacles(VectorObstaclePtr& obstacles, bool executeCallback)
{
    for (auto & obstacle : obstacles) {
        addObstacle(obstacle, executeCallback);
    }

}

bool SceneImpl::removeObstacle(std::string obstacleName, bool executeCallback)
{
    ObstacleSharedPtr obstacle = getObstacle(obstacleName);
    if (!obstacle) {
        WARNING("Can't remove obstacle with name '" + obstacleName + "'");
        return false;
    }
    for (size_t i = 0; i < obstacles_.size(); i++) {
        bool found = false;
        std::string obstName = obstacles_[i]->getName();
        if (obstacles_[i]->getName() == obstacleName) {
            found = true;
        } else if (obstName.find("_nestLink_") != std::string::npos && obstName.find(obstacleName) != std::string::npos) {
            found = true;
        }

        if (found) {
            int managedIndex = obstacleManaged(obstacle->getName());
            if (managedIndex > -1) {
                sceneCollisionManager_->unregisterObject(obstacles_[i]->getCollisionObject().get());
                sceneCollisionManager_->update();
                obstaclesInManager_.erase(obstaclesInManager_.begin() + managedIndex);
            }

            if (collisionObjectMap_.find(obstacles_[i]->getCollisionObject().get()) != collisionObjectMap_.end())
                collisionObjectMap_.erase(obstacles_[i]->getCollisionObject().get());
            obstacles_.erase(obstacles_.begin() + i);
            if (executeCallback) {
                if (removeObstacleCallback_) {
                    removeObstacleCallback_(obstacleName);
                    return true;
                }
            }

            break;
        }
    }

    return true;
}

bool SceneImpl::removeObstacles(std::vector<std::string>& obstacle_names, bool executeCallback)
{
    for (auto & k : obstacle_names) {
        removeObstacle(k, executeCallback);
    }

    return true;
}

bool SceneImpl::changeObstaclePose(const std::string& name, const VectorFloat& poseVec, bool executeCallback)
{
    ObstacleSharedPtr obstacle = getObstacle(name);
    if (!obstacle) {
        WARNING("Can't change pose of obstacle '" + name + "'");
    }
    bool poseChanged = false;
    if (obstacle) {
        int managedIndex = obstacleManaged(name);
        Matrix3f rotMatrix = Quaternionf(poseVec[6], poseVec[3], poseVec[4], poseVec[5]).toRotationMatrix();
        Matrixdf poseMatr = Matrixdf::Identity(4, 4);
        poseMatr.block<3, 3>(0, 0) = rotMatrix;
        poseMatr(0, 3) = poseVec[0];
        poseMatr(1, 3) = poseVec[1];
        poseMatr(2, 3) = poseVec[2];
        if (managedIndex > -1) {
            poseChanged = true;
            sceneCollisionManager_->unregisterObject(obstacle->getCollisionObject().get());
            obstacle->getGeometry()->setPose(poseMatr);
            obstacle->createCollisionObject();
            sceneCollisionManager_->registerObject(obstacle->getCollisionObject().get());
            sceneCollisionManager_->update();
            collisionObjectMap_[obstacle->getCollisionObject().get()] = obstacle->getName();
        } else {
            obstacle->getGeometry()->setPose(poseMatr);
        }

        if (executeCallback) {
            if (changePoseCallback_)
                changePoseCallback_(name, poseVec);
        }
    } else {
        WARNING("Trying to change pose of obstacle '" + name + "' in scene, but obstacle not present. Skipping...");
    }

    return poseChanged;
}

VectorObstaclePtr SceneImpl::getObstacles() const
{
    return obstacles_;
}

oppt::ObstacleSharedPtr SceneImpl::getObstacle(std::string name) const
{
    for (size_t i = 0; i < obstacles_.size(); i++) {
        auto n = obstacles_[i]->getName();
        if (n == name) {
            return obstacles_[i];
        } else if (n.find("_nestLink_") != std::string::npos && n.find(name) != std::string::npos) {
            return obstacles_[i];
        }
    }

    return nullptr;
}

int SceneImpl::obstacleManaged(const std::string& obstacleName) const
{
    for (size_t i = 0; i < obstaclesInManager_.size(); i++) {
        auto n = obstaclesInManager_[i];
        if (n == obstacleName) {
            return i;
        } else if (n.find("_nestLink_") != std::string::npos && n.find(obstacleName) != std::string::npos) {
            return i;
        }
    }

    return -1;
}

void SceneImpl::makeDiscreteCollisionReport(oppt::CollisionReportSharedPtr& collisionReport)
{
    oppt::DiscreteCollisionReport* discreteCollisionReport =
        static_cast<oppt::DiscreteCollisionReport*>(collisionReport.get());
    fcl::CollisionRequest collisionRequest;
    collisionRequest.enable_contact = discreteCollisionReport->enableContact;
    collisionRequest.num_max_contacts = 10;
    fcl::CollisionResult collisionResult;
    CollisionData collisionData;
    collisionData.request = collisionRequest;
    collisionData.result = collisionResult;
    robotCollisionManager_->update();
    sceneCollisionManager_->collide(robotCollisionManager_.get(), &collisionData, defaultCollisionFunction);
    discreteCollisionReport->collides = collisionData.result.isCollision();
    for (size_t i = 0; i != collisionData.o1.size(); ++i) {
        discreteCollisionReport->collisionPairs.push_back(std::make_pair(collisionObjectMap_.at(collisionData.o1[i]),
                collisionObjectMap_.at(collisionData.o2[i])));
    }
}

void SceneImpl::makeContinuousCollisionReport(oppt::CollisionReportSharedPtr& collisionReport)
{
    oppt::ContinuousCollisionReport* continuousCollisionReport =
        static_cast<oppt::ContinuousCollisionReport*>(collisionReport.get());
    fcl::ContinuousCollisionRequest request(continuousCollisionReport->numIterations,
                                            0.0001,
                                            fcl::CCDM_LINEAR,
                                            fcl::GST_LIBCCD,
                                            fcl::CCDC_NAIVE);
    std::vector<fcl::CollisionObject*> robotCollisionObjects;
    robotCollisionManager_->getObjects(robotCollisionObjects);
    for (size_t i = 0; i < obstacles_.size(); i++) {
        if (!obstacles_[i]->isEnabled()) {
            continue;
        }
        CollisionObjectSharedPtr obstacleCollisionObject = obstacles_[i]->getCollisionObject();
        for (size_t j = 0; j < continuousCollisionReport->tfGoal.size(); j++) {
            fcl::ContinuousCollisionResult result;
            fcl::continuousCollide(robotCollisionObjects_[j]->getCollisionObject().get(),
                                   continuousCollisionReport->tfGoal[j],
                                   obstacleCollisionObject.get(),
                                   obstacleCollisionObject->getTransform(),
                                   request,
                                   result);
            if (result.is_collide) {
                continuousCollisionReport->collides = true;
                continuousCollisionReport->collidingObstacles[0] = obstacles_[i]->getName();
                continuousCollisionReport->collidingBodyIndex = j;
                continuousCollisionReport->collidingBody = robotCollisionObjects_[j]->getName();
                return;
            }
        }
    }
}

void SceneImpl::makeExtendedCollisionReport(oppt::ExtendedCollisionReportSharedPtr& collisionReport) const
{
    fcl::ContinuousCollisionRequest request(20,
                                            0.0001,
                                            fcl::CCDM_LINEAR,
                                            fcl::GST_LIBCCD,
                                            fcl::CCDC_NAIVE);
    for (size_t i = 0; i < obstacles_.size(); i++) {
        if (!obstacles_[i]->isEnabled()) {
            continue;
        }
        CollisionObjectSharedPtr obstacleCollisionObject = obstacles_[i]->getCollisionObject();
        for (size_t j = 0; j < collisionReport->robotCollisionObjectsStart.size(); j++) {
            fcl::ContinuousCollisionResult result;
            fcl::continuousCollide(collisionReport->robotCollisionObjectsStart[j]->getCollisionObject().get(),
                                   collisionReport->robotCollisionObjectsGoal[j]->getCollisionObject()->getTransform(),
                                   obstacleCollisionObject.get(),
                                   obstacleCollisionObject->getTransform(),
                                   request,
                                   result);
            if (result.is_collide) {
                collisionReport->timeOfContact = result.time_of_contact;
                collisionReport->collides = true;
                collisionReport->collidingObstacles[0] = obstacles_[i]->getName();
                collisionReport->collidingBodyIndex = j;
                collisionReport->collidingBody = collisionReport->robotCollisionObjectsGoal[j]->getName();
                return;
            }

        }
    }
}

void SceneImpl::setRemoveObstacleCallback(std::function<void(std::string)> callbackFunction)
{
    removeObstacleCallback_ = callbackFunction;
}

void SceneImpl::setAddObstacleCallback(std::function<void(oppt::ObstacleSharedPtr&)> callbackFunction)
{
    addObstacleCallback_ = callbackFunction;
}

void SceneImpl::setChangeObstaclePoseCallback(std::function<void(const std::string&, const VectorFloat&)> callbackFunction)
{
    changePoseCallback_ = callbackFunction;
}

void SceneImpl::setRobotCollisionObjects(VectorCollisionObjects& robotCollisionObjects)
{
    robotCollisionObjects_ = robotCollisionObjects;
    robotCollisionManager_->clear();
    for (auto & robotCollisionObject : robotCollisionObjects_) {
        robotCollisionManager_->registerObject(robotCollisionObject->getCollisionObject().get());
        collisionObjectMap_[robotCollisionObject->getCollisionObject().get()] = robotCollisionObject->getName();
    }
}

void SceneImpl::setCollisionInvariantRobotCollisionObjects(VectorCollisionObjects &robotCollisionObjects) {
    collisionInvariantRobotCollisionObjects_ = robotCollisionObjects;
}

VectorCollisionObjects SceneImpl::getCollisionInvariantRobotCollisionObjects() const {
    return collisionInvariantRobotCollisionObjects_;
}

VectorCollisionObjects SceneImpl::getRobotCollisionObjects() const
{
    return robotCollisionObjects_;
}

std::string SceneImpl::toSDFString() const
{
    std::string sdfString = "";
    for (auto & obstacle : obstacles_) {
        sdfString += obstacle->toSDFString();
    }

    return sdfString;
}

}
