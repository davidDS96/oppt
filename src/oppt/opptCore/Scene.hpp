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
#ifndef _SCENE_HPP_
#define _SCENE_HPP_
#include "typedefs.hpp"

namespace oppt
{
/**
* Virtual base class for a scene, which is a geometric
* representation of the environment the robot operates in
*/
class Scene
{
public:
    Scene():
        obstacles_(),
        robotCollisionObjects_(),
        obstaclesInManager_(),
        sceneCollisionManager_(new fcl::DynamicAABBTreeCollisionManager()),
        robotCollisionManager_(new fcl::DynamicAABBTreeCollisionManager()) {
    }

    /** @brief Default destructor */
    virtual ~Scene() = default;

    /**
     * @brief Clone this scene and return a oppt::SceneSharedPtr to the cloned scene
     */
    virtual SceneSharedPtr clone() const = 0;

    /**
     * Adds an obstacle to the scene.
     * @param obstacle A oppt::ObstacleSharedPtr to the obstacle that is being added to the scene
     * @param executeCallback If true, the callback that is set via the oppt::Scene::setAddObstacleCallback is called
     */
    virtual void addObstacle(oppt::ObstacleSharedPtr& obstacle, bool executeCallback = true) = 0;

    /**
     * Adds multiple obstacles to the scene.
     * @param obstacles A oppt::VectorObstaclePtr containing of the obstacles that are being added to the scene
     * @param executeCallback If true, the callback that is set via the oppt::Scene::setAddObstacleCallback is called
     */
    virtual void addObstacles(VectorObstaclePtr& obstacles, bool executeCallback = true) = 0;

    /**
     * Removes and obstacle from the scene
     * @param obstacleName The name of the obstacle that is being removed. If an obstacle with this name doesn't exist,
     * it is being ignored
     * @param executeCallback If true, the callback that is set via the oppt::Scene::setRemoveObstacleCallback is called
     */
    virtual bool removeObstacle(std::string obstacleName, bool executeCallback = true) = 0;

    /**
     * Removes multiple obstacles from the scene
     * @param obstacleName The names of the obstacles that are being removed.
     * @param executeCallback If true, the callback that is set via the oppt::Scene::setRemoveObstacleCallback is called
     */
    virtual bool removeObstacles(std::vector<std::string>& obstacle_names, bool executeCallback = true) = 0;

    /**
     * Change the pose of an obstacle
     * @param name The name of the obstacle whose pose is being changed
     * @param poseVec A oppt::VectorFloat consisting of the position and euler angles of the new pose
     * @param executeCallback If true, the callback that is set via the oppt::Scene::setChangeObstaclePoseCallback is called
     */
    virtual bool changeObstaclePose(const std::string& name, const VectorFloat& poseVec, bool executeCallback = true) = 0;

    /**
     * @brief Get a oppt::VectorObstaclePtr containing all obstacles in this scene
     */
    virtual VectorObstaclePtr getObstacles() const = 0;

    /**
     * Get a oppt::ObstacleSharedPtr the obstacle with the given name.
     * If no such obstacle exists, a nullptr is returned
     */
    virtual oppt::ObstacleSharedPtr getObstacle(std::string name) const  = 0;

    /**
     * Performs a discrete collision check on the scene based on a oppt::CollisionReportSharedPtr
     */
    virtual void makeDiscreteCollisionReport(oppt::CollisionReportSharedPtr& collisionReport) = 0;

    /**
     * Performs a continuous collision check on the scene based on a oppt::CollisionReportSharedPtr
     */
    virtual void makeContinuousCollisionReport(oppt::CollisionReportSharedPtr& collisionReport) = 0;

    virtual void makeExtendedCollisionReport(oppt::ExtendedCollisionReportSharedPtr& collisionReport) const = 0;

    /**
     * @brief Set the callback function this is being called when an obstacle is removed
     */
    virtual void setRemoveObstacleCallback(std::function<void(std::string)> callbackFunction) = 0;

    /**
     * @brief Set the callback function this is being called when an obstacle is added
     */
    virtual void setAddObstacleCallback(std::function<void(oppt::ObstacleSharedPtr&)> callbackFunction) = 0;

    /**
     * @brief Set the callback function this is being called the pose of an obstacle is changed
     */
    virtual void setChangeObstaclePoseCallback(std::function<void(const std::string&, const VectorFloat&)> callbackFunction) = 0;

    /**
     * Set the robot collision objects that are being collision-checked against the obstacles in the scene
     * when oppt::Scene::makeDiscreteCollisionReport or oppt::Scene::makeContinuousCollisionReport is called
     */
    virtual void setRobotCollisionObjects(VectorCollisionObjects& robotCollisionObjects) = 0;

    /**
     * Set the robot collision objects that are ingnored during collision checking
     */
    virtual void setCollisionInvariantRobotCollisionObjects(VectorCollisionObjects &robotCollisionObjects) = 0;

    /**
     * @brief Return the robot collision objects managed by this scene
     */
    virtual VectorCollisionObjects getRobotCollisionObjects() const = 0;

    /**
     * Get the robot collision objects that are ingnored during collision checking
     */
    virtual VectorCollisionObjects getCollisionInvariantRobotCollisionObjects() const = 0;

    /**
     * @brief Returns an SDF string of the scene
     */
    virtual std::string toSDFString() const = 0;

protected:
    VectorObstaclePtr obstacles_;

    VectorCollisionObjects robotCollisionObjects_;

    VectorCollisionObjects collisionInvariantRobotCollisionObjects_;

    VectorString obstaclesInManager_;

    std::function<void(std::string)> removeObstacleCallback_ = nullptr;

    std::function<void(oppt::ObstacleSharedPtr&)> addObstacleCallback_ = nullptr;

    std::function<void(const std::string&, const VectorFloat&)> changePoseCallback_ = nullptr;

    std::shared_ptr<fcl::BroadPhaseCollisionManager> sceneCollisionManager_;

    std::shared_ptr<fcl::BroadPhaseCollisionManager> robotCollisionManager_;

    virtual int obstacleManaged(const std::string& obstacleName) const = 0;

};
}

#endif
