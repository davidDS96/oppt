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
#ifndef _SCENE_IMPL_HPP_
#define _SCENE_IMPL_HPP_
#include "oppt/opptCore/Scene.hpp"

namespace oppt
{
/**
* Implementation of the geometric representation of the environment
* inside the oppt::RobotEnvironment
*/
class SceneImpl: public Scene
{
public:
    /** @brief Default constructor */
    SceneImpl();

    /** @brief Default constructor */
    virtual ~SceneImpl();

    virtual SceneSharedPtr clone() const override;

    virtual void addObstacle(oppt::ObstacleSharedPtr& obstacle, bool executeCallback = true) override;

    virtual void addObstacles(VectorObstaclePtr& obstacles, bool executeCallback = true) override;

    virtual bool removeObstacle(std::string obstacleName, bool executeCallback = true) override;

    virtual bool removeObstacles(std::vector<std::string>& obstacle_names, bool executeCallback = true) override;

    virtual bool changeObstaclePose(const std::string& name, const VectorFloat& poseVec, bool executeCallback = true) override;

    virtual VectorObstaclePtr getObstacles() const override;

    virtual ObstacleSharedPtr getObstacle(std::string name) const override;

    virtual void makeDiscreteCollisionReport(oppt::CollisionReportSharedPtr& collisionReport) override;    

    virtual void makeContinuousCollisionReport(oppt::CollisionReportSharedPtr& collisionReport) override;

    virtual void makeExtendedCollisionReport(oppt::ExtendedCollisionReportSharedPtr& collisionReport) const override;

    virtual void setRemoveObstacleCallback(std::function<void(std::string)> callbackFunction) override;

    virtual void setAddObstacleCallback(std::function<void(oppt::ObstacleSharedPtr&)> callbackFunction) override;

    virtual void setRobotCollisionObjects(VectorCollisionObjects& robotCollisionObjects) override;

    virtual void setCollisionInvariantRobotCollisionObjects(VectorCollisionObjects &robotCollisionObjects) override;

    virtual VectorCollisionObjects getRobotCollisionObjects() const override;

    virtual VectorCollisionObjects getCollisionInvariantRobotCollisionObjects() const override;

    virtual void setChangeObstaclePoseCallback(std::function<void(const std::string&, const VectorFloat&)> callbackFunction) override;

    virtual std::string toSDFString() const override;

protected:
    virtual int obstacleManaged(const std::string& obstacleName) const override;

    std::unordered_map<fcl::CollisionObject*, std::string> collisionObjectMap_;

};

}

#endif
