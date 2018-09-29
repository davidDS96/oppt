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
#include "GazeboSubscriber.hpp"
#include "oppt/gazeboInterface/GazeboInterface.hpp"

using namespace oppt;

GazeboSubscriber::GazeboSubscriber(GazeboInterface* gazeboInterface, const std::string& worldName):
    gazeboInterface_(gazeboInterface),
    worldName_(worldName),
    node_(gazebo::transport::NodePtr(new gazebo::transport::Node()))
{
    node_->Init("GazeboSubscriber");
    std::string topicStringRequest = "/gazebo/" + worldName_ + "/request";
    std::string topicStringModify = "/gazebo/" + worldName_ + "/model/modify";
    requestSubscriber_ = node_->Subscribe(topicStringRequest, &GazeboSubscriber::onRequest, this, true);
    modifySubscriber_ = node_->Subscribe(topicStringModify, &GazeboSubscriber::onModify, this, true);
}

GazeboSubscriber::~GazeboSubscriber()
{
    processingSuppressed_ = true;
    requestSubscriber_.reset();
    modifySubscriber_.reset();
    node_.reset();
}

void GazeboSubscriber::setRemoveObstacleCallback(RemoveObstacleCallback& removeObstacleCallback)
{
    removeObstacleCallback_ = removeObstacleCallback;
}

void GazeboSubscriber::setObstaclePoseChangedCallback(PoseChangedCallback& poseChangedCallback)
{
    poseChangedCallback_ = poseChangedCallback;
}

void GazeboSubscriber::suppressProcessing(const bool& suppress)
{
    processingSuppressed_ = suppress;
}

void GazeboSubscriber::onRequest(ConstRequestPtr& msg)
{
    if (!processingSuppressed_ && removeObstacleCallback_) {
        if (msg->request() == "entity_delete") {
            //LOGGING("ENTITY DELETE: " + msg->data());
            auto data = msg->data();
            if (data.find("__COLLISION_VISUAL__") != std::string::npos) {
                VectorString elems;
                split(data, "::", elems);
                removeObstacleCallback_(elems[0]);
            }
        }
    }
}

void GazeboSubscriber::onModify(ConstModelPtr& _msg)
{
    if (!processingSuppressed_ && poseChangedCallback_) {
        if (_msg->has_pose()) {
            std::string name = _msg->name();
	    LOGGING("ON MODIFY " + name);
            auto pose = _msg->pose();
            VectorFloat poseVec(7);
            poseVec[0] = pose.position().x();
            poseVec[1] = pose.position().y();
            poseVec[2] = pose.position().z();

            poseVec[3] = pose.orientation().x();
            poseVec[4] = pose.orientation().y();
            poseVec[5] = pose.orientation().z();
            poseVec[6] = pose.orientation().w();
            gazeboInterface_->changeModelPose(name, poseVec);
            poseChangedCallback_(name, poseVec);
        }
    }
}
