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
#ifndef __ROBOT_ENVIRONMENT_CHANGES_HPP__
#define __ROBOT_ENVIRONMENT_CHANGES_HPP__
#include "oppt/opptCore/core.hpp"
#include <queue>

namespace oppt
{
    
enum EnvironmentChangeType {
    OBSTACLE_ADDED,
    OBSTACLE_REMOVED,
    OBSTACLE_POSE_CHANGED
};

/**
 * An abstract class representing a change in the environment. Classes that implement this class 
 * have to implement the getType method
 */
class EnvironmentChange
{
public:
    EnvironmentChange(const bool &requiresCallback):
    requiresCallback_(requiresCallback){

    }
    
    template<class T>
    T* as() {
        return static_cast<T*>(this);
    }
    
    /**
     * @brief Returns true if this EnvironmentChange has to be applied to the underlying model of the GazeboInterface.
     * When the user changes the environment using the Gazebo client, this will return false
     */
    bool requiresCallback() const {
	return requiresCallback_;
    }

    /**
     * @brief Get the type of the EnvironmentChange
     */
    virtual EnvironmentChangeType getType() const = 0;
    
    /**
     * @brief Serialize the EnvironmentChange
     * 
     * @param step The step in which the change occured
     * @param os Stream to the output file
     */
    virtual void serialize(std::ofstream& os) const = 0;
    
private:
    bool requiresCallback_;
};

/**
 * Represents a change in the environment when an obstacle has been added
 */
class ObstacleAddedChange: public EnvironmentChange
{
public:
    ObstacleAddedChange(const std::string& sdfString, const bool &requiresCallback=false):
        EnvironmentChange(requiresCallback),
        sdfString_(sdfString){

    }

    virtual EnvironmentChangeType getType() const override {
        return EnvironmentChangeType::OBSTACLE_ADDED;
    }

    const std::string getSDFString() const {
        return sdfString_;
    }
    
    virtual void serialize(std::ofstream& os) const override {
	std::string sdfTrimmed = sdfString_;
	
	// Remove \n characters
	sdfTrimmed.erase(std::remove(sdfTrimmed.begin(), sdfTrimmed.end(), '\n'), sdfTrimmed.end());
	os << "add " << sdfTrimmed << endl;
    }

private:
    const std::string sdfString_;
};

/**
 * Represents a change in the environment when an obstacle has been removed
 */
class ObstacleRemovedChange: public EnvironmentChange
{
public:
    ObstacleRemovedChange(const std::string& obstacleName, const bool &requiresCallback=false):
        EnvironmentChange(requiresCallback),
        obstacleName_(obstacleName) {

    }

    virtual EnvironmentChangeType getType() const override {
        return EnvironmentChangeType::OBSTACLE_REMOVED;
    }

    const std::string getObstacleName() const {
        return obstacleName_;
    }
    
    virtual void serialize(std::ofstream& os) const override {
	os << "remove " << obstacleName_ << endl;
    }

private:
    const std::string obstacleName_;

};

/**
 * Represents a change in the environment when the pose of an obstacle has changed
 */
class ObstaclePoseChange: public EnvironmentChange
{
public:
    ObstaclePoseChange(const std::string& obstacleName, 
		       const VectorFloat& poseVec, 
		       const bool &requiresCallback=false):
        EnvironmentChange(requiresCallback),
        obstacleName_(obstacleName),
        poseVec_(poseVec) {

    }

    const std::string getObstacleName() const {
        return obstacleName_;
    }

    const VectorFloat getPoseVec() const {
        return poseVec_;
    }

    virtual EnvironmentChangeType getType() const override {
        return EnvironmentChangeType::OBSTACLE_POSE_CHANGED;
    }
    
    virtual void serialize(std::ofstream& os) const override {
	os << "changePose " << obstacleName_ << " ";
	for (size_t i = 0; i != poseVec_.size(); ++i) {
	    os << poseVec_[i] << " ";
	}
	
	os << endl;
    }

private:
    const std::string obstacleName_;
    const VectorFloat poseVec_;

};

/**
 * A simple wrapper class around a queue of environment changes
 */
class EnvironmentChanges
{
public:
    EnvironmentChanges():
        environmentChanges_() {

    }

    void addChange(EnvironmentChangeSharedPtr environmentChange) {
        environmentChanges_.push(environmentChange);
    }

    const EnvironmentChangeSharedPtr getNextChange() {
        if (environmentChanges_.size() == 0)
            return nullptr;
        EnvironmentChangeSharedPtr nextChange = environmentChanges_.front();
        environmentChanges_.pop();
        return nextChange;
    }
    
    const unsigned int getNumChanges() const {
	return environmentChanges_.size();
    }

private:
    std::queue<EnvironmentChangeSharedPtr> environmentChanges_;

};

/** @brief std::shared_ptr to oppt::EnvironmentChanges */
typedef std::shared_ptr<EnvironmentChanges> EnvironmentChangesPtr;
}

#endif
