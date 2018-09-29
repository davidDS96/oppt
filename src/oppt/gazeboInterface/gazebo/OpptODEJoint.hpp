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
#ifndef _OPPT_ODE_JOINT_HPP_
#define _OPPT_ODE_JOINT_HPP_
#include "OpptJoint.hpp"
#include "opende/src/joints/hinge.h"
#include "opende/src/joints/ball.h"
#include "opende/src/joints/slider.h"
#include "opende/src/joints/screw.h"
#include "opende/src/joints/gearbox.h"
#include "opende/src/joints/universal.h"
#include "opende/src/joints/fixed.h"

namespace gazebo
{
namespace physics
{

template<class JointType, class ODEJointType>
class OpptODEJoint: public JointType, public OpptJoint
{
public:
    OpptODEJoint(dWorldID _worldId, BasePtr _parent):
        JointType(_worldId, _parent),
        OpptJoint() {

    }
    
    virtual void SetForceImpl(unsigned int _index, double _effort) override {
        if (!blockSetForce_)
            JointType::SetForceImpl(_index, _effort);
    }
    
    virtual std::vector<double> getCumulativeAngles() const override { 
	return std::vector<double>();
    }
    
    virtual void setCumulativeAngles(const std::vector<double> &cumulativeAngles) const override { 
	
    }
    
    virtual std::string getName() const override {
        return this->GetName();
    }

};

template<class JointType, class ODEJointType>
class OpptODEJointExtended: public JointType, public OpptJoint
{
public:
    OpptODEJointExtended(dWorldID _worldId, BasePtr _parent):
        JointType(_worldId, _parent),
        OpptJoint() {

    }

    virtual void SetForceImpl(unsigned int _index, double _effort) override {
        if (!blockSetForce_)
            JointType::SetForceImpl(_index, _effort);
    }

    virtual std::vector<double> getCumulativeAngles() const override {
        return std::vector<double>({static_cast<ODEJointType* const>(this->jointId)->cumulative_angle});
    }

    virtual void setCumulativeAngles(const std::vector<double> &cumulativeAngles) const override {	
        static_cast<ODEJointType*>(this->jointId)->cumulative_angle = cumulativeAngles[0];
    }

    virtual std::string getName() const override {
        return this->GetName();
    }
};

template<class JointType, class ODEJointType>
class OpptODEJointExtended2: public JointType, public OpptJoint
{
public:
    OpptODEJointExtended2(dWorldID _worldId, BasePtr _parent):
        JointType(_worldId, _parent),
        OpptJoint() {

    }

    virtual void SetForceImpl(unsigned int _index, double _effort) override {
        if (!blockSetForce_)
            JointType::SetForceImpl(_index, _effort);
    }

    virtual std::vector<double> getCumulativeAngles() const override {
	return std::vector<double>({static_cast<ODEJointType* const>(this->jointId)->cumulative_angle1, 
	    static_cast<ODEJointType* const>(this->jointId)->cumulative_angle2});        
    }

    virtual void setCumulativeAngles(const std::vector<double> &cumulativeAngles) const override {	
        static_cast<ODEJointType*>(this->jointId)->cumulative_angle1 = cumulativeAngles[0];
	static_cast<ODEJointType*>(this->jointId)->cumulative_angle2 = cumulativeAngles[1];
    }

    virtual std::string getName() const override {
        return this->GetName();
    }
};

}
}

#endif
