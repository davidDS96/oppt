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
#ifndef _OPPT_JOINT_HPP_
#define _OPPT_JOINT_HPP_

namespace gazebo
{
namespace physics
{
class OpptJoint
{
public:
    OpptJoint():
        getCumulativeAngleFn_(nullptr) {
    }

    virtual void blockSetForce(const bool& block) {
        blockSetForce_ = block;
    }

    void setGetCumulativeAngleFn(std::function<double()> getCumulativeAngleFn) {
        getCumulativeAngleFn_ = getCumulativeAngleFn;
    }

    void setSetCumulativeAngleFn(std::function<void(const double&)> setCumulativeAngleFn) {
        setCumulativeAngleFn_ = setCumulativeAngleFn;
    }

    virtual std::vector<double> getCumulativeAngles() const = 0;

    virtual void setCumulativeAngles(const std::vector<double> &cumulativeAngles) const = 0;
    
    virtual std::string getName() const = 0;

protected:
    bool blockSetForce_ = false;

    std::function<double()> getCumulativeAngleFn_;

    std::function<void(const double&)> setCumulativeAngleFn_;
};

}
}

#endif
