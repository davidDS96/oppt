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
#ifndef _RRT_CONNECT_HPP_
#define _RRT_CONNECT_HPP_
#include "Tree.hpp"

namespace oppt
{
enum ExtendStatus {
    TRAPPED,
    ADVANCED,
    REACHED

};

class RRTConnect
{
public:
    RRTConnect(const RobotEnvironment* robotEnvironment, const FloatType& planningRange);

    ~RRTConnect();

    TrajectorySharedPtr solve(const RobotStateSharedPtr& state, 
                              const FloatType& timeout);

    void setGoalStates(const std::vector<VectorFloat>& goalStates);

    void reset();

    void printTimes();
    
    void setIsValidFunction(std::function<bool(const RobotStateSharedPtr& state)> &isValidFn);

private:
    std::unique_ptr<Tree> startTree_;

    std::unique_ptr<Tree> goalTree_;

    std::vector<VectorFloat> goalStates_;

    const RobotEnvironment* robotEnvironment_ = nullptr;

    const FloatType planningRange_;

    FloatType goalBias_;

    DistanceFunction distanceFunction_ = nullptr;

private:
    bool satisfiesGoal_(const Node* node) const;

    const RobotStateSharedPtr sample_() const;

    const RobotStateSharedPtr sampleUniform_() const;

    std::pair<ExtendStatus, const Node*> extend_(Tree* tree,
            const RobotStateSharedPtr& qRand,
            const Node* nearestNeighbour = nullptr);

    std::pair<ExtendStatus, const Node*> connect_(Tree* tree,
            const RobotStateSharedPtr& qRand);

    TrajectorySharedPtr generateTrajectory_(const Node* nodeTree1, const Node* nodeTree2);

    void shortenPath_(TrajectorySharedPtr& trajectory);

    const RobotStateSharedPtr interpolate_(const VectorFloat& vec1,
                                           const VectorFloat& vec2,
                                           const FloatType& t) const;

    const RobotStateSharedPtr interpolate_(const RobotStateSharedPtr& state1,
                                           const VectorFloat& state2,
                                           const FloatType& t) const;

    const RobotStateSharedPtr interpolate_(const RobotStateSharedPtr& state1,
                                           const RobotStateSharedPtr& state2,
                                           const FloatType& t) const;

    bool isValid_(const RobotStateSharedPtr& state);
    
    std::function<bool(const RobotStateSharedPtr& state)> isValidFn_ = nullptr;

    FloatType nnSearchTime_ = 0.0;

    FloatType extendTime_ = 0.0;

    FloatType interpolationTime_ = 0;

    FloatType validCheckTime_ = 0;

};
}

#endif
