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
#include "RRTConnect.hpp"
#include "Tree.hpp"
#include "oppt/opptCore/trajectory.hpp"
#include "oppt/robotEnvironment/include/RobotEnvironment.hpp"
#include "oppt/opptCore/utils.hpp"
#include <algorithm>
#include <boost/timer.hpp>

namespace oppt
{
RRTConnect::RRTConnect(const RobotEnvironment* robotEnvironment, const FloatType& planningRange):
    startTree_(new Tree()),
    goalTree_(new Tree()),
    goalStates_(),
    robotEnvironment_(robotEnvironment),
    planningRange_(planningRange),
    goalBias_(0.075)
{
    unsigned int dof = robotEnvironment->getRobot()->getDOF();    
    distanceFunction_ = DistanceFunction([dof](const FloatType * s1,
    const FloatType * s2) {
        return math::euclideanDistance(s1, s2, dof);
    });

    startTree_->setDistanceFunction(distanceFunction_);
    goalTree_->setDistanceFunction(distanceFunction_);
}

RRTConnect::~RRTConnect()
{

}

void RRTConnect::setGoalStates(const std::vector< VectorFloat >& goalStates)
{
    goalStates_ = goalStates;
}

bool RRTConnect::satisfiesGoal_(const Node* node) const
{
    PropagationResultSharedPtr propRes(new PropagationResult());
    propRes->nextState = node->getRobotState();
    return robotEnvironment_->isTerminal(propRes);
}

const RobotStateSharedPtr RRTConnect::sample_() const
{
    return sampleUniform_();
}


const RobotStateSharedPtr RRTConnect::sampleUniform_() const
{
    auto randomEngine = robotEnvironment_->getRobot()->getRandomEngine();
    return robotEnvironment_->getRobot()->getStateSpace()->sampleUniform(randomEngine);
}

std::pair<ExtendStatus, const Node*> RRTConnect::extend_(Tree* tree,
        const RobotStateSharedPtr& qRand,
        const Node* nearestNeighbour)
{
    if (!nearestNeighbour) {        
        nearestNeighbour = tree->nearestNeighbour(qRand);        
    }
    
    FloatType d = distanceFunction_(nearestNeighbour->getValues().data(),
                                    qRand->as<VectorState>()->asVector().data());
    if (d <= planningRange_) {
        if (!isValid_(qRand))
            return std::pair<ExtendStatus, const Node *>(TRAPPED, nearestNeighbour);
        return std::pair<ExtendStatus, const Node*>(REACHED,
                tree->allocNode(nearestNeighbour, qRand));
    }

    const RobotStateSharedPtr nnVec = interpolate_(nearestNeighbour->getRobotState(),
                                      qRand,
                                      planningRange_ / d);
    if (isValid_(nnVec)) {
        return std::pair<ExtendStatus, const Node*>(ADVANCED,
                tree->allocNode(nearestNeighbour,
                                nnVec));
    }

    return std::pair<ExtendStatus, const Node*>(TRAPPED,
            nearestNeighbour);
}

std::pair<ExtendStatus, const Node*> RRTConnect::connect_(Tree* tree,
        const RobotStateSharedPtr& qRand)
{
    boost::timer t0;
    auto nearestNeighbour = tree->nearestNeighbour(qRand);
    nnSearchTime_ += t0.elapsed();
    while (true) {
        auto extendResult = extend_(tree, qRand, nearestNeighbour);
        if (extendResult.first == REACHED || extendResult.first == TRAPPED) {
            return extendResult;
        }

        nearestNeighbour = extendResult.second;

    }
}

void RRTConnect::setIsValidFunction(std::function<bool(const RobotStateSharedPtr& state)> &isValidFn) {
    isValidFn_ = isValidFn;
}

bool RRTConnect::isValid_(const RobotStateSharedPtr& state)
{    
    return isValidFn_(state);
}

void RRTConnect::printTimes()
{
    cout << "nnSearchTime_: " << nnSearchTime_ << endl;
    cout << "extendTime_: " << extendTime_ << endl;
    cout << "interpolationTime_: " << interpolationTime_ << endl;
    cout << "validCheckTime_: " << validCheckTime_ << endl;
}

void RRTConnect::reset()
{
    nnSearchTime_ = 0;
    extendTime_ = 0;
    validCheckTime_ = 0;
    interpolationTime_ = 0;
}

TrajectorySharedPtr RRTConnect::solve(const RobotStateSharedPtr& state, const FloatType& timeout)
{
    startTree_->reset();
    goalTree_->reset();
    startTree_->makeRoot(state);
    RobotStateSharedPtr goalState(new VectorState(goalStates_[0]));
    goalTree_->makeRoot(goalState);

    bool solved = false;
    bool startTree = true;
    boost::timer t0;
    while (!solved) {
        Tree* currentTree = startTree ? startTree_.get() : goalTree_.get();
        Tree* otherTree = startTree ? goalTree_.get() : startTree_.get();

        bool sampledValid = false;
        RobotStateSharedPtr qRand = nullptr;
        while (!sampledValid) {
            qRand = sample_();            
            sampledValid = true;
        }

        auto extendResult = extend_(currentTree, qRand);
        if (extendResult.first != TRAPPED) {
            // Try to connect to the other tree
            auto connectResult = connect_(otherTree,
                                          extendResult.second->getRobotState());
            if (connectResult.first == REACHED) {
                if (startTree) {
                    return generateTrajectory_(extendResult.second,
                                               connectResult.second);
                }

                return generateTrajectory_(connectResult.second,
                                           extendResult.second);
            }
        }
        
        startTree = !startTree;        
        if (t0.elapsed() > timeout) {	    
            return nullptr;
        }
    }

    // Should never get here
    return nullptr;
}

TrajectorySharedPtr RRTConnect::generateTrajectory_(const Node* nodeTree1,
        const Node* nodeTree2)
{
    TrajectorySharedPtr trajectory(new Trajectory());
    auto currNode = nodeTree1;
    while (currNode) {
        RobotStateSharedPtr state(new VectorState(currNode->getValues()));
        trajectory->stateTrajectory.push_back(state);
        currNode = currNode->getParent();
    }
    std::reverse(trajectory->stateTrajectory.begin(), trajectory->stateTrajectory.end());

    currNode = nodeTree2->getParent();
    while (currNode) {
        RobotStateSharedPtr state(new VectorState(currNode->getValues()));
        trajectory->stateTrajectory.push_back(state);
        currNode = currNode->getParent();
    }

    //shortenPath_(trajectory);
    return trajectory;
}

void RRTConnect::shortenPath_(TrajectorySharedPtr& trajectory)
{
    std::vector<unsigned int> markForDelete;
    for (int i = trajectory->stateTrajectory.size() - 1; i >= 0; --i) {
        for (int j = i - 2; j >= 0; --j) {
            if (j < i - 1) {
                if (distanceFunction_(trajectory->stateTrajectory[i]->as<VectorState>()->asVector().data(),
                                      trajectory->stateTrajectory[j]->as<VectorState>()->asVector().data()) <= planningRange_) {
                    unsigned int k = j + 1;
                    while (k != i) {
                        markForDelete.push_back(k);
                        k++;
                    }

                    i = j;
                }
            }
        }


    }

    for (int i = markForDelete.size() - 1; i >= 0; --i) {
        trajectory->stateTrajectory.erase(trajectory->stateTrajectory.begin() + markForDelete[i]);

    }
}

const RobotStateSharedPtr RRTConnect::interpolate_(const VectorFloat& vec1,
        const VectorFloat& vec2,
        const FloatType& t) const
{
    RobotStateSharedPtr robotState1 = std::make_shared<VectorState>(vec1);
    RobotStateSharedPtr robotState2 = std::make_shared<VectorState>(vec2);
    return robotEnvironment_->getRobot()->getStateSpace()->interpolate(robotState1, robotState2, t);
}

const RobotStateSharedPtr RRTConnect::interpolate_(const RobotStateSharedPtr& state1,
        const VectorFloat& state2,
        const FloatType& t) const
{
    RobotStateSharedPtr robotState2 = std::make_shared<VectorState>(state2);
    return robotEnvironment_->getRobot()->getStateSpace()->interpolate(state1, robotState2, t);
}

const RobotStateSharedPtr RRTConnect::interpolate_(const RobotStateSharedPtr& state1,
        const RobotStateSharedPtr& state2,
        const FloatType& t) const
{
    return robotEnvironment_->getRobot()->getStateSpace()->interpolate(state1, state2, t);
}

}
