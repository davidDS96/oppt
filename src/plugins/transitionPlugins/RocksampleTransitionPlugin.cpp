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
#include "oppt/plugin/Plugin.hpp"
#include "oppt/opptCore/Distribution.hpp"

namespace oppt
{
class RocksampleTransitionPlugin: public TransitionPlugin
{
public:
    typedef std::function<bool()> DubinCollisionFunction;

    RocksampleTransitionPlugin():
        TransitionPlugin(),
        rockLocations_() {
    }

    virtual ~RocksampleTransitionPlugin() = default;

    virtual bool load(RobotEnvironment* const robotEnvironment, const std::string& optionsFile) override {
        robotEnvironment_ = robotEnvironment;

        // Get the rock locations from the robotEnvironment
        getRockLocations();
        return true;
    }

    virtual PropagationResultSharedPtr propagateState(const PropagationRequest* propagationRequest) const override {
        PropagationResultSharedPtr propagationResult(new PropagationResult());
        VectorFloat stateVector = propagationRequest->currentState->as<VectorState>()->asVector();
        VectorFloat actionVec = propagationRequest->action->as<VectorAction>()->asVector();

        VectorFloat resultingState(stateVector);
        bool moved = false;
        bool sampled = false;
        switch ((unsigned int)(actionVec[0] + 0.025)) {
        case 1: resultingState[1] = (FloatType)((unsigned int)(resultingState[1] + 0.25) + 1); // Go north
            break;
        case 2: resultingState[1] = (FloatType)((unsigned int)(resultingState[1] + 0.25) - 1); // Go south
            break;
        case 3: resultingState[0] = (FloatType)((unsigned int)(resultingState[0] + 0.25) + 1); // Go east
            break;
        case 4: resultingState[0] = (FloatType)((unsigned int)(resultingState[0] + 0.25) - 1); // Go west
            break;
        case 5: sampleRockAtCurrentLocation(resultingState); // Sample rock at the current location
            break;
        default:  // Got a sampling action. Nothing to do here
            break;
        }

        // Normalize the output state
        propagationResult->previousState = propagationRequest->currentState;
        propagationResult->nextState =
            std::make_shared<oppt::VectorState>(resultingState);
        propagationResult->madeCollisionReport = true;
        propagationResult->constraintsEnforced = true;
        propagationResult->collided = false;
        return propagationResult;
    }

private:
    const RobotEnvironment* robotEnvironment_ = nullptr;

    // Rock indices and their respective location
    std::unordered_map<unsigned int, VectorFloat> rockLocations_;

private:
    void getRockLocations() {
        auto scene = robotEnvironment_->getScene();
        VectorObstaclePtr obstacles;
        scene->getObstacles(obstacles);
        unsigned int idx = 2;
        for (auto & obstacle : obstacles) {
            if (obstacle->isEnabled()) {
                auto pose = obstacle->getGeometry()->getPose();
                VectorFloat location(2, 0);
                location[0] = pose(0, 3);
                location[1] = pose(1, 3);
                rockLocations_[idx] = location;
                idx++;
            }
        }
    }
    
    void sampleRockAtCurrentLocation(VectorFloat& stateVec) const {
        // Check if there's a rock at the current location
        unsigned int s1 = (unsigned int)(stateVec[0] + 0.25);
        unsigned int s2 = (unsigned int)(stateVec[1] + 0.25);
        for (auto & location : rockLocations_) {
            if (s1 == (unsigned int)(location.second[0] + 0.25) && s2 == (unsigned int)(location.second[1] + 0.25)) {
                stateVec[location.first] = 0;
                return;
            }
        }
    }
};

OPPT_REGISTER_TRANSITION_PLUGIN(RocksampleTransitionPlugin)

}
