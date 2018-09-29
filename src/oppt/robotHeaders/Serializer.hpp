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
#ifndef __OPPT__SERIALIZER__HPP__
#define __OPPT__SERIALIZER__HPP__
#include "oppt/opptCore/core.hpp"
#include "Action.hpp"
#include "Observation.hpp"
#include "RobotState.hpp"

namespace oppt
{

/**
 * A class used for serialization of states, actions and observations
 */
class Serializer
{
public:
    template<class T>
    T* as() {
        return static_cast<T*>(this);
    }

    /**
     * @brief Loads a RobotState from an input stream
     * @param is The input stream
     * @returns A shared pointer to the loaded RobotState
     */
    virtual oppt::RobotStateSharedPtr loadState(std::istream& is) const = 0;

    /**
     * @brief Loads a RobotState from an input string
     * @param input The input string
     * @returns A shared pointer to the loaded RobotState
     */
    virtual RobotStateSharedPtr loadState(const std::string& input) const = 0;

    /**
     * @brief Loads an Action from an input stream
     * @param is The input stream
     * @returns A shared pointer to the loaded Action
     */
    virtual oppt::ActionSharedPtr loadAction(std::istream& is) const = 0;

    /**
     * @brief Loads an Observation from an input stream
     * @param is The input stream
     * @returns A shared pointer to the loaded Observation
     */
    virtual oppt::ObservationSharedPtr loadObservation(std::istream& is) const = 0;

    virtual std::vector<oppt::RobotStateSharedPtr> loadGoalStatesFromFile(std::string& filename) const = 0;

    /**
     * @brief Serializes a RobotState to a given output stream
     * @param state The state to serializer
     * @param os The output stream     
     */
    virtual void saveState(oppt::RobotStateSharedPtr& state, std::ostream& os) const {
        state->serialize(os);
    }

    /**
     * @brief Serializes an Action to a given output stream
     * @param state The state to serializer
     * @param os The output stream     
     */
    virtual void saveAction(oppt::ActionSharedPtr& action, std::ostream& os) const {
        action->serialize(os);
    }
    
    /**
     * @brief Serializes an Observation to a given output stream
     * @param state The state to serializer
     * @param os The output stream     
     */
    virtual void saveObservation(oppt::ObservationSharedPtr& observation, std::ostream& os) const {
        observation->serialize(os);
    }
};

/**
 * Specialization of the Serializer for vector valued states, actions and observations
 */
class VectorSerializer: public Serializer
{
public:
    VectorSerializer();

    virtual oppt::RobotStateSharedPtr loadState(std::istream& is) const override;

    virtual RobotStateSharedPtr loadState(const std::string& input) const override;

    virtual oppt::ActionSharedPtr loadAction(std::istream& is) const override;

    virtual oppt::ObservationSharedPtr loadObservation(std::istream& is) const override;

    virtual std::vector<oppt::RobotStateSharedPtr> loadGoalStatesFromFile(std::string& filename) const override;

};

}

#endif
