/** @file position_history.cpp
 *
 * Contains the implementations for PositionData and PositionDataTextSerializer.
 */
#include "position_history.hpp"

#include <iostream>
#include <sstream>


#include "solvers/ABT/solver/ActionNode.hpp"
#include "solvers/ABT/solver/BeliefNode.hpp"

#include "solvers/ABT/solver/abstract-problem/Action.hpp"

namespace robot
{
/* ---------------------- PositionData --------------------- */
PositionData::PositionData(RobotModel* model,
                           const oppt::RobotStateSharedPtr& robotState,
                           const oppt::ActionSharedPtr& action,
                           int old_step) :
    model_(model),
    robotState_(robotState),
    action_(action),
    current_step_(old_step + 1)
{
}

PositionData::PositionData(RobotModel* model,
                           const oppt::RobotStateSharedPtr& robotState,
                           const oppt::ActionSharedPtr& action,
                           int old_step,
                           bool /*copy_const*/) :
    model_(model),
    robotState_(robotState),
    action_(action),
    current_step_(old_step)
{
}


std::unique_ptr<abt::HistoricalData> PositionData::copy() const
{
    return std::make_unique<PositionData>(model_, robotState_, action_, current_step_, true);
}

std::unique_ptr<abt::HistoricalData> PositionData::createChild(
    abt::Action const& action,
    abt::Observation const& /*observation*/) const
{
    oppt::ActionSharedPtr robotAction = model_->getBaseAction(action);
    oppt::PropagationResultSharedPtr propagationResult = model_->makeNextState(robotState_, robotAction);
    return std::make_unique<PositionData>(model_,
                                          propagationResult->nextState,
                                          robotAction,
                                          current_step_);
}

void PositionData::print(std::ostream& os) const
{
    //robotState_->print(os);
}

int PositionData::getCurrentStep() const
{
    return current_step_;
}

const oppt::RobotStateSharedPtr PositionData::getRobotState() const
{
    return robotState_;
}

const oppt::ActionSharedPtr PositionData::getAction() const
{
    return action_;
}

/* --------------------- PositionDataTextSerializer -------------------- */
void PositionDataTextSerializer::saveHistoricalData(abt::HistoricalData const* data,
        std::ostream& os)
{
    PositionData const& position_data = static_cast<PositionData const&>(*data);
    position_data.getRobotState()->serialize(os);
    os << endl;
    os << position_data.getCurrentStep() << " END" << endl;
}

std::unique_ptr<abt::HistoricalData> PositionDataTextSerializer::loadHistoricalData(
    std::istream& is)
{
    std::string line;
    std::getline(is, line);
    RobotModel* model = static_cast<RobotModel*>(getModel());
    RobotStateSharedPtr robotState = model->getRobotEnvironment()->getRobot()->getSerializer()->loadState(line);
    std::getline(is, line);

    int currentStep;
    std::istringstream ss(line);
    ss >> currentStep;
    return std::make_unique<PositionData>(model, robotState, nullptr, currentStep, true);
}

} /* namespace manipulator */
