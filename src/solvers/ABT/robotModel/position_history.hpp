/** @file position_history.hpp
 *
 * Defines a class to keep track of the position of the robot in RockSample.
 *
 * This is useful, since the position is fully observable but is not included in observations.
 */
#ifndef ROBOT_POSITION_HISTORY_HPP_
#define ROBOT_POSITION_HISTORY_HPP_

#include <memory>
#include <vector>

#include "solvers/ABT/solver/abstract-problem/HistoricalData.hpp"

#include "solvers/ABT/solver/serialization/TextSerializer.hpp"

//#include "problems/shared/GridPosition.hpp"
//#include "CartesianCoordinates.hpp"
#include "RobotModel.hpp"
#include "RobotAction.hpp"
#include <iostream>

namespace robot
{
class RobotAction;
class RobotModel;

/** An implementation of the serialization methods for the PositionData class. */
class PositionDataTextSerializer : virtual public abt::TextSerializer
{
public:
    void saveHistoricalData(abt::HistoricalData const* data, std::ostream& os) override;
    std::unique_ptr<abt::HistoricalData> loadHistoricalData(std::istream& is) override;
};

/** A class to store the robot position associated with a given belief node.
 *
 * Since the robot position in RockSample is fully observable, all particles in any belief will
 * in fact have the same position, which is stored here.
 */
class PositionData : public abt::HistoricalData
{
    friend class PositionDataTextSerializer;
public:
    /** Creates a new PositionData instance for the given model, and located in the given grid
     * square.
     */
    PositionData(RobotModel* model,
                 const oppt::RobotStateSharedPtr &robotState,
                 const oppt::ActionSharedPtr &action,
                 int old_step);
    PositionData(RobotModel* model,
                 const oppt::RobotStateSharedPtr &robotState,
                 const oppt::ActionSharedPtr &action,
                 int old_step,
                 bool /*copy_const*/);
    virtual ~PositionData() = default;
    _NO_COPY_OR_MOVE(PositionData);

    std::unique_ptr<abt::HistoricalData> copy() const;

    std::unique_ptr<abt::HistoricalData> createChild(
        abt::Action const& action,
        abt::Observation const& observation) const override;

    int getCurrentStep() const;
    
    const oppt::ActionSharedPtr getAction() const;

    const oppt::RobotStateSharedPtr getRobotState() const;

    void print(std::ostream& os) const override;

private:
    /** The RockSampleModel instance this PositionData instance is associated with. */
    RobotModel* model_;
    /** The grid position of this PositionData. */

    const oppt::RobotStateSharedPtr robotState_;
    
    /** The action that lead to the robotState */
    const oppt::ActionSharedPtr action_;

    int current_step_;

};


} /* namespace manipulator */

#endif /* MANIPULATOR_POSITION_HISTORY_HPP_ */
