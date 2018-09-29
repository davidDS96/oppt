#include "FrapuABTSimulator.hpp"
#include "solvers/ABT/ABTOptions.hpp"
#include "oppt/events/Event.hpp"

namespace abt
{
FrapuABTSimulator::FrapuABTSimulator(std::unique_ptr< Model > model, Solver* solver, bool hasDynamicChanges):
    Simulator(std::move(model), solver, hasDynamicChanges),
    treeInspector_(std::make_unique<TreeInspector>())
{

}

FloatType FrapuABTSimulator::runSimulation(std::ofstream& os)
{
    bool canDoSimulation = true;
    bool terminalStateReached = false;
    std::pair<FloatType, std::pair<long, FloatType> > simulationResult;
    while (stepSimulation(os)) {

    }
    if (options_->hasVerboseOutput) {
        cout << endl << endl << "Final State:" << endl;
        State const& currentState = *getCurrentState();
        cout << currentState << endl;
        BeliefNode* currentBelief = agent_->getCurrentBelief();
        cout << "Belief #" << currentBelief->getId() << endl;
        model_->drawSimulationState(currentBelief, currentState, cout);
    }
    return totalDiscountedReward_;

}

bool FrapuABTSimulator::stepSimulation(std::ofstream& os)
{
    bool stepResult = Simulator::stepSimulation(os);

    auto options = getModel()->getOptions();
    if (static_cast<const ABTExtendedOptions*>(options)->saveParticles) {
        bool logTmp = logGazeboState;
        bool gazeboStateLogging = false;
        enableGazeboStateLogging(gazeboStateLogging);
        auto currentBelief = getAgent()->getCurrentBelief();
        std::vector<State const*> currentParticles = currentBelief->getStates();

        os << "PARTICLES BEGIN" << endl;
        for (size_t k = 0; k < currentParticles.size(); k++) {            
            currentParticles[k]->serialize(os, "p"); 
	    os << " \n";
        }
        os << "PARTICLES END" << endl;
        enableGazeboStateLogging(logTmp);
    }

    // Do some additional stuff here
    if (treeInspector_) {
        treeInspector_->makeTreeStatistics(this, os);
    }
    
    oppt::event::Events::stepFinished(0, getStepCount());

    return stepResult;
}

TreeInspector *const FrapuABTSimulator::getTreeInspector() const {
    return treeInspector_.get();
}

}
