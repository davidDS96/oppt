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
#include "oppt/robotHeaders/robot.hpp"
#include "oppt/opptCore/ObservationReport.hpp"
#include "oppt/opptCore/EnvironmentInfo.hpp"
#include "oppt/opptCore/CollisionObject.hpp"

#ifdef USE_RVIZ
#include "oppt/viewerPublisher/ViewerPublisher.hpp"
#endif

#include "oppt/plugin/Plugin.hpp"

using std::cout;
using std::endl;

namespace oppt
{

Robot::Robot(std::string worldFile,
             std::string robotFile,
             std::string configFile,
             std::string prefix,
             unsigned int& threadId):
    robotName_(""),
    robotFile_(robotFile),
    viewer_(nullptr),
    environmentInfo_(nullptr),
    stateSpace_(nullptr),
    actionSpace_(nullptr),
    observationSpace_(nullptr),
    serializer_(nullptr),
    randomEngine_(nullptr),
    configFile_(configFile),
    collisionsAllowed_(false),
    robotCollisionObjectsVec_(),
    linkCollisionGeometryMap_(),
    prefix_(prefix)
{
    sdf::SDFPtr sdfModel(new sdf::SDF());
    sdf::init(sdfModel);
    sdf::readFile(robotFile, sdfModel);
    sdf::ElementPtr rootElement = sdfModel->Root();
    sdf::ElementPtr modelElement = rootElement->GetElement("model");
    if (!modelElement) {
        ERROR("Robot: SDF model file has no model element");
    }

    robotName_ = modelElement->Get<std::string>("name");    
    ikSolver_ = std::make_unique<DefaultIKSolver>(randomEngine_.get(), "", "", "");
}

Robot::~Robot()
{
}

bool Robot::init()
{
    return true;
}

bool Robot::initializeViewer_(std::string modelFile, std::string environmentFile)
{
#ifdef USE_RVIZ
    viewer_ = std::make_shared<ViewerPublisher>();
    static_cast<ViewerPublisher*>(viewer_.get())->setupViewer(modelFile, environmentFile);
#endif
    return true;
}

std::string Robot::getName() const
{
    return robotName_;
}

bool Robot::getCollisionsAllowed() const
{
    return collisionsAllowed_;
}

void Robot::setCollisionsAllowed(const bool& collisionsAllowed) const
{
    collisionsAllowed_ = collisionsAllowed;
}

void Robot::setRandomEngine(RandomEnginePtr randomEngine)
{
    randomEngine_ = randomEngine;
}

RandomEnginePtr Robot::getRandomEngine() const
{
    return randomEngine_;
}

void Robot::initTransitionPlugin(const std::string& pluginFile,
                                 RobotEnvironment* robotEnvironment)
{
    //if (propagator_)
    //    propagator_->close();
    propagator_ =
        TransitionPlugin::Create(pluginFile, "transitionPlugin");
    transitionPluginFile_ = pluginFile;
}

void Robot::initObservationPlugin(const std::string& pluginFile,
                                  RobotEnvironment* robotEnvironment)
{
    //if (observationPlugin_)
    //    observationPlugin_->close();
    observationPlugin_ =
        ObservationPlugin::Create(pluginFile, "observationPlugin");
    observationPluginFile_ = pluginFile;
}

void Robot::loadTransitionPlugin(const std::string& optionsFile,
                                 RobotEnvironment* robotEnvironment)
{
    if (!propagator_->load(robotEnvironment, optionsFile)) {
        ERROR("Propagator plugin '" +
              transitionPluginFile_ +
              "' couldn't be loaded");
    }

    transitionPluginOptionsFile_ = optionsFile;
}

void Robot::loadObservationPlugin(const std::string& optionsFile,
                                  RobotEnvironment* robotEnvironment)
{
    if (!observationPlugin_->load(robotEnvironment, optionsFile)) {
        ERROR("Observation plugin '" +
              observationPluginFile_ +
              "' couldn't be loaded");
    }

    observationPluginOptionsFile_ = optionsFile;
}

PropagationResultSharedPtr Robot::propagateState(PropagationRequestSharedPtr& propagationRequest)
{
    boost::this_thread::interruption_point();
    PropagationResultSharedPtr propagationResult = propagator_->propagateState(propagationRequest.get());
    propagationResult->previousState = propagationRequest->currentState.get();
    propagationResult->action = propagationRequest->action.get();
    propagationResult->errorVector = propagationRequest->errorVector;
    return propagationResult;
}

ObservationResultSharedPtr Robot::makeObservationReport(ObservationRequestSharedPtr& observationRequest) const
{
    boost::this_thread::interruption_point();
    ObservationResultSharedPtr observationResult = observationPlugin_->getObservation(observationRequest.get());
    observationResult->state = observationRequest->currentState.get();
    observationResult->action = observationRequest->action.get();
    return observationResult;

}

void Robot::initCollisionObjects()
{
    for (auto& linkGeometriesEntry : linkCollisionGeometryMap_) {
        for (auto& linkGeometry : linkGeometriesEntry.second) {
            CollisionGeometrySharedPtr linkCollisionGeometry = linkGeometry->getCollisionGeometry();
            fcl::Matrix3f rotationMatrix(1, 0, 0,
                                         0, 1, 0,
                                         0, 0, 1);
            fcl::Vec3f translationVector(0, 0, 0);
            fcl::Transform3f trans(rotationMatrix, translationVector);
            CollisionObjectSharedPtr collObj(new fcl::CollisionObject(linkCollisionGeometry, trans));
            OpptCollisionObjectSharedPtr opptCollisionObject = std::make_shared<OpptCollisionObject>(collObj, linkGeometry->getName());
            robotCollisionObjectsVec_.push_back(opptCollisionObject);
        }
    }
}

oppt::CollisionReportSharedPtr Robot::makeDiscreteCollisionReport(const oppt::RobotStateSharedPtr& state) const
{
    if (!discreteCollisionFunction_)
        ERROR("No oppt::DiscreteCollisionFunction defined");
    return discreteCollisionFunction_(state);
}

oppt::CollisionReportSharedPtr Robot::makeContinuousCollisionReport(const oppt::RobotStateSharedPtr& state1,
        const oppt::RobotStateSharedPtr& state2,
        const unsigned int& numIterations) const
{
    if (!continuousCollisionFunction_)
        ERROR("No oppt::ContinuousCollisionFunction defined");
    return continuousCollisionFunction_(state1, state2, numIterations);
}

Serializer* Robot::getSerializer() const
{
    if (!serializer_)
        ERROR("Serializer is null");
    return serializer_.get();
}

oppt::StateSpaceSharedPtr Robot::getStateSpace() const
{
    return stateSpace_;
}

oppt::ObservationSpaceSharedPtr Robot::getObservationSpace() const
{
    return observationSpace_;
}

oppt::ActionSpaceSharedPtr Robot::getActionSpace() const
{
    if (!actionSpace_) {
        assert(false && "ACTION SPACE IS NULL");
    }
    return actionSpace_;
}

void Robot::setEnvironmentInfo(oppt::EnvironmentInfoSharedPtr& environmentInfo)
{
    if (!environmentInfo) {
        oppt::ERROR("Robot: setSenvironmentInfo: environmentInfo is null!!!");
    }

    environmentInfo_ = environmentInfo;
    environmentInfo_->getScene()->setRobotCollisionObjects(robotCollisionObjectsVec_);
    environmentInfo_->getScene()->setCollisionInvariantRobotCollisionObjects(invariantRobotCollisionObjectsVec_);
}

void Robot::addObstacleCallback(oppt::ObstacleSharedPtr& obstacle)
{

}

void Robot::removeObstacleCallback(std::string name)
{

}

void Robot::changeObstaclePoseCallback(const std::string& name, const VectorFloat& poseVec)
{

}

void Robot::initEnvironmentCallbacks()
{

}

void Robot::registerOnAddObstacleCallback(std::function<void(const std::string&)>& callback)
{

}

void Robot::registerOnRemoveObstacleCallback(std::function<void(const std::string&)>& callback)
{

}

void Robot::registerOnPoseChangedCallback(std::function<void(const std::string&, const VectorFloat&)>& callback)
{

}

void Robot::setupViewer(std::string modelFile, std::string environmentFile)
{
#ifdef USE_RVIZ
    if (viewer_) {
        static_cast<ViewerPublisher*>(viewer_.get())->setupViewer(modelFile, environmentFile);
        static_cast<ViewerPublisher*>(viewer_.get())->updateFromEnvironmentInfo(environmentInfo_);
    }
#endif
}

const std::string Robot::getRobotFile() const
{
    return robotFile_;
}

FloatType Robot::calcLikelihood(const RobotStateSharedPtr& state,
                                const ActionSharedPtr& action,
                                const ObservationSharedPtr& observation)
{
    return observationPlugin_->calcLikelihood(state, action, observation);
}

void Robot::setGazeboInterface(GazeboInterface* const gazeboInterface)
{
    gazeboInterface_ = gazeboInterface;
}

bool Robot::normalizedSpaces() const
{
    return normalizedSpaces_;
}

TransitionPlugin* const Robot::getTransitionPlugin() const
{
    return propagator_.get();
}

ObservationPlugin* const Robot::getObservationPlugin() const
{
    return observationPlugin_.get();
}

void Robot::setDiscreteCollisionFunction(DiscreteCollisionFunction discreteCollisionFunction)
{
    discreteCollisionFunction_ = discreteCollisionFunction;
}

void Robot::setContinuousCollisionFunction(ContinuousCollisionFunction continuousCollisionFunction)
{
    continuousCollisionFunction_ = continuousCollisionFunction;
}

ViewerBase* const Robot::getViewer() const
{
    if (viewer_)
        return viewer_.get();
    return nullptr;
}

void Robot::setIKSolver(IKSolverUniquePtr ikSolver) {
    ikSolver_ = std::move(ikSolver);
}

IKSolver *const Robot::getIKSolver() const {
    if (ikSolver_)
        return ikSolver_.get();
    return nullptr;
}

std::pair<bool, VectorFloat> Robot::getIKSolution(const VectorFloat &pose,
        const unsigned int &numAttempts,
        const VectorFloat &qInit) const {
    if (ikSolver_)
        return ikSolver_->solve(pose, numAttempts, qInit);
    return std::make_pair(false, VectorFloat());
}


}
