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
#include "include/RobotEnvironment.hpp"
#include <boost/lexical_cast.hpp>
#include "oppt/plugin/Plugin.hpp"
#include "oppt/gazeboInterface/GazeboInterface.hpp"
#include "include/SceneImpl.hpp"
#include "oppt/robotHeaders/RobotImpl/RobotImpl.hpp"
#ifdef USE_RVIZ
#include "oppt/viewerPublisher/ViewerPublisher.hpp"
#endif

namespace oppt
{

RobotEnvironment::RobotEnvironment():
    robot_(nullptr),
    robot_path_(""),
    config_path_(""),
    worldFile_(""),
    environmentInfo_(nullptr),
    scene_(nullptr),
    onAddObstacleFn_(nullptr),
    onRemoveObstacleFn_(nullptr),
    onEnvironmentChangedFns_(),
    threadId_(0),
    environmentChanges_(std::make_shared<EnvironmentChanges>()),
    registeredRobotEnvironments_()
{

}

RobotEnvironment::~RobotEnvironment()
{
    robot_.reset();
}

const std::string RobotEnvironment::getPrefix() const
{
    return prefix_;
}

void RobotEnvironment::setProblemEnvironmentChangeHandlerFn(ProblemEnvironmentChangeHandlerFn problemEnvironmentChangeHandlerFn)
{
    problemEnvironmentChangeHandlerFn_ = problemEnvironmentChangeHandlerFn;
}

void RobotEnvironment::initializeEnvironmentCallbacks()
{
    if (prefix_ == "exec") {
        onAddObstacleFn_ =
            std::function<void(const std::string&)>(std::bind(&RobotEnvironment::onAddObstacle,
                    this,
                    std::placeholders::_1));
        robot_->registerOnAddObstacleCallback(onAddObstacleFn_);
        onRemoveObstacleFn_ =
            std::function<void(const std::string&)>(std::bind(&RobotEnvironment::onRemoveObstacle,
                    this,
                    std::placeholders::_1));
        robot_->registerOnRemoveObstacleCallback(onRemoveObstacleFn_);
        onObstaclePoseChangedFn_ =
            std::function<void(const std::string&, const VectorFloat&)>(std::bind(&RobotEnvironment::onObstaclePoseChanged,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2));
        robot_->registerOnPoseChangedCallback(onObstaclePoseChangedFn_);
    }
}

void RobotEnvironment::addEnvironmentChange(const EnvironmentChangeSharedPtr& environmentChange)
{
    environmentChanges_->addChange(environmentChange);
}

void RobotEnvironment::onAddObstacle(const std::string& str)
{
    // This method gets registered and called in GazeboInterface to notify the environment
    // About changes made in the gazebo client of the exec environment
    if (prefix_ == "exec") {
        EnvironmentChangeSharedPtr change = std::make_shared<ObstacleAddedChange>(str);
        //environmentChanges_->addChange(change);

        // Inform the solver about the changes

        // This needs to be changed such that ProblemEnvironment gets informed
        // about the change
        // which includes removing the addChange() line above
        // since this will be done by the ProblemEnvironment
        if (problemEnvironmentChangeHandlerFn_) {
            //EnvironmentChangeType changeType = EnvironmentChangeType::OBSTACLE_ADDED;
            problemEnvironmentChangeHandlerFn_(change);
        }
    }

}

void RobotEnvironment::onRemoveObstacle(const std::string& str)
{
    // This method gets registered and called in GazeboInterface to notify the environment
    // About changes made in the gazebo client of the exec environment
    if (prefix_ == "exec") {
        std::string obstacleName = str;

        // Deal with scoped names
        if (obstacleName.find("::") != std::string::npos) {
            VectorString elems;
            split(str, "::", elems);
            obstacleName = elems[elems.size() - 1];
        }
        EnvironmentChangeSharedPtr change = std::make_shared<ObstacleRemovedChange>(obstacleName);
        //environmentChanges_->addChange(change);
        // Inform the solver about the changes
        if (problemEnvironmentChangeHandlerFn_) {
            //EnvironmentChangeType changeType = EnvironmentChangeType::OBSTACLE_REMOVED;
            problemEnvironmentChangeHandlerFn_(change);
        }
    }
}

void RobotEnvironment::onObstaclePoseChanged(const std::string& str, const VectorFloat& pose)
{
    // This method gets registered and called in GazeboInterface to notify the environment
    // About changes made in the gazebo client of the exec environment
    if (prefix_ == "exec") {
        EnvironmentChangeSharedPtr change = std::make_shared<ObstaclePoseChange>(str, pose);
        //environmentChanges_->addChange(change);
        if (problemEnvironmentChangeHandlerFn_) {
            //EnvironmentChangeType changeType = EnvironmentChangeType::OBSTACLE_POSE_CHANGED;
            problemEnvironmentChangeHandlerFn_(change);
        }
    }
}

void RobotEnvironment::applyChanges()
{
    // Applies the environment changes to the exec environment. This adds or removes obstacles to/from the environment scene
    // and notifies all registered planning environments about the environment changes    
    unsigned int numChanges = environmentChanges_->getNumChanges();
    EnvironmentChangeSharedPtr change = environmentChanges_->getNextChange();
    while (change) {
        LOGGING("Apply change in " + prefix_ + ", " + std::to_string(threadId_));
        switch (change->getType()) {
        case EnvironmentChangeType::OBSTACLE_ADDED: {
            LOGGING("Apply obstacle added change");
            std::string sdfString = change->as<ObstacleAddedChange>()->getSDFString();
            SDFEnvironmentParser sdfParser;
            VectorObstaclePtr obstacles = sdfParser.parseObstaclesFromSDFString(sdfString);
            for (auto & obstacle : obstacles) {
                if (obstacle)
                    addObstacle(obstacle, change->requiresCallback());
            }

            notifyPlanningEnvironments(change);
            break;
        }
        case EnvironmentChangeType::OBSTACLE_REMOVED: {
            std::string removedObstacleName = change->as<ObstacleRemovedChange>()->getObstacleName();
            LOGGING("Apply obstacle removed change " + removedObstacleName);
            removeObstacle(removedObstacleName, change->requiresCallback());
            notifyPlanningEnvironments(change);
            break;
        }
        case EnvironmentChangeType::OBSTACLE_POSE_CHANGED: {
            const std::string obstacleName = change->as<ObstaclePoseChange>()->getObstacleName();
            LOGGING("Apply obstacle pose change " + obstacleName);
            const VectorFloat pose = change->as<ObstaclePoseChange>()->getPoseVec();
            changeObstaclePose(obstacleName, pose, change->requiresCallback());
            notifyPlanningEnvironments(change);
            break;
        }
        default: {
            WARNING("Change type not recognized");
            break;
        }

        }

        change = environmentChanges_->getNextChange();
    }

#ifdef USE_RVIZ
    auto viewer = robot_->getViewer();
    if (numChanges > 0 && viewer) {             
        static_cast<ViewerPublisher *>(viewer)->updateFromEnvironmentInfo(environmentInfo_, true);        
    }
#endif
}

void RobotEnvironment::notifyPlanningEnvironments(const EnvironmentChangeSharedPtr& environmentChange)
{
    // Calls onEnvironmentChanged for the registered planning environments.
    // This method is executed from the exec environment
    for (auto & environmentChangedNotificationFunction : onEnvironmentChangedFns_) {        
        environmentChangedNotificationFunction(environmentChange);
    }
}

void RobotEnvironment::registerForEnvironmentChanges(RobotEnvironment* planningEnvironment)
{
    for (auto & registeredEnvironment : registeredRobotEnvironments_) {
        if (planningEnvironment == registeredEnvironment) {
            LOGGING("RobotEnvironment already registered for changes");
            return;
        }
    }

    // This is executed in the exec environment and called by the planning environments
    LOGGING("Registering (" + 
        planningEnvironment->getPrefix() + 
        ", " + 
        std::to_string(planningEnvironment->getThreadId()) + 
        ") to (" + prefix_ + ", " + std::to_string(threadId_));
    OnEnvironmentChangedFunction onEnvironmentChangedFunction(std::bind(&RobotEnvironment::onEnvironmentChanged,
            planningEnvironment,
            std::placeholders::_1));
    onEnvironmentChangedFns_.push_back(onEnvironmentChangedFunction);
    registeredRobotEnvironments_.push_back(planningEnvironment);
}

bool RobotEnvironment::unregisterForEnvironmentChanges(RobotEnvironment *const planningEnvironment) {
    for (size_t i = 0; i != registeredRobotEnvironments_.size(); ++i) {
        if (registeredRobotEnvironments_[i] == planningEnvironment) {
            LOGGING("Unregistering environment (" + 
                planningEnvironment->getPrefix() +
                ", " +
                std::to_string(planningEnvironment->getThreadId()) +
                ") from (" +
                prefix_ + ", " + std::to_string(threadId_) + ")");
            registeredRobotEnvironments_.erase(registeredRobotEnvironments_.begin() + i);
            onEnvironmentChangedFns_.erase(onEnvironmentChangedFns_.begin() + i);            
            return true;
        }
    }

    return false;
}

void RobotEnvironment::onEnvironmentChanged(const EnvironmentChangeSharedPtr& environmentChange)
{
    // This is executed by the planning environments when they get informed
    // by the exec environment that there was an environment change
    LOGGING("Applying change in registered environment " + prefix_ + ", " + std::to_string(threadId_));    
    switch (environmentChange->getType()) {
    case EnvironmentChangeType::OBSTACLE_ADDED: {
        std::string sdfString = environmentChange->as<ObstacleAddedChange>()->getSDFString();
        SDFEnvironmentParser sdfParser;
        VectorObstaclePtr obstacles = sdfParser.parseObstaclesFromSDFString(sdfString);
        for (auto & obstacle : obstacles) {
            if (obstacle) {
                addObstacle(obstacle, environmentChange->requiresCallback());
                notifyPlanningEnvironments(environmentChange);
            }
        }

        break;
    }
    case EnvironmentChangeType::OBSTACLE_REMOVED: {
        std::string obstacleName = environmentChange->as<ObstacleRemovedChange>()->getObstacleName();
        removeObstacle(obstacleName, environmentChange->requiresCallback());
        notifyPlanningEnvironments(environmentChange);
        break;
    }
    case EnvironmentChangeType::OBSTACLE_POSE_CHANGED: {
        std::string obstacleName = environmentChange->as<ObstaclePoseChange>()->getObstacleName();
        VectorFloat pose = environmentChange->as<ObstaclePoseChange>()->getPoseVec();
        changeObstaclePose(obstacleName, pose, environmentChange->requiresCallback());
        notifyPlanningEnvironments(environmentChange);
        break;
    }
    default: {
        WARNING("Change type not recognized");
        break;
    }
    }
}

void RobotEnvironment::setRandomEngine(RandomEnginePtr randomEngine)
{
    randomEngine_ = randomEngine;
}

void RobotEnvironment::makeEnvironmentInfo(const bool& interactive)
{
    environmentInfo_ = std::make_shared<oppt::EnvironmentInfo>(interactive);
    environmentInfo_->setScene(scene_);
    robot_->setEnvironmentInfo(environmentInfo_);
    robot_->initEnvironmentCallbacks();
}

void RobotEnvironment::setEnvironmentInfo(std::shared_ptr<oppt::EnvironmentInfo>& environmentInfo)
{
    environmentInfo_ = environmentInfo;
    scene_ = environmentInfo_->getScene();
    robot_->setEnvironmentInfo(environmentInfo_);
    robot_->initEnvironmentCallbacks();
}

oppt::Robot* RobotEnvironment::getRobot() const
{
    return robot_.get();
}

bool RobotEnvironment::loadEnvironment(std::string environment_file, const std::string& robotName, const bool& reset)
{
    scene_ = std::make_shared<SceneImpl>();
    SDFEnvironmentParser sdfParser;
    VectorObstaclePtr obstacles = sdfParser.parseObstaclesFromFile(environment_file, robotName);
    scene_->addObstacles(obstacles);
    if (!robot_) {
        ERROR("RobotEnvironment: Error: Robot has not been initialized yet");
    }

    if (reset)
        gazeboInterface_->reset();
    return true;
}

bool RobotEnvironment::addObstacle(oppt::ObstacleSharedPtr& obstacle, bool executeCallbacks)
{
    environmentInfo_->getScene()->addObstacle(obstacle, executeCallbacks);
#ifdef USE_RVIZ
    auto viewer = robot_->getViewer();
    if (viewer) {
        static_cast<ViewerPublisher *>(viewer)->updateFromEnvironmentInfo(environmentInfo_);
    }
#endif
    return true;

}

bool RobotEnvironment::changeObstaclePose(const std::string& name, const VectorFloat& poseVec, bool executeCallbacks)
{
    environmentInfo_->getScene()->changeObstaclePose(name, poseVec, executeCallbacks);
#ifdef USE_RVIZ
    auto viewer = robot_->getViewer();
    if (viewer)
        static_cast<ViewerPublisher *>(viewer)->updateFromEnvironmentInfo(environmentInfo_);
#endif
    return true;
}

bool RobotEnvironment::removeObstacle(std::string obstacleName, bool executeCallbacks)
{
    environmentInfo_->getScene()->removeObstacle(obstacleName, executeCallbacks);
    return true;
}

const std::string RobotEnvironment::getWorldFile() const
{
    return worldFile_;
}

std::vector<RobotEnvironment*> RobotEnvironment::getRegisteredRobotEnvironments() const
{
    return registeredRobotEnvironments_;
}

bool RobotEnvironment::setExecutionEnvironment(const bool& executionEnvironment)
{
    isExecutionEnvironment_ = executionEnvironment;
    return true;
}

void RobotEnvironment::setThreadId(const unsigned int& threadId)
{
    threadId_ = threadId;
}

const unsigned int RobotEnvironment::getThreadId() const
{
    return threadId_;
}

const bool RobotEnvironment::isExecutionEnvironment() const
{
    return isExecutionEnvironment_;
}

const SceneSharedPtr RobotEnvironment::getScene() const
{
    return scene_;
}

void RobotEnvironment::loadRewardPlugin(const std::string& pluginFile,
                                        const std::string& optionsFile)
{
    if (rewardPlugin_) {               
        //rewardPlugin_->close();
    }    
    
    rewardPlugin_ =
        RewardPlugin::Create(pluginFile, "rewardPlugin");
    
    if (!rewardPlugin_->load(this, optionsFile)) {
        ERROR("Reward plugin '" +
              pluginFile +
              "' couldn't be loaded");
    }

    rewardPluginFile_ = pluginFile;
    rewardPluginOptionsFile_ = optionsFile;
}

FloatType RobotEnvironment::getReward(const PropagationResultSharedPtr& propagationResult) const
{
    if (!rewardPlugin_)
        ERROR("No reward plugin loaded");
    return rewardPlugin_->getReward(propagationResult);
}

RewardPlugin *const RobotEnvironment::getRewardPlugin() const {
    return rewardPlugin_.get();
}

void RobotEnvironment::loadTerminalPlugin(const std::string& pluginFile,
        const std::string& optionsFile)
{
    //if (terminalPlugin_)
        //terminalPlugin_->close();
    terminalPlugin_ = TerminalPlugin::Create(pluginFile, "terminalPlugin");
    if (!terminalPlugin_->load(this, optionsFile))
        ERROR("Terminal plugin '" + pluginFile + "' couldn't be loaded");
    terminalPluginFile_ = pluginFile;
    terminalPluginOptionsFile_ = optionsFile;
}

void RobotEnvironment::loadInitialBeliefPlugin(const std::string& pluginFile,
        const std::string& optionsFile)
{
    //if (initialBeliefPlugin_)
        //initialBeliefPlugin_->close();    
    initialBeliefPlugin_ = InitialBeliefPlugin::Create(pluginFile, "initialBeliefPlugin");
    if (!initialBeliefPlugin_->load(this, optionsFile))
        ERROR("InitialBeliefPlugin '" + pluginFile +
              "' couldn't be loaded (the library was found, but the load() method returned false)");
    initialBeliefPluginFile_ = pluginFile;
    initialBeliefOptionsFile_ = optionsFile;
}

bool RobotEnvironment::isTerminal(const PropagationResultSharedPtr& propagationResult) const
{
    return terminalPlugin_->isTerminal(propagationResult);
}

bool RobotEnvironment::isValid(const PropagationResultSharedPtr& propagationResult) const
{
    return terminalPlugin_->isValid(propagationResult)->isValid;
}

RobotStateSharedPtr RobotEnvironment::sampleInitialState() const
{
    if (!initialBeliefPlugin_)
        ERROR("Initial belief plugin not loaded");    
    RobotStateSharedPtr initState = initialBeliefPlugin_->sampleAnInitState();
    RobotStateSharedPtr initialStateNormalized =
        robot_->getStateSpace()->normalizeState(initState);
    return initialStateNormalized;
}

bool RobotEnvironment::makeGazeboInterface()
{
    std::string worldFile = getWorldFile();
    std::string robotFile = getRobot()->getRobotFile();
    std::string prefix = getPrefix();
    unsigned int threadId = getThreadId();
    gazeboInterface_ = std::make_unique<GazeboInterface>(worldFile, robotFile, prefix, threadId);
    return true;
}

GazeboInterface* const RobotEnvironment::getGazeboInterface() const
{
    return gazeboInterface_.get();
}

InitialBeliefPlugin *const RobotEnvironment::getInitialBeliefPlugin() const {
    return initialBeliefPlugin_.get();
}

RobotEnvironmentSharedPtr RobotEnvironment::clone(const unsigned int& uid) {
    if (prefix_ == "exec")
        ERROR("Attempting to clone the robot operational environment");
    std::shared_ptr<RobotEnvironment> env = std::make_shared<RobotEnvironment>();
    env->setRandomEngine(randomEngine_);
    env->setThreadId(uid);
    env->createRobot<oppt::RobotImpl>(worldFile_,
                                      robot_path_,
                                      config_path_,
                                      prefix_,
                                      uid);
    env->getRobot()->initTransitionPlugin(robot_->transitionPluginFile_, env.get());
    env->getRobot()->initObservationPlugin(robot_->observationPluginFile_, env.get());
    env->getRobot()->init();
    env->getRobot()->normalizedSpaces_ = robot_->normalizedSpaces_;
    EnvironmentInfoSharedPtr clonedEnvironmentInfo = environmentInfo_->clone();
    env->setEnvironmentInfo(clonedEnvironmentInfo);
    StateSpaceInfo stateSpaceInfo = robot_->getStateSpace()->getInfo();
    env->getRobot()->makeStateSpace(stateSpaceInfo);
    ObservationSpaceInfo observationSpaceInfo = robot_->getObservationSpace()->getInfo();
    env->getRobot()->makeObservationSpace(observationSpaceInfo);
    ActionSpaceInfo actionSpaceInfo = robot_->getActionSpace()->getInfo();
    env->getRobot()->makeActionSpace(actionSpaceInfo);
    if (!env->getRobot()->getActionSpace()) {
        oppt::ERROR("RobotEnvironment: Robot has no action space!");
    }

    env->initializeEnvironmentCallbacks();
    env->loadRewardPlugin(rewardPluginFile_, rewardPluginOptionsFile_);
    env->loadTerminalPlugin(terminalPluginFile_, terminalPluginOptionsFile_);
    env->loadInitialBeliefPlugin(initialBeliefPluginFile_, initialBeliefOptionsFile_);

    env->getRobot()->loadTransitionPlugin(robot_->transitionPluginOptionsFile_, env.get());
    env->getRobot()->loadObservationPlugin(robot_->observationPluginOptionsFile_, env.get());
    env->getRobot()->setCollisionsAllowed(robot_->getCollisionsAllowed());

    auto ikSolver = std::move(robot_->getIKSolver()->clone());
    env->getRobot()->setIKSolver(std::move(ikSolver));
    return env;
}

}


