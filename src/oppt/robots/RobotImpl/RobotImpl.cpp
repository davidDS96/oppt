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
#include "oppt/robotHeaders/RobotImpl/RobotImpl.hpp"
#include "oppt/robotHeaders/RobotImpl/RobotImplSDFParser.hpp"
#include "oppt/robotHeaders/RobotImpl/RobotImplSpaceInformationFactory.hpp"
#include "oppt/opptCore/EnvironmentInfo.hpp"
#include "oppt/plugin/Plugin.hpp"
#include "oppt/opptCore/CollisionObject.hpp"
#ifdef USE_RVIZ
#include "oppt/viewerPublisher/ViewerPublisher.hpp"
#endif

namespace oppt
{

RobotImpl::RobotImpl(std::string worldFile,
                     std::string robotFile,
                     std::string configFile,
                     std::string prefix,
                     unsigned int& threadId):
    Robot(worldFile, robotFile, configFile, prefix, threadId),
    worldFile_(worldFile),
    robotFile_(robotFile),
    collisionLinks_(),
    linkVisualGeometryMap_(),
    dof_(0),
    robotConfigOptions_()
{
    std::unique_ptr <options::OptionParser> robotConfigParser =
        RobotConfigOptions::makeParser();
    robotConfigParser->setOptions(&robotConfigOptions_);
    robotConfigParser->parseCfgFile(configFile);
    robotConfigParser->finalize();
    serializer_.reset(new RobotImplSerializer(worldFile));

    // Default implementation of a discrete collision function
    discreteCollisionFunction_ = [this](const RobotStateSharedPtr & state) {
        RobotStateSharedPtr denormalizedState = stateSpace_->denormalizeState(state);
        oppt::CollisionReportSharedPtr collisionReport(new oppt::DiscreteCollisionReport());
        oppt::DiscreteCollisionReport* discreteCollisionReport =
            static_cast<oppt::DiscreteCollisionReport*>(collisionReport.get());
        discreteCollisionReport->collides = false;
        discreteCollisionReport->enableContact = true;
        createRobotCollisionObjects(denormalizedState);
        environmentInfo_->getScene()->makeDiscreteCollisionReport(collisionReport);
        return collisionReport;
    };

    // Default implementation of a continuous collision function
    continuousCollisionFunction_ = [this](const RobotStateSharedPtr & state1,
                                          const RobotStateSharedPtr & state2,
    const unsigned int& numIterations) {
        RobotStateSharedPtr denormalizedState1 = stateSpace_->denormalizeState(state1);
        RobotStateSharedPtr denormalizedState2 = stateSpace_->denormalizeState(state2);
        oppt::CollisionReportSharedPtr collisionReport(new oppt::ContinuousCollisionReport());
        oppt::ContinuousCollisionReport* continuousCollisionReport =
            static_cast<oppt::ContinuousCollisionReport*>(collisionReport.get());
        continuousCollisionReport->tfGoal = VectorFCLTransform3f(linkCollisionGeometryMap_.size());
        continuousCollisionReport->numIterations = numIterations;

        // Get the goal transforms
        createRobotCollisionObjects(denormalizedState2);
        unsigned int idx = 0;
        for (auto& robotCollisionObject : robotCollisionObjectsVec_) {
            // Unfortunately, we need to copy the transformation object here
            FCLTransform3f tf = robotCollisionObject->getCollisionObject()->getTransform();
            fcl::Vec3f vec = tf.getTranslation();
            fcl::Vec3f vec2(vec[0], vec[1], vec[2]);
            fcl::Quaternion3f quat = tf.getQuatRotation();
            fcl::Quaternion3f quat2(quat.getW(), quat.getX(), quat.getY(), quat.getZ());
            FCLTransform3f tf2(quat2, vec2);
            continuousCollisionReport->tfGoal[idx] = tf2;
            idx++;
        }

        createRobotCollisionObjects(denormalizedState1);
        environmentInfo_->getScene()->makeContinuousCollisionReport(collisionReport);
        return collisionReport;
    };

    // Default implementation of a dirty collision function
    dirtyCollisionFunction_ = [this](void *const data) {
        auto linkPoses = gazeboInterface_->getLinksCollisionWorldPosesDirty(collisionLinks_);
        for (size_t i = 0; i != linkPoses.size(); ++i) {
            fcl::Quaternion3f fclQuat(linkPoses[i][3],
                                      linkPoses[i][4],
                                      linkPoses[i][5],
                                      linkPoses[i][6]);
            fcl::Vec3f translationVector(linkPoses[i][0],
                                         linkPoses[i][1],
                                         linkPoses[i][2]);
            fcl::Transform3f trans(fclQuat, translationVector);
            robotCollisionObjectsVec_[i]->getCollisionObject()->setTransform(trans);
            if (robotCollisionObjectsVec_[i]->getCollisionObject()->collisionGeometry())
                robotCollisionObjectsVec_[i]->getCollisionObject()->computeAABB();

        }

        oppt::CollisionReportSharedPtr collisionReport(new oppt::DiscreteCollisionReport());
        oppt::DiscreteCollisionReport* discreteCollisionReport =
            static_cast<oppt::DiscreteCollisionReport*>(collisionReport.get());
        discreteCollisionReport->collides = false;
        discreteCollisionReport->enableContact = false;
        environmentInfo_->getScene()->makeDiscreteCollisionReport(collisionReport);        
        return collisionReport;
    };
}

bool RobotImpl::init()
{
    RobotImplSDFParser sdfParser;
    VectorGeometryPtr robotLinkCollisionGeometries = sdfParser.parseCollisionGeometries(robotFile_);
    VectorGeometryPtr invariantRobotCollisionGeometries;
    VectorString collisionInvariantLinks = robotConfigOptions_.collisionInvariantLinks;
    for (auto& geometry : robotLinkCollisionGeometries) {
        bool contains = false;
        for (auto& collisionInvariantLink : collisionInvariantLinks) {
            if (geometry->getName().find(collisionInvariantLink + "::") != std::string::npos) {
                contains = true;
            }
        }

        VectorString elems;
        split(geometry->getName(), "::", elems);
        std::string linkName = elems[0];
        if (!contains) {
            if (linkCollisionGeometryMap_.find(linkName) == linkCollisionGeometryMap_.end())
                linkCollisionGeometryMap_[linkName] = std::vector<GeometrySharedPtr>();
            linkCollisionGeometryMap_.at(linkName).push_back(geometry);
        } else {
            invariantRobotCollisionGeometries.push_back(geometry);
        }
    }

    VectorGeometryPtr robotLinkVisualGeometries = sdfParser.parseVisualGeometries(robotFile_);
    for (auto& geometry : robotLinkVisualGeometries) {
        linkVisualGeometryMap_[geometry->getName()] = geometry;
    }

    initCollisionObjects();

    // Make the collision objects for the collision invariant links    
    for (auto &linkGeometry : invariantRobotCollisionGeometries) {
        CollisionGeometrySharedPtr linkCollisionGeometry = linkGeometry->getCollisionGeometry();
        fcl::Matrix3f rotationMatrix(1, 0, 0,
                                     0, 1, 0,
                                     0, 0, 1);
        fcl::Vec3f translationVector(0, 0, 0);
        fcl::Transform3f trans(rotationMatrix, translationVector);
        CollisionObjectSharedPtr collObj(new fcl::CollisionObject(linkCollisionGeometry, trans));
        OpptCollisionObjectSharedPtr opptCollisionObject = std::make_shared<OpptCollisionObject>(collObj, linkGeometry->getName());
        invariantRobotCollisionObjectsVec_.push_back(opptCollisionObject);
    }

    VectorString robotLinksInWorld = gazeboInterface_->getRobotLinkNames();
    for (auto& link : collisionInvariantLinks) {
        if (!contains(robotLinksInWorld, link))
            ERROR("link '" + link + "' not defined in your robot model");
    }
    VectorString robotLinks = gazeboInterface_->getRobotLinkNames();
    //TODO: Filter out dimensionless links
    collisionLinks_ = VectorString();
    for (auto &entry : linkCollisionGeometryMap_) {
        collisionLinks_.push_back(entry.first);
    }

    gazeboInterface_->setCollisionCheckedLinks(collisionLinks_);
    static_cast<RobotImplSerializer*>(serializer_.get())->setWorld(gazeboInterface_->getWorld());
}

VectorString RobotImpl::getCollisionCheckedLinks() const {
    return collisionLinks_;
}

bool RobotImpl::makeStateSpace(StateSpaceInfo& stateSpaceInfo)
{
    const StateSpaceInformationPtr stateSpaceInformation =
        SpaceInformationFactory::makeStateSpaceInformation(&robotConfigOptions_, gazeboInterface_);
    gazeboInterface_->setStateSpaceInformation(stateSpaceInformation);
    VectorFloat lowerLimits, upperLimits;
    gazeboInterface_->getStateLimits(lowerLimits, upperLimits);

    // Limits for additional dimensions
    for (auto& lowerUpper : stateSpaceInformation->lowerUpperLimitsAdditional) {
        lowerLimits.push_back(lowerUpper[0]);
        upperLimits.push_back(lowerUpper[1]);
    }

    stateSpaceInfo.numDimensions = lowerLimits.size();
    stateSpace_ = std::make_shared<VectorStateSpace>(stateSpaceInfo);
    if (stateSpace_->getNumDimensions() == 0)
        ERROR("The state space has zero dimensions. "
              "Have you forgotten to specify states in your robot configuration file?");
    DistanceMetric distanceMetric = oppt::EuclideanMetric();
    stateSpace_->setDistanceMetric(distanceMetric);
    stateSpace_->as<VectorStateSpace>()->makeStateLimits(lowerLimits, upperLimits);
    dof_ = stateSpaceInformation->jointPositions.size();
    dof_ += stateSpaceInformation->containedLinkPositionsX.size();
    dof_ += stateSpaceInformation->containedLinkPositionsY.size();
    dof_ += stateSpaceInformation->containedLinkPositionsZ.size();
    dof_ += 6 * stateSpaceInformation->containedLinkPoses.size();
    serializer_->as<RobotImplSerializer>()->setStateSpace(stateSpace_.get());
    //static_cast<RobotImplIntegrator*>(integrator.get())->setStateSpace(stateSpace_);
}

bool RobotImpl::makeActionSpace(ActionSpaceInfo& actionSpaceInfo)
{
    const ActionSpaceInformationPtr actionSpaceInformation =
        SpaceInformationFactory::makeActionSpaceInformation(&robotConfigOptions_, gazeboInterface_);
    gazeboInterface_->setActionSpaceInformation(actionSpaceInformation);
    VectorFloat lowerLimits, upperLimits;
    gazeboInterface_->getActionLimits(lowerLimits, upperLimits);

    // Limits for additional dimensions
    for (auto& lowerUpper : actionSpaceInformation->lowerUpperLimitsAdditional) {
        lowerLimits.push_back(lowerUpper.first);
        upperLimits.push_back(lowerUpper.second);
    }

    actionSpaceInfo.numDimensions = lowerLimits.size();
    actionSpace_ = std::make_shared<oppt::ContinuousVectorActionSpace>(actionSpaceInfo);
    actionSpace_->as<ContinuousVectorActionSpace>()->makeActionLimits(lowerLimits, upperLimits);

    if (actionSpace_->getNumDimensions() == 0)
        ERROR("The action space has zero dimensions. "
              "Have you forgotten to specify actions in your robot configuration file?");
}

bool RobotImpl::makeObservationSpace(ObservationSpaceInfo& observationSpaceInfo)
{
    const ObservationSpaceInformationPtr observationSpaceInformation =
        SpaceInformationFactory::makeObservationSpaceInformation(&robotConfigOptions_, gazeboInterface_);
    gazeboInterface_->setObservationSpaceInformation(observationSpaceInformation);
    VectorFloat lowerLimits, upperLimits;

    if (!(robotConfigOptions_.ignoreGazeboSensors))
        gazeboInterface_->getObservationLimits(lowerLimits, upperLimits);

    // Limits for additional dimensions
    for (auto& lowerUpper : observationSpaceInformation->lowerUpperLimitsAdditional) {
        lowerLimits.push_back(lowerUpper[0]);
        upperLimits.push_back(lowerUpper[1]);
    }

    observationSpaceInfo.numDimensions = lowerLimits.size();
    gazeboInterface_->setObservationSpaceDimension(observationSpaceInfo.numDimensions);
    observationSpace_ = std::make_shared<VectorObservationSpace>(observationSpaceInfo);
    if (observationSpace_->getNumDimensions() == 0)
        ERROR("The observation space has zero dimensions. "
              "Have you forgotten to specify observations in your robot configuration file?");
    observationSpace_->as<VectorObservationSpace>()->makeObservationLimits(lowerLimits, upperLimits);
}

void RobotImpl::createRobotCollisionObjects(const oppt::RobotStateSharedPtr& state) const
{
    auto linkPoses = gazeboInterface_->getLinksCollisionWorldPoses(state->as<VectorState>()->asVector(), nullptr, collisionLinks_);
    for (size_t i = 0; i != linkPoses.size(); ++i) {
        fcl::Quaternion3f fclQuat(linkPoses[i][3],
                                  linkPoses[i][4],
                                  linkPoses[i][5],
                                  linkPoses[i][6]);
        fcl::Vec3f translationVector(linkPoses[i][0],
                                     linkPoses[i][1],
                                     linkPoses[i][2]);
        FCLTransform3f trans(fclQuat, translationVector);
        robotCollisionObjectsVec_[i]->getCollisionObject()->setTransform(trans);
        if (robotCollisionObjectsVec_[i]->getCollisionObject()->collisionGeometry())
            robotCollisionObjectsVec_[i]->getCollisionObject()->computeAABB();
    }
}

CollisionReportSharedPtr RobotImpl::makeDiscreteCollisionReportDirty() const
{
    if (!dirtyCollisionFunction_)
        ERROR("No oppt::DirtyCollisionFunction defined");
    return dirtyCollisionFunction_(nullptr);
}

unsigned int RobotImpl::getDOF() const
{
    return dof_;
}

void RobotImpl::updateViewer(const oppt::RobotStateSharedPtr& state,
                             const std::vector<oppt::RobotStateSharedPtr>& particles,
                             const VectorFloat& particleColor,
                             const bool& keepParticles,
                             const bool& deleteExistingParticles)
{
#ifdef USE_RVIZ
    if (viewer_ && viewer_->viewerRunning()) {
        RobotStateSharedPtr denormalizedState = stateSpace_->denormalizeState(state);
        VectorFloat stateVec = denormalizedState->as<VectorState>()->asVector();
        VectorGeometryPtr robotGeometries;

        VectorString visualNames(linkVisualGeometryMap_.size());
        size_t k = 0;
        for (auto& linkGeomEntry : linkVisualGeometryMap_) {
            visualNames[k] = linkGeomEntry.first;
            k++;
        }
        std::vector<Matrixdf> visualPoses;
        gazeboInterface_->getRobotVisualPoses(stateVec, visualNames, visualPoses, denormalizedState->getGazeboWorldState());
        for (size_t i = 0; i < visualNames.size(); i++) {
            const GeometrySharedPtr geomInMap = linkVisualGeometryMap_.at(visualNames[i]);
            GeometrySharedPtr geom = geomInMap->shallowCopy();
            geom->setPose(visualPoses[i]);
            robotGeometries.push_back(geom);
        }

        std::vector<std::vector<Matrixdf>> particlePoses(particles.size());
        for (size_t i = 0; i < particles.size(); i++) {
            std::vector<Matrixdf> particlePose;
            denormalizedState = stateSpace_->denormalizeState(particles[i]);
            stateVec = denormalizedState->as<VectorState>()->asVector();
            gazeboInterface_->getRobotVisualPoses(stateVec, visualNames, visualPoses, denormalizedState->getGazeboWorldState());
            for (size_t j = 0; j < visualNames.size(); j++) {
                particlePose.push_back(visualPoses[j]);
            }

            particlePoses.push_back(particlePose);
        }

        static_cast<ViewerPublisher*>(viewer_.get())->drawGeometries(robotGeometries,
                particlePoses,
                particleColor,
                keepParticles,
                deleteExistingParticles);
    }
#endif
}

void RobotImpl::setEnvironmentInfo(oppt::EnvironmentInfoSharedPtr& environmentInfo)
{
    Robot::setEnvironmentInfo(environmentInfo);
    gazeboInterface_->setInteractive(environmentInfo->isInteractive());
}

void RobotImpl::initEnvironmentCallbacks()
{
    if (environmentInfo_->isInteractive()) {
        std::function<void(oppt::ObstacleSharedPtr&)> addObstacleFn =
            std::bind(&RobotImpl::addObstacleCallback, this, std::placeholders::_1);
        std::function<void(std::string)> removeObstacleFn =
            std::bind(&RobotImpl::removeObstacleCallback, this, std::placeholders::_1);
        std::function<void(const std::string&, const VectorFloat&)> changePoseFn =
            std::bind(&RobotImpl::changeObstaclePoseCallback, this, std::placeholders::_1, std::placeholders::_2);
        environmentInfo_->getScene()->setAddObstacleCallback(addObstacleFn);
        environmentInfo_->getScene()->setRemoveObstacleCallback(removeObstacleFn);
        environmentInfo_->getScene()->setChangeObstaclePoseCallback(changePoseFn);
    }
}

void RobotImpl::registerOnAddObstacleCallback(std::function<void(const std::string&)>& callback)
{
    if (environmentInfo_->isInteractive())
        gazeboInterface_->registerOnAddObstacleCallback(callback);
}

void RobotImpl::registerOnRemoveObstacleCallback(std::function<void(const std::string&)>& callback)
{
    if (environmentInfo_->isInteractive())
        gazeboInterface_->registerOnRemoveObstacleCallback(callback);
}

void RobotImpl::registerOnPoseChangedCallback(std::function < void(const std::string&,
        const VectorFloat&) >& callback)
{
    if (environmentInfo_->isInteractive())
        gazeboInterface_->registerOnPoseChangedCallback(callback);
}

void RobotImpl::addObstacleCallback(oppt::ObstacleSharedPtr& obstacle)
{
    std::string sdfString = obstacle->toSDFString();
    gazeboInterface_->addObstacleFromSDFString(sdfString);
}

void RobotImpl::removeObstacleCallback(std::string name)
{
    gazeboInterface_->removeObstacle(name);
}

void RobotImpl::changeObstaclePoseCallback(const std::string& name, const VectorFloat& poseVec)
{
    gazeboInterface_->changeModelPose(name, poseVec);
}

void RobotImpl::setCollisionsAllowed(const bool& collisionsAllowed) const
{
    Robot::setCollisionsAllowed(collisionsAllowed);
    if (!collisionsAllowed)
        gazeboInterface_->setDirtyCollisionFunction(dirtyCollisionFunction_);
}

void RobotImpl::setDirtyCollisionFunction(DirtyCollisionFunction dirtyCollisionFunction)
{
    dirtyCollisionFunction_ = dirtyCollisionFunction;
    gazeboInterface_->setDirtyCollisionFunction(dirtyCollisionFunction_);

}

}
