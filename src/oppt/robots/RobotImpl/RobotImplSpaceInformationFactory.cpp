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
#include "oppt/robotHeaders/RobotImpl/RobotImplSpaceInformationFactory.hpp"
#include "oppt/opptCore/StateInformation.hpp"
#include "oppt/opptCore/ActionInformation.hpp"
#include "oppt/opptCore/ObservationInformation.hpp"

namespace oppt
{
namespace SpaceInformationFactory
{
const StateSpaceInformationPtr makeStateSpaceInformation(const RobotConfigOptions* robotConfigOptions,
        const GazeboInterface* gazeboInterface)
{
    VectorString robotJointsInWorld = gazeboInterface->getRobotJointNames();
    VectorString robotLinksInWorld = gazeboInterface->getLinkNames();
    VectorString jointsInConfigFile = robotConfigOptions->jointPositions;
    jointsInConfigFile.insert(jointsInConfigFile.end(),
                              robotConfigOptions->jointPositions.begin(),
                              robotConfigOptions->jointPositions.end());

    VectorString linksInConfigFile = robotConfigOptions->linkPoses;
    linksInConfigFile.insert(linksInConfigFile.end(),
                             robotConfigOptions->linkPoses.begin(),
                             robotConfigOptions->linkPoses.end());
    linksInConfigFile.insert(linksInConfigFile.end(),
                             robotConfigOptions->linkVelocitiesLinear.begin(),
                             robotConfigOptions->linkVelocitiesLinear.end());
    linksInConfigFile.insert(linksInConfigFile.end(),
                             robotConfigOptions->linkVelocitiesAngular.begin(),
                             robotConfigOptions->linkVelocitiesAngular.end());
    for (auto & joint : jointsInConfigFile) {
        if (!contains(robotJointsInWorld, joint))
            ERROR("joint '" + joint + "' not defined in your robot model");
    }

    for (auto & link : linksInConfigFile) {
        if (!contains(robotLinksInWorld, link))
            ERROR("link '" + link + "' not defined in your robot model");
    }

    StateSpaceInformationPtr stateSpaceInformation(new StateSpaceInformation());
    stateSpaceInformation->jointPositions = robotConfigOptions->jointPositions;
    stateSpaceInformation->jointVelocities = robotConfigOptions->jointVelocities;
    stateSpaceInformation->containedLinkPoses = robotConfigOptions->linkPoses;
    stateSpaceInformation->containedLinkPosesLowerLimits = robotConfigOptions->linkPosesLowerLimits;
    stateSpaceInformation->containedLinkPosesUpperLimits = robotConfigOptions->linkPosesUpperLimits;

    stateSpaceInformation->containedLinkPositionsX = robotConfigOptions->linkPositionsX;
    stateSpaceInformation->containedLinkPositionsXLimits = robotConfigOptions->linkPositionsXLimits;
    stateSpaceInformation->containedLinkPositionsY = robotConfigOptions->linkPositionsY;
    stateSpaceInformation->containedLinkPositionsYLimits = robotConfigOptions->linkPositionsYLimits;
    stateSpaceInformation->containedLinkPositionsZ = robotConfigOptions->linkPositionsZ;
    stateSpaceInformation->containedLinkPositionsZLimits = robotConfigOptions->linkPositionsZLimits;

    stateSpaceInformation->containedLinkOrientationsX = robotConfigOptions->linkOrientationsX;
    stateSpaceInformation->containedLinkOrientationsXLimits = robotConfigOptions->linkOrientationsXLimits;
    stateSpaceInformation->containedLinkOrientationsY = robotConfigOptions->linkOrientationsY;
    stateSpaceInformation->containedLinkOrientationsYLimits = robotConfigOptions->linkOrientationsYLimits;
    stateSpaceInformation->containedLinkOrientationsZ = robotConfigOptions->linkOrientationsZ;
    stateSpaceInformation->containedLinkOrientationsZLimits = robotConfigOptions->linkOrientationsZLimits;

    stateSpaceInformation->containedLinkLinearVelocitiesX = robotConfigOptions->linkLinearVelocitiesX;
    stateSpaceInformation->containedLinkLinearVelocitiesY = robotConfigOptions->linkLinearVelocitiesY;
    stateSpaceInformation->containedLinkLinearVelocitiesZ = robotConfigOptions->linkLinearVelocitiesZ;
    stateSpaceInformation->containedLinkLinearVelocitiesXLimits = robotConfigOptions->linkLinearVelocitiesXLimits;
    stateSpaceInformation->containedLinkLinearVelocitiesYLimits = robotConfigOptions->linkLinearVelocitiesYLimits;
    stateSpaceInformation->containedLinkLinearVelocitiesZLimits = robotConfigOptions->linkLinearVelocitiesZLimits;
    stateSpaceInformation->containedLinkVelocitiesLinear = robotConfigOptions->linkVelocitiesLinear;
    stateSpaceInformation->containedLinkVelocitiesLinearLimits = robotConfigOptions->linkVelocitiesLinearLimits;

    stateSpaceInformation->containedLinkAngularVelocitiesX = robotConfigOptions->linkAngularVelocitiesX;
    stateSpaceInformation->containedLinkAngularVelocitiesY = robotConfigOptions->linkAngularVelocitiesY;
    stateSpaceInformation->containedLinkAngularVelocitiesZ = robotConfigOptions->linkAngularVelocitiesZ;
    stateSpaceInformation->containedLinkAngularVelocitiesXLimits = robotConfigOptions->linkAngularVelocitiesXLimits;
    stateSpaceInformation->containedLinkAngularVelocitiesYLimits = robotConfigOptions->linkAngularVelocitiesYLimits;
    stateSpaceInformation->containedLinkAngularVelocitiesZLimits = robotConfigOptions->linkAngularVelocitiesZLimits;

    stateSpaceInformation->containedLinkVelocitiesAngular = robotConfigOptions->linkVelocitiesAngular;
    stateSpaceInformation->containedLinkVelocitiesAngularLimits = robotConfigOptions->linkVelocitiesAngularLimits;

    stateSpaceInformation->additionalDimensions = robotConfigOptions->additionalStateDimensions;
    stateSpaceInformation->lowerUpperLimitsAdditional = robotConfigOptions->lowerUpperLimitsAdditional;
    std::vector<VectorString> redundantJoints = robotConfigOptions->redundantJoints;
    VectorString nonRedundantJointPositions;
    VectorString resolvedRedundantJointPositions;
    VectorString nonRedundantJointVelocities;
    bool breaking;

    for (size_t i = 0; i < stateSpaceInformation->jointPositions.size(); i++) {
        bool containing = false;
        for (auto & redundantJointList : redundantJoints) {
            if (contains(redundantJointList, stateSpaceInformation->jointPositions[i])) {
                if (!contains(nonRedundantJointPositions, redundantJointList[0])) {
                    nonRedundantJointPositions.push_back(redundantJointList[0]);
                }
                containing = true;
                break;
            }
        }
        if (!containing) {
            if (!contains(nonRedundantJointPositions, stateSpaceInformation->jointPositions[i])) {
                nonRedundantJointPositions.push_back(stateSpaceInformation->jointPositions[i]);
            }
        }
    }

    for (size_t i = 0; i < stateSpaceInformation->jointVelocities.size(); i++) {
        bool containing = false;
        for (auto & redundantJointList : redundantJoints) {
            if (contains(redundantJointList, stateSpaceInformation->jointVelocities[i])) {
                if (!contains(nonRedundantJointVelocities, redundantJointList[0])) {
                    nonRedundantJointVelocities.push_back(redundantJointList[0]);
                }
                containing = true;
                break;
            }
        }
        if (!containing) {
            if (!contains(nonRedundantJointVelocities, stateSpaceInformation->jointVelocities[i])) {
                nonRedundantJointVelocities.push_back(stateSpaceInformation->jointVelocities[i]);
            }
        }
    }

    stateSpaceInformation->jointPositions = nonRedundantJointPositions;
    stateSpaceInformation->jointVelocities = nonRedundantJointVelocities;
    stateSpaceInformation->redundantJoints = redundantJoints;
    checkSpaceLimits(stateSpaceInformation.get(), robotConfigOptions, "stateSpace");
    return stateSpaceInformation;
}

void checkSpaceLimits(SpaceInformation* spaceInformation,
                      const RobotConfigOptions* robotConfigOptions,
                      const std::string spaceType)
{
    if (robotConfigOptions->normalizedSpaces) {
        if (spaceInformation->containedLinkPoses.size() != spaceInformation->containedLinkPosesLowerLimits.size())
            ERROR("When you use normalized spaces you need to set linkPosesLowerLimits correctly");
        if (spaceInformation->containedLinkPoses.size() != spaceInformation->containedLinkPosesUpperLimits.size())
            ERROR("When you use normalized spaces you need to set linkPosesUpperLimits correctly");

        if (spaceInformation->containedLinkPositionsX.size() != spaceInformation->containedLinkPositionsXLimits.size())
            ERROR("When you use normalized spaces you need to set linkPositionXLimits correctly");
        if (spaceInformation->containedLinkPositionsY.size() != spaceInformation->containedLinkPositionsYLimits.size())
            ERROR("When you use normalized spaces you need to set linkPositionYLimits correctly");
        if (spaceInformation->containedLinkPositionsZ.size() != spaceInformation->containedLinkPositionsZLimits.size())
            ERROR("When you use normalized spaces you need to set linkPositionZLimits correctly");

        if (spaceInformation->containedLinkOrientationsX.size() != spaceInformation->containedLinkOrientationsXLimits.size())
            ERROR("When you use normalized spaces you need to set linkOrientationsXLimits correctly");
        if (spaceInformation->containedLinkOrientationsY.size() != spaceInformation->containedLinkOrientationsYLimits.size())
            ERROR("When you use normalized spaces you need to set linkOrientationsYLimits correctly");
        if (spaceInformation->containedLinkOrientationsZ.size() != spaceInformation->containedLinkOrientationsZLimits.size())
            ERROR("When you use normalized spaces you need to set linkOrientationsZLimits correctly");

        if (spaceInformation->containedLinkLinearVelocitiesX.size() !=
                spaceInformation->containedLinkLinearVelocitiesXLimits.size())
            ERROR("When you use normalized spaces you need to set linkLineariVelocityXLimits correctly");
        if (spaceInformation->containedLinkLinearVelocitiesY.size() !=
                spaceInformation->containedLinkLinearVelocitiesYLimits.size())
            ERROR("When you use normalized spaces you need to set linkLineariVelocityYLimits correctly");
        if (spaceInformation->containedLinkLinearVelocitiesZ.size() !=
                spaceInformation->containedLinkLinearVelocitiesZLimits.size())
            ERROR("When you use normalized spaces you need to set linkLineariVelocityZLimits correctly");


        if (spaceInformation->containedLinkVelocitiesLinear.size() !=
                spaceInformation->containedLinkVelocitiesLinearLimits.size())
            ERROR("When you use normalized spaces you need to set linkVelocitiesLinearLimits correctly");
        if (spaceInformation->containedLinkVelocitiesAngular.size() !=
                spaceInformation->containedLinkVelocitiesAngularLimits.size())
            ERROR("When you use normalized spaces you need to set linkVelocitiesAngularLimits correctly");
	
	if (spaceInformation->additionalDimensions != spaceInformation->lowerUpperLimitsAdditional.size())
            ERROR("When you use normalized spaces you need to set additionalDimensionLimits correctly");
    }

    FloatType infLimit = std::numeric_limits<FloatType>::infinity();

    // ######################################################################
    // Link poses
    // ######################################################################
    if (spaceInformation->containedLinkPosesLowerLimits.size() == 0) {
        for (size_t i = 0; i != spaceInformation->containedLinkPoses.size(); ++i) {
            VectorFloat lowerLimits;
            for (size_t j = 0; j != 6; ++j) {
                lowerLimits.push_back(-infLimit);
            }
            spaceInformation->containedLinkPosesLowerLimits.push_back(lowerLimits);
        }
    }
    if (spaceInformation->containedLinkPosesUpperLimits.size() == 0) {
        for (size_t i = 0; i != spaceInformation->containedLinkPoses.size(); ++i) {
            VectorFloat upperLimits;
            for (size_t j = 0; j != 6; ++j) {
                upperLimits.push_back(infLimit);
            }
            spaceInformation->containedLinkPosesUpperLimits.push_back(upperLimits);
        }
    }
    for (size_t i = 0; i != spaceInformation->containedLinkPosesLowerLimits.size(); ++i) {
        if (spaceInformation->containedLinkPosesLowerLimits[i].size() != 6)
            ERROR("Wrong formatting for linkPosesLowerLimits");
        if (spaceInformation->containedLinkPosesUpperLimits[i].size() != 6)
            ERROR("Wrong formatting for linkPosesUpperLimits");
        for (size_t j = 0; j != 6; ++j) {
            if (spaceInformation->containedLinkPosesLowerLimits[i][j] >
                    spaceInformation->containedLinkPosesUpperLimits[i][j])
                ERROR("linkPosesLowerLimits must be smaller than linkPosesUpperLimits");
        }
    }

    // ######################################################################
    // Link positions
    // ######################################################################
    if (spaceInformation->containedLinkPositionsXLimits.size() == 0) {
        for (size_t i = 0; i != spaceInformation->containedLinkPositionsX.size(); ++i) {
            VectorFloat limits( { -infLimit, infLimit});
            spaceInformation->containedLinkPositionsXLimits.push_back(limits);
        }
    }
    for (size_t i = 0; i != spaceInformation->containedLinkPositionsXLimits.size(); ++i) {
        if (spaceInformation->containedLinkPositionsXLimits[i][0] > spaceInformation->containedLinkPositionsXLimits[i][1])
            ERROR("linkPositionsXLimits is malformed");
    }
    if (spaceInformation->containedLinkPositionsYLimits.size() == 0) {
        for (size_t i = 0; i != spaceInformation->containedLinkPositionsY.size(); ++i) {
            VectorFloat limits( { -infLimit, infLimit});
            spaceInformation->containedLinkPositionsYLimits.push_back(limits);
        }
    }
    for (size_t i = 0; i != spaceInformation->containedLinkPositionsYLimits.size(); ++i) {
        if (spaceInformation->containedLinkPositionsYLimits[i][0] > spaceInformation->containedLinkPositionsYLimits[i][1])
            ERROR("linkPositionsYLimits is malformed");
    }
    if (spaceInformation->containedLinkPositionsZLimits.size() == 0) {
        for (size_t i = 0; i != spaceInformation->containedLinkPositionsZ.size(); ++i) {
            VectorFloat limits( { -infLimit, infLimit});
            spaceInformation->containedLinkPositionsZLimits.push_back(limits);
        }
    }
    for (size_t i = 0; i != spaceInformation->containedLinkPositionsZLimits.size(); ++i) {
        if (spaceInformation->containedLinkPositionsZLimits[i][0] > spaceInformation->containedLinkPositionsZLimits[i][1])
            ERROR("linkPositionsZLimits is malformed");
    }

    // ######################################################################
    // Link orientations
    // ######################################################################
    if (spaceInformation->containedLinkOrientationsXLimits.size() == 0) {
        for (size_t i = 0; i != spaceInformation->containedLinkOrientationsX.size(); ++i) {
            VectorFloat limits( { -infLimit, infLimit});
            spaceInformation->containedLinkOrientationsXLimits.push_back(limits);
        }
    }
    for (size_t i = 0; i != spaceInformation->containedLinkOrientationsXLimits.size(); ++i) {
        if (spaceInformation->containedLinkOrientationsXLimits[i][0] > spaceInformation->containedLinkOrientationsXLimits[i][1])
            ERROR("linkOrientationsXLimits is malformed");
    }
    if (spaceInformation->containedLinkOrientationsYLimits.size() == 0) {
        for (size_t i = 0; i != spaceInformation->containedLinkOrientationsY.size(); ++i) {
            VectorFloat limits( { -infLimit, infLimit});
            spaceInformation->containedLinkOrientationsYLimits.push_back(limits);
        }
    }
    for (size_t i = 0; i != spaceInformation->containedLinkOrientationsYLimits.size(); ++i) {
        if (spaceInformation->containedLinkOrientationsYLimits[i][0] > spaceInformation->containedLinkOrientationsYLimits[i][1])
            ERROR("linkOrientationsYLimits is malformed");
    }
    if (spaceInformation->containedLinkOrientationsZLimits.size() == 0) {
        for (size_t i = 0; i != spaceInformation->containedLinkOrientationsZ.size(); ++i) {
            VectorFloat limits( { -infLimit, infLimit});
            spaceInformation->containedLinkOrientationsZLimits.push_back(limits);
        }
    }
    for (size_t i = 0; i != spaceInformation->containedLinkOrientationsZLimits.size(); ++i) {
        if (spaceInformation->containedLinkOrientationsZLimits[i][0] > spaceInformation->containedLinkOrientationsZLimits[i][1])
            ERROR("linkOrientationsZLimits is malformed");
    }

    // ######################################################################
    // Link velocities linear
    // ######################################################################
    if (spaceInformation->containedLinkLinearVelocitiesXLimits.size() == 0) {
        for (size_t i = 0; i != spaceInformation->containedLinkLinearVelocitiesX.size(); ++i) {
            VectorFloat limits( { -infLimit, infLimit});
            spaceInformation->containedLinkLinearVelocitiesXLimits.push_back(limits);
        }
    }
    for (size_t i = 0; i != spaceInformation->containedLinkLinearVelocitiesXLimits.size(); ++i) {
        if (spaceInformation->containedLinkLinearVelocitiesXLimits[i][0] >
                spaceInformation->containedLinkLinearVelocitiesXLimits[i][1])
            ERROR("linkLinearVelocitiesXLimits is malformed");
    }
    if (spaceInformation->containedLinkLinearVelocitiesYLimits.size() == 0) {
        for (size_t i = 0; i != spaceInformation->containedLinkLinearVelocitiesY.size(); ++i) {
            VectorFloat limits( { -infLimit, infLimit});
            spaceInformation->containedLinkLinearVelocitiesYLimits.push_back(limits);
        }
    }
    for (size_t i = 0; i != spaceInformation->containedLinkLinearVelocitiesYLimits.size(); ++i) {
        if (spaceInformation->containedLinkLinearVelocitiesYLimits[i][0] >
                spaceInformation->containedLinkLinearVelocitiesYLimits[i][1])
            ERROR("linkLinearVelocitiesYLimits is malformed");
    }
    if (spaceInformation->containedLinkLinearVelocitiesZLimits.size() == 0) {
        for (size_t i = 0; i != spaceInformation->containedLinkLinearVelocitiesZ.size(); ++i) {
            VectorFloat limits( { -infLimit, infLimit});
            spaceInformation->containedLinkLinearVelocitiesZLimits.push_back(limits);
        }
    }
    for (size_t i = 0; i != spaceInformation->containedLinkLinearVelocitiesZLimits.size(); ++i) {
        if (spaceInformation->containedLinkLinearVelocitiesZLimits[i][0] >
                spaceInformation->containedLinkLinearVelocitiesZLimits[i][1])
            ERROR("linkLinearVelocitiesZLimits is malformed");
    }

    if (spaceInformation->containedLinkVelocitiesLinearLimits.size() == 0) {
        for (size_t i = 0; i != spaceInformation->containedLinkVelocitiesLinear.size(); ++i) {
            VectorFloat limits( { -infLimit, infLimit});
            spaceInformation->containedLinkVelocitiesLinearLimits.push_back(limits);
        }
    }
    for (size_t i = 0; i != spaceInformation->containedLinkVelocitiesLinearLimits.size(); ++i) {
        if (spaceInformation->containedLinkVelocitiesLinearLimits[i][0] >
                spaceInformation->containedLinkVelocitiesLinearLimits[i][1])
            ERROR("linkVelocitiesLinearLimits is malformed");
    }

    // ######################################################################
    // Link velocities angular
    // ######################################################################
    if (spaceInformation->containedLinkAngularVelocitiesXLimits.size() == 0) {
        for (size_t i = 0; i != spaceInformation->containedLinkAngularVelocitiesX.size(); ++i) {
            VectorFloat limits( { -infLimit, infLimit});
            spaceInformation->containedLinkAngularVelocitiesXLimits.push_back(limits);
        }
    }
    for (size_t i = 0; i != spaceInformation->containedLinkAngularVelocitiesXLimits.size(); ++i) {
        if (spaceInformation->containedLinkAngularVelocitiesXLimits[i][0] >
                spaceInformation->containedLinkAngularVelocitiesXLimits[i][1])
            ERROR("linkAngularVelocitiesXLimits is malformed");
    }
    if (spaceInformation->containedLinkAngularVelocitiesYLimits.size() == 0) {
        for (size_t i = 0; i != spaceInformation->containedLinkAngularVelocitiesY.size(); ++i) {
            VectorFloat limits( { -infLimit, infLimit});
            spaceInformation->containedLinkAngularVelocitiesYLimits.push_back(limits);
        }
    }
    for (size_t i = 0; i != spaceInformation->containedLinkAngularVelocitiesYLimits.size(); ++i) {
        if (spaceInformation->containedLinkAngularVelocitiesYLimits[i][0] >
                spaceInformation->containedLinkAngularVelocitiesYLimits[i][1])
            ERROR("linkAngularVelocitiesYLimits is malformed");
    }
    if (spaceInformation->containedLinkAngularVelocitiesZLimits.size() == 0) {
        for (size_t i = 0; i != spaceInformation->containedLinkAngularVelocitiesZ.size(); ++i) {
            VectorFloat limits( { -infLimit, infLimit});
            spaceInformation->containedLinkAngularVelocitiesZLimits.push_back(limits);
        }
    }
    for (size_t i = 0; i != spaceInformation->containedLinkAngularVelocitiesZLimits.size(); ++i) {
        if (spaceInformation->containedLinkAngularVelocitiesZLimits[i][0] >
                spaceInformation->containedLinkAngularVelocitiesZLimits[i][1])
            ERROR("linkAngularVelocitiesZLimits is malformed");
    }



    if (spaceInformation->containedLinkVelocitiesAngularLimits.size() == 0) {
        for (size_t i = 0; i != spaceInformation->containedLinkVelocitiesAngular.size(); ++i) {
            VectorFloat limits( { -infLimit, infLimit});
            spaceInformation->containedLinkVelocitiesAngularLimits.push_back(limits);
        }
    }
    for (size_t i = 0; i != spaceInformation->containedLinkVelocitiesAngularLimits.size(); ++i) {
        if (spaceInformation->containedLinkVelocitiesAngularLimits[i][0] >
                spaceInformation->containedLinkVelocitiesAngularLimits[i][1])
            ERROR("linkVelocitiesAngularLimits is malformed");
    }

    if (spaceInformation->lowerUpperLimitsAdditional.size() == 0) {
        for (size_t i = 0; i != spaceInformation->additionalDimensions; ++i) {
            VectorFloat limits( { -infLimit, infLimit});
            spaceInformation->lowerUpperLimitsAdditional.push_back(limits);
        }
    }
    for (size_t i = 0; i != spaceInformation->lowerUpperLimitsAdditional.size(); ++i) {
        if ((spaceInformation->lowerUpperLimitsAdditional[i][0] > spaceInformation->lowerUpperLimitsAdditional[i][1]) ||
                (spaceInformation->lowerUpperLimitsAdditional[i].size() != 2))
            ERROR("additionalDimensionLimits is malformed");
    }
}

const ActionSpaceInformationPtr makeActionSpaceInformation(const RobotConfigOptions* robotConfigOptions,
        const GazeboInterface* gazeboInterface)
{
    VectorString robotJointsInWorld = gazeboInterface->getRobotJointNames();
    VectorString jointsInConfigFile = robotConfigOptions->jointTorques;
    jointsInConfigFile.insert(jointsInConfigFile.end(),
                              robotConfigOptions->jointPositionsAct.begin(),
                              robotConfigOptions->jointPositionsAct.end());
    jointsInConfigFile.insert(jointsInConfigFile.end(),
                              robotConfigOptions->jointVelocitiesAct.begin(),
                              robotConfigOptions->jointVelocitiesAct.end());
    for (auto & joint : jointsInConfigFile) {
        if (!contains(robotJointsInWorld, joint))
            ERROR("joint '" + joint + "' not defined in your robot model");
    }

    ActionSpaceInformationPtr actionSpaceInformation(new ActionSpaceInformation());
    actionSpaceInformation->torqueControlledJoints = robotConfigOptions->jointTorques;
    actionSpaceInformation->velocityControlledJoints = robotConfigOptions->jointVelocitiesAct;
    actionSpaceInformation->positionControlledJoints = robotConfigOptions->jointPositionsAct;
    

    actionSpaceInformation->additionalDimensions = robotConfigOptions->additionalActionDimensions;
    for (auto & lowerUpper : robotConfigOptions->lowerUpperActionLimitsAdditional) {
        if (lowerUpper.size() != 2)
            ERROR("Lower and upper limits must be of size 2");
        std::pair<FloatType, FloatType> lu = {lowerUpper[0], lowerUpper[1]};
        actionSpaceInformation->lowerUpperLimitsAdditional.push_back(lu);
    }

    if (actionSpaceInformation->lowerUpperLimitsAdditional.size() != actionSpaceInformation->additionalDimensions)
        ERROR("Size of limits for additional action dimensions doesn't concede with number of additional dimensions");

    std::vector<VectorString> redundantJoints = robotConfigOptions->redundantJoints;
    VectorString nonRedundantJointTorques;
    VectorString nonRedundantJointVelocities;
    VectorString nonRedundantJointPositions;
    bool breaking;

    for (size_t i = 0; i < actionSpaceInformation->torqueControlledJoints.size(); i++) {
        bool containing = false;
        for (auto & redundantJointList : redundantJoints) {
            if (contains(redundantJointList, actionSpaceInformation->torqueControlledJoints[i])) {
                if (!contains(nonRedundantJointTorques, redundantJointList[0])) {
                    nonRedundantJointTorques.push_back(redundantJointList[0]);
                }
                containing = true;
                break;
            }
        }
        if (!containing) {
            if (!contains(nonRedundantJointTorques, actionSpaceInformation->torqueControlledJoints[i])) {
                nonRedundantJointTorques.push_back(actionSpaceInformation->torqueControlledJoints[i]);
            }
        }
    }

    for (size_t i = 0; i < actionSpaceInformation->velocityControlledJoints.size(); i++) {
        bool containing = false;
        for (auto & redundantJointList : redundantJoints) {
            if (contains(redundantJointList, actionSpaceInformation->velocityControlledJoints[i])) {
                if (!contains(nonRedundantJointVelocities, redundantJointList[0])) {
                    nonRedundantJointVelocities.push_back(redundantJointList[0]);
                }
                containing = true;
                break;
            }
        }
        if (!containing) {
            if (!contains(nonRedundantJointVelocities, actionSpaceInformation->velocityControlledJoints[i])) {
                nonRedundantJointVelocities.push_back(actionSpaceInformation->velocityControlledJoints[i]);
            }
        }
    }

    for (size_t i = 0; i < actionSpaceInformation->positionControlledJoints.size(); i++) {
        bool containing = false;
        for (auto & redundantJointList : redundantJoints) {
            if (contains(redundantJointList, actionSpaceInformation->positionControlledJoints[i])) {
                if (!contains(nonRedundantJointPositions, redundantJointList[0])) {
                    nonRedundantJointPositions.push_back(redundantJointList[0]);
                }
                containing = true;
                break;
            }
        }

        if (!containing) {
            if (!contains(nonRedundantJointPositions, actionSpaceInformation->positionControlledJoints[i])) {
                nonRedundantJointPositions.push_back(actionSpaceInformation->positionControlledJoints[i]);
            }
        }
    }

    actionSpaceInformation->torqueControlledJoints = nonRedundantJointTorques;
    actionSpaceInformation->velocityControlledJoints = nonRedundantJointVelocities;
    actionSpaceInformation->positionControlledJoints = nonRedundantJointPositions;
    actionSpaceInformation->redundantJoints = redundantJoints;
    return actionSpaceInformation;
}

const ObservationSpaceInformationPtr makeObservationSpaceInformation(const RobotConfigOptions* robotConfigOptions,
        const GazeboInterface* gazeboInterface)
{
    VectorString robotJointsInWorld = gazeboInterface->getRobotJointNames();
    VectorString robotLinksInWorld = gazeboInterface->getLinkNames();
    VectorString jointsInConfigFile = robotConfigOptions->jointPositionsObs;
    jointsInConfigFile.insert(jointsInConfigFile.end(),
                              robotConfigOptions->jointVelocitiesObs.begin(),
                              robotConfigOptions->jointVelocitiesObs.end());
    VectorString linksInConfigFile = robotConfigOptions->linkPosesObs;
    linksInConfigFile.insert(linksInConfigFile.end(),
                             robotConfigOptions->linkVelocitiesLinearObs.begin(),
                             robotConfigOptions->linkVelocitiesLinearObs.end());
    linksInConfigFile.insert(linksInConfigFile.end(),
                             robotConfigOptions->linkVelocitiesAngularObs.begin(),
                             robotConfigOptions->linkVelocitiesAngularObs.end());
    for (auto & joint : jointsInConfigFile) {
        if (!contains(robotJointsInWorld, joint))
            ERROR("joint '" + joint + "' not defined in your robot model");
    }

    for (auto & link : linksInConfigFile) {
        if (!contains(robotLinksInWorld, link))
            ERROR("link '" + link + "' not defined in your robot model");
    }

    ObservationSpaceInformationPtr observationSpaceInformation(new ObservationSpaceInformation());
    observationSpaceInformation->jointPositions = robotConfigOptions->jointPositionsObs;
    observationSpaceInformation->jointVelocities = robotConfigOptions->jointVelocitiesObs;
    observationSpaceInformation->containedLinkPoses = robotConfigOptions->linkPosesObs;
    observationSpaceInformation->containedLinkPosesLowerLimits = robotConfigOptions->linkPosesLowerLimitsObs;
    observationSpaceInformation->containedLinkPosesUpperLimits = robotConfigOptions->linkPosesUpperLimitsObs;

    observationSpaceInformation->containedLinkPositionsX = robotConfigOptions->linkPositionsXObs;
    observationSpaceInformation->containedLinkPositionsXLimits = robotConfigOptions->linkPositionsXLimitsObs;
    observationSpaceInformation->containedLinkPositionsY = robotConfigOptions->linkPositionsYObs;
    observationSpaceInformation->containedLinkPositionsYLimits = robotConfigOptions->linkPositionsYLimitsObs;
    observationSpaceInformation->containedLinkPositionsZ = robotConfigOptions->linkPositionsZObs;
    observationSpaceInformation->containedLinkPositionsZLimits = robotConfigOptions->linkPositionsZLimitsObs;

    observationSpaceInformation->containedLinkOrientationsX = robotConfigOptions->linkOrientationsXObs;
    observationSpaceInformation->containedLinkOrientationsXLimits = robotConfigOptions->linkOrientationsXLimitsObs;
    observationSpaceInformation->containedLinkOrientationsY = robotConfigOptions->linkOrientationsYObs;
    observationSpaceInformation->containedLinkOrientationsYLimits = robotConfigOptions->linkOrientationsYLimitsObs;
    observationSpaceInformation->containedLinkOrientationsZ = robotConfigOptions->linkOrientationsZObs;
    observationSpaceInformation->containedLinkOrientationsZLimits = robotConfigOptions->linkOrientationsZLimitsObs;

    observationSpaceInformation->containedLinkLinearVelocitiesX = robotConfigOptions->linkLinearVelocitiesXObs;
    observationSpaceInformation->containedLinkLinearVelocitiesY = robotConfigOptions->linkLinearVelocitiesYObs;
    observationSpaceInformation->containedLinkLinearVelocitiesZ = robotConfigOptions->linkLinearVelocitiesZObs;
    observationSpaceInformation->containedLinkLinearVelocitiesXLimits = robotConfigOptions->linkLinearVelocitiesXLimitsObs;
    observationSpaceInformation->containedLinkLinearVelocitiesYLimits = robotConfigOptions->linkLinearVelocitiesYLimitsObs;
    observationSpaceInformation->containedLinkLinearVelocitiesZLimits = robotConfigOptions->linkLinearVelocitiesZLimitsObs;
    observationSpaceInformation->containedLinkVelocitiesLinear = robotConfigOptions->linkVelocitiesLinearObs;
    observationSpaceInformation->containedLinkVelocitiesLinearLimits = robotConfigOptions->linkVelocitiesLinearLimitsObs;

    observationSpaceInformation->containedLinkAngularVelocitiesX = robotConfigOptions->linkAngularVelocitiesXObs;
    observationSpaceInformation->containedLinkAngularVelocitiesY = robotConfigOptions->linkAngularVelocitiesYObs;
    observationSpaceInformation->containedLinkAngularVelocitiesZ = robotConfigOptions->linkAngularVelocitiesZObs;
    observationSpaceInformation->containedLinkAngularVelocitiesXLimits = robotConfigOptions->linkAngularVelocitiesXLimitsObs;
    observationSpaceInformation->containedLinkAngularVelocitiesYLimits = robotConfigOptions->linkAngularVelocitiesYLimitsObs;
    observationSpaceInformation->containedLinkAngularVelocitiesZLimits = robotConfigOptions->linkAngularVelocitiesZLimitsObs;

    observationSpaceInformation->containedLinkVelocitiesAngular = robotConfigOptions->linkVelocitiesAngular;
    observationSpaceInformation->containedLinkVelocitiesAngularLimits = robotConfigOptions->linkVelocitiesAngularLimits;

    observationSpaceInformation->additionalDimensions = robotConfigOptions->additionalObservationDimensions;    
    observationSpaceInformation->lowerUpperLimitsAdditional = robotConfigOptions->lowerUpperLimitsAdditionalObs;

    std::vector<VectorString> redundantJoints = robotConfigOptions->redundantJoints;
    VectorString nonRedundantJointPositions;
    VectorString nonRedundantJointVelocities;
    bool breaking;

    for (size_t i = 0; i < observationSpaceInformation->jointPositions.size(); i++) {
        bool containing = false;
        for (auto & redundantJointList : redundantJoints) {
            if (contains(redundantJointList, observationSpaceInformation->jointPositions[i])) {
                if (!contains(nonRedundantJointPositions, redundantJointList[0])) {
                    nonRedundantJointPositions.push_back(redundantJointList[0]);
                }
                containing = true;
                break;
            }
        }
        if (!containing) {
            if (!contains(nonRedundantJointPositions, observationSpaceInformation->jointPositions[i])) {
                nonRedundantJointPositions.push_back(observationSpaceInformation->jointPositions[i]);
            }
        }
    }

    for (size_t i = 0; i < observationSpaceInformation->jointVelocities.size(); i++) {
        bool containing = false;
        for (auto & redundantJointList : redundantJoints) {
            if (contains(redundantJointList, observationSpaceInformation->jointVelocities[i])) {
                if (!contains(nonRedundantJointVelocities, redundantJointList[0])) {
                    nonRedundantJointVelocities.push_back(redundantJointList[0]);
                }
                containing = true;
                break;
            }
        }
        if (!containing) {
            if (!contains(nonRedundantJointVelocities, observationSpaceInformation->jointVelocities[i])) {
                nonRedundantJointVelocities.push_back(observationSpaceInformation->jointVelocities[i]);
            }
        }
    }

    observationSpaceInformation->jointPositions = nonRedundantJointPositions;
    observationSpaceInformation->jointVelocities = nonRedundantJointVelocities;
    checkSpaceLimits(observationSpaceInformation.get(), robotConfigOptions, "observationSpace");

    return observationSpaceInformation;
}

}
}
