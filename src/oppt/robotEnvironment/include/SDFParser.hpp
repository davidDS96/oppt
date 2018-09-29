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
#ifndef _SDF_ENVIRONMENT_PARSER_HPP_
#define _SDF_ENVIRONMENT_PARSER_HPP_
#include "oppt/opptCore/core.hpp"
#include <sdf/parser.hh>

namespace oppt
{
class SDFEnvironmentParser
{
public:
    SDFEnvironmentParser();

    VectorObstaclePtr parseObstaclesFromSDFString(const std::string& sdfString) const;

    VectorObstaclePtr parseObstaclesFromFile(std::string& filename, const std::string& robotName) const;

private:
    VectorFloat toDoubleVec(VectorString& stringVec) const;

    Matrixdf processPoseElement(sdf::ElementPtr& poseElement) const;

    VectorFloat processVisualElement(sdf::ElementPtr& visualElement) const;

    ObstacleSharedPtr processLinkElement(std::string& modelName,
                                         sdf::ElementPtr& poseElement,
                                         sdf::ElementPtr& linkElement) const;

    bool processBoxElement(std::string& name,
                           sdf::ElementPtr& boxElement,
                           sdf::ElementPtr& poseElement,
                           sdf::ElementPtr& visualElement,
                           bool& enabled,
                           VectorObstaclePtr& obstacles) const;

    bool processMeshElement(std::string& name,
                            sdf::ElementPtr& meshElement,
                            sdf::ElementPtr& poseElement,
                            sdf::ElementPtr& visualElement,
                            bool& enabled,
                            VectorObstaclePtr& obstacles) const;

    bool processSphereElement(std::string& name,
                              sdf::ElementPtr& sphereElement,
                              sdf::ElementPtr& poseElement,
                              sdf::ElementPtr& visualElement,
                              bool& enabled,
                              VectorObstaclePtr& obstacles) const;

    bool processCylinderElement(std::string& name,
                                sdf::ElementPtr& cylinderElement,
                                sdf::ElementPtr& poseElement,
                                sdf::ElementPtr& visualElement,
                                bool& enabled,
                                VectorObstaclePtr& obstacles) const;

};
}

#endif
