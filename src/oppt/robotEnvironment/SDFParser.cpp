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
#include "include/SDFParser.hpp"
#include "oppt/opptCore/resources/resources.hpp"
#include "include/BoxObstacle.hpp"
#include "include/MeshObstacle.hpp"
#include "include/SphereObstacle.hpp"
#include "include/CylinderObstacle.hpp"

namespace oppt
{

SDFEnvironmentParser::SDFEnvironmentParser()
{

}

VectorObstaclePtr SDFEnvironmentParser::parseObstaclesFromSDFString(const std::string& sdfString) const
{
    std::string sdfStr(sdfString);
    if (sdfStr.find("<sdf") == std::string::npos) {
        sdfStr = "<sdf version='1.6'>" + sdfStr + "</sdf>";
    }

    sdf::SDFPtr sdfModel(new sdf::SDF());
    sdf::init(sdfModel);
    sdf::readString(sdfStr, sdfModel);
    if (!sdfModel) {
        ERROR("SDFParser: Could not parse SDF string");
    }

    sdf::ElementPtr rootElement = sdfModel->Root();
    if (!rootElement) {
        ERROR("SDFParser: Invalid SDF string");
    }

    sdf::ElementPtr modelElement = rootElement->GetElement("model");
    if (!modelElement) {
        ERROR("SDFParser: Invalid SDF string");
    }

    std::string modelName = modelElement->Get<std::string>("name");
    sdf::ElementPtr poseElement = modelElement->GetElement("pose");

    VectorObstaclePtr obstacles;
    sdf::ElementPtr linkElement = modelElement->GetElement("link");
    while (linkElement) {
        auto linkName = linkElement->Get<std::string>("name");
        std::string obstName = modelName + "_nestLink_" + linkName;
        auto obstacle = processLinkElement(obstName, poseElement, linkElement);
        if (obstacle)
            obstacles.push_back(obstacle);
        linkElement = linkElement->GetNextElement("link");
    }

    return obstacles;
}

ObstacleSharedPtr SDFEnvironmentParser::processLinkElement(std::string& modelName,
        sdf::ElementPtr& poseElement,
        sdf::ElementPtr& linkElement) const
{
    VectorObstaclePtr obstacles;
    sdf::ElementPtr collisionElement =
        linkElement->GetElement("collision");
    sdf::ElementPtr visualElement =
        linkElement->GetElement("visual");
    auto name = linkElement->Get<std::string>("name");
    bool enabled = true;
    sdf::ElementPtr geometryElement;
    std::string collisionName = collisionElement->Get<std::string>("name");
    if (collisionName == "__default__") {
        enabled = false;
        std::string visualName = visualElement->Get<std::string>("name");
        if (visualName == "__default__") {
	    std::string msg = "SDFParser: Obstacle ";
	    msg += name;
	    msg += " has no collision or visual element";
            ERROR(msg);
        }

        geometryElement = visualElement->GetElement("geometry");
    } else {
        geometryElement = collisionElement->GetElement("geometry");
    }

    sdf::ElementPtr geometryTypeElement;
    if (!geometryElement) {
        ERROR(modelName + " has no geometryElement");
    }

    if (geometryElement->HasElement("mesh")) {
        geometryTypeElement = geometryElement->GetElement("mesh");
        processMeshElement(modelName,
                           geometryTypeElement,
                           poseElement,
                           visualElement,
                           enabled,
                           obstacles);
    } else if (geometryElement->HasElement("box")) {
        geometryTypeElement = geometryElement->GetElement("box");
        processBoxElement(modelName,
                          geometryTypeElement,
                          poseElement,
                          visualElement,
                          enabled,
                          obstacles);
    } else if (geometryElement->HasElement("sphere")) {
        geometryTypeElement = geometryElement->GetElement("sphere");
        processSphereElement(modelName,
                             geometryTypeElement,
                             poseElement,
                             visualElement,
                             enabled,
                             obstacles);
    } else if (geometryElement->HasElement("cylinder")) {
        geometryTypeElement = geometryElement->GetElement("cylinder");
        processCylinderElement(modelName,
                               geometryTypeElement,
                               poseElement,
                               visualElement,
                               enabled,
                               obstacles);
    } else {
        WARNING("SDFParser: Couldn't parse SDF string. It either has an unsupported geometry type, or is not a model");
        return nullptr;
    }

    return obstacles[0];
}

VectorObstaclePtr SDFEnvironmentParser::parseObstaclesFromFile(std::string& filename,
        const std::string& robotName) const
{
    const std::string file = filename;
    sdf::SDFPtr sdfModel(new sdf::SDF());
    sdf::init(sdfModel);
    sdf::readFile(file, sdfModel);
    if (!sdfModel) {
        oppt::ERROR("Could not parse SDF file");
    }

    VectorObstaclePtr obstacles;
    sdf::ElementPtr rootElement = sdfModel->Root();
    if (rootElement) {
        sdf::ElementPtr worldElement = rootElement->GetElement("world");
        if (worldElement) {
            sdf::ElementPtr modelElement = worldElement->GetElement("model");
            while (modelElement) {
                std::string modelName = modelElement->Get<std::string>("name");
                if (modelName != robotName) {
                    sdf::ElementPtr poseElement = modelElement->GetElement("pose");
                    sdf::ElementPtr linkElement = modelElement->GetElement("link");
                    while (linkElement) {
                        ObstacleSharedPtr obstacle = processLinkElement(modelName, poseElement, linkElement);
                        if (obstacle)
                            obstacles.push_back(obstacle);
                        linkElement = linkElement->GetNextElement("link");
                    }
                }

                modelElement = modelElement->GetNextElement("model");
            }

        }
    }

    return obstacles;
}

VectorFloat SDFEnvironmentParser::toDoubleVec(VectorString& stringVec) const
{
    VectorFloat FloatTypeVec;
    for (size_t i = 0; i < stringVec.size(); i++) {
        FloatTypeVec.push_back(atof(stringVec[i].c_str()));
    }

    return FloatTypeVec;
}

Matrixdf SDFEnvironmentParser::processPoseElement(sdf::ElementPtr& poseElement) const
{
    if (poseElement) {
        if (poseElement->GetValue()) {
            std::string poseStr = poseElement->GetValue()->GetAsString();
            std::vector<std::string> elems;
            split(poseStr, ' ', elems);
            VectorFloat poseVec = toDoubleVec(elems);
            Matrix4f transMatrix = math::getTranslationMatrix(poseVec[0], poseVec[1], poseVec[2]);
	    Matrix3f rotMatrix = math::eulerAnglesToRotationMatrix(poseVec[3], poseVec[4], poseVec[5]);
	    Matrix4f rotMatrix2 = Matrix4f::Identity(4, 4);
	    rotMatrix2.block(0, 0, 3, 3) = rotMatrix;
	    Matrix4f result = transMatrix * rotMatrix2;
	    return result;
	    
	    
	    
        } else {
            oppt::WARNING("Pose of SDF element is malformed!");
        }
    }

    return Matrixdf::Identity(4, 4);
}

VectorFloat SDFEnvironmentParser::processVisualElement(sdf::ElementPtr& visualElement) const
{
    VectorFloat color( {0.5, 0.5, 0.5, 1.0});
    if (visualElement->HasElement("material")) {
        sdf::ElementPtr materialElem = visualElement->GetElement("material");
        if (materialElem->HasElement("ambient")) {
            sdf::ElementPtr ambientElem = materialElem->GetElement("ambient");
            std::string colorString = ambientElem->GetValue()->GetAsString();
            VectorString pieces;
            split(colorString, ' ', pieces);
            color[0] = atof(pieces[0].c_str());
            color[1] = atof(pieces[1].c_str());
            color[2] = atof(pieces[2].c_str());
            color[3] = atof(pieces[3].c_str());
        }
    }

    return color;
}

bool SDFEnvironmentParser::processMeshElement(std::string& name,
        sdf::ElementPtr& meshElement,
        sdf::ElementPtr& poseElement,
        sdf::ElementPtr& visualElement,
        bool& enabled,
        VectorObstaclePtr& obstacles) const
{
    if (meshElement->HasAttribute("scale")) {

    }

    std::string meshUri = meshElement->GetElement("uri")->GetValue()->GetAsString();
    std::string scaleStr = meshElement->GetElement("scale")->GetValue()->GetAsString();
    std::vector<std::string> scaleVecElemes;
    split(scaleStr, ' ', scaleVecElemes);
    VectorFloat scale( {1.0, 1.0, 1.0});
    if (scaleVecElemes.size() == 3) {
        scale[0] = atof(scaleVecElemes[0].c_str());
        scale[1] = atof(scaleVecElemes[1].c_str());
        scale[2] = atof(scaleVecElemes[2].c_str());
    }

    VectorString meshURIs( {meshUri});
    for (auto & meshUri : meshURIs) {
        if (!oppt::resources::FileExists(meshUri))
            ERROR("File '" + meshUri + "' doesn't exist");
    }
    Matrixdf pose = processPoseElement(poseElement);
    std::string translationString =
        std::to_string(pose(0, 3)) + " " + std::to_string(pose(1, 3)) + " " + std::to_string(pose(2, 3));
    std::string rotationString = "";
    for (size_t i = 0; i < 3; i++) {
        for (size_t j = 0; j < 3; j++) {
            rotationString += std::to_string(pose(i, j));
            if (i != 2 && j != 2) {
                rotationString += " ";
            }
        }
    }
     
    VectorFloat color = processVisualElement(visualElement);    
    bool fromFile = false;
    ObstacleSharedPtr obstacle(new oppt::MeshObstacle(name, meshURIs, pose, scale, fromFile));
    obstacle->setEnabled(enabled);
    obstacle->setColor(color);
    obstacles.push_back(obstacle);
    return true;
}

bool SDFEnvironmentParser::processSphereElement(std::string& name,
        sdf::ElementPtr& sphereElement,
        sdf::ElementPtr& poseElement,
        sdf::ElementPtr& visualElement,
        bool& enabled,
        VectorObstaclePtr& obstacles) const
{
    sdf::ElementPtr radiusElement = sphereElement->GetElement("radius");
    if (!radiusElement) {
        oppt::WARNING("Sphere has no radius element");
        return false;
    }

    FloatType radius = atof(radiusElement->GetValue()->GetAsString().c_str());
    Matrixdf pose = processPoseElement(poseElement);    
    VectorFloat color = processVisualElement(visualElement);    
    ObstacleSharedPtr obstacle(new SphereObstacle(name, pose, radius));
    obstacle->setEnabled(enabled);
    obstacle->setColor(color);
    obstacles.push_back(obstacle);
    return true;
}

bool SDFEnvironmentParser::processCylinderElement(std::string& name,
        sdf::ElementPtr& cylinderElement,
        sdf::ElementPtr& poseElement,
        sdf::ElementPtr& visualElement,
        bool& enabled,
        VectorObstaclePtr& obstacles) const
{
    sdf::ElementPtr radiusElement = cylinderElement->GetElement("radius");
    sdf::ElementPtr lengthElement = cylinderElement->GetElement("length");
    if (!radiusElement) {
        oppt::WARNING("Cylinder has no radius element");
        return false;
    }

    if (!lengthElement) {
        oppt::WARNING("Cylinder has no length element");
        return false;
    }

    FloatType radius = atof(radiusElement->GetValue()->GetAsString().c_str());
    FloatType length = atof(lengthElement->GetValue()->GetAsString().c_str());
    Matrixdf pose = processPoseElement(poseElement);    
    VectorFloat color = processVisualElement(visualElement);    
    ObstacleSharedPtr obstacle(new CylinderObstacle(name, pose, radius, length));
    obstacle->setEnabled(enabled);
    obstacle->setColor(color);
    obstacles.push_back(obstacle);
    return true;
}

bool SDFEnvironmentParser::processBoxElement(std::string& name,
        sdf::ElementPtr& boxElement,
        sdf::ElementPtr& poseElement,
        sdf::ElementPtr& visualElement,
        bool& enabled,
        VectorObstaclePtr& obstacles) const
{
    sdf::ElementPtr sizeElement = boxElement->GetElement("size");
    if (!sizeElement) {
        oppt::WARNING("Box has no size element");
        return false;
    }

    std::string sizeStr = sizeElement->GetValue()->GetAsString();
    VectorString sizeElems;
    split(sizeStr, ' ', sizeElems);
    if (sizeElems.size() != 3) {
        oppt::WARNING("Size of box has incorrect number of values");
        return false;
    }

    FloatType sizeX = atof(sizeElems[0].c_str());
    FloatType sizeY = atof(sizeElems[1].c_str());
    FloatType sizeZ = atof(sizeElems[2].c_str());
    VectorFloat dimensions( {sizeX, sizeY, sizeZ});
    Matrixdf pose = processPoseElement(poseElement);    
    VectorFloat color = processVisualElement(visualElement);   
    ObstacleSharedPtr obstacle(new oppt::BoxObstacle(name,
                               pose,
                               dimensions));
    obstacle->setEnabled(enabled);
    obstacle->setColor(color);
    obstacles.push_back(obstacle);
    return true;
}

}
