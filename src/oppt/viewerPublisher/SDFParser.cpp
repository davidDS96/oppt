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
#include "SDFParser.hpp"
#include "oppt/opptCore/resources/resources.hpp"

namespace oppt
{
SDFParser::SDFParser():
    EnvironmentParser()
{

}

SDFMarkersSharedPtr SDFParser::parseFromSDFString(std::string& sdfString, const std::string& robotName, std::string environmentFile) const
{
    SDFMarkersSharedPtr environmentMarkers(new SDFMarkers());

    sdf::SDFPtr sdfModel(new sdf::SDF());
    sdf::init(sdfModel);
    sdf::readString(sdfString, sdfModel);
    if (!sdfModel) {
        oppt::ERROR("Could not parse SDF file");
    }

    sdf::ElementPtr rootElement = sdfModel->Root();
    if (!rootElement) {
        ERROR("SDFParser: sdf has no root element");
    }

    sdf::ElementPtr worldElement = rootElement->GetElement("world");
    if (!worldElement) {
        ERROR("SDFParser: sdf has no world element");
    }

    std::string worldName = worldElement->Get<std::string>("name");
    sdf::ElementPtr modelElement = worldElement->GetElement("model");
    if (worldName == "__default__") {
        modelElement = rootElement->GetElement("model");
    }

    if (!modelElement) {
        ERROR("SDFParser: sdf has no model element");
    }

    while (modelElement) {
        std::string modelName = modelElement->Get<std::string>("name");
        if (modelName != robotName) {
            sdf::ElementPtr poseElement = modelElement->GetElement("pose");
            sdf::ElementPtr visualElement =
                modelElement->GetElement("link")->GetElement("visual");
            if (visualElement) {
                sdf::ElementPtr geometryElement = visualElement->GetElement("geometry");
                sdf::ElementPtr geometryTypeElement;
                if (geometryElement) {
                    if (geometryElement->HasElement("mesh")) {
                        geometryTypeElement = geometryElement->GetElement("mesh");
                        processMeshElement(geometryTypeElement,
                                           poseElement,
                                           visualElement,
                                           environmentMarkers,
                                           environmentFile);
                        environmentMarkers->markerNames.push_back(modelName);
                    } else if (geometryElement->HasElement("box")) {
                        geometryTypeElement = geometryElement->GetElement("box");
                        processBoxElement(geometryTypeElement,
                                          poseElement,
                                          visualElement,
                                          environmentMarkers);
                        environmentMarkers->markerNames.push_back(modelName);
                    } else if (geometryElement->HasElement("sphere")) {
                        geometryTypeElement = geometryElement->GetElement("sphere");
                        processSphereElement(geometryTypeElement,
                                             poseElement,
                                             visualElement,
                                             environmentMarkers);
                        environmentMarkers->markerNames.push_back(modelName);
                    } else if (geometryElement->HasElement("cylinder")) {
                        geometryTypeElement = geometryElement->GetElement("cylinder");
                        processCylinderElement(geometryTypeElement,
                                               poseElement,
                                               visualElement,
                                               environmentMarkers);
                    }
                }
            }
        }

        modelElement = modelElement->GetNextElement("model");
    }

    return environmentMarkers;

}

std::string SDFParser::getRobotName(const std::string& robotFile) const
{
    sdf::SDFPtr sdfModel(new sdf::SDF());
    sdf::init(sdfModel);
    sdf::readFile(robotFile, sdfModel);
    std::string robotName;
    if (!sdfModel) {
        robotName = "_dummy_robot_name_";
        return robotName;
    }

    sdf::ElementPtr rootElement = sdfModel->Root();
    if (!rootElement->HasElement("model")) {
        robotName = "_dummy_robot_name_";
        return robotName;
    }

    sdf::ElementPtr modelElem = rootElement->GetElement("model");
    if (!modelElem) {
        ERROR("Root has no model");
    }

    return modelElem->Get<std::string>("name");
}

SDFMarkersSharedPtr SDFParser::parseFromFile(std::string& filename, std::string& robotFile) const
{
    const std::string file = filename;
    std::ifstream ifs(file);
    std::string sdfString((std::istreambuf_iterator<char>(ifs)),
                          (std::istreambuf_iterator<char>()));
    std::string robotName = getRobotName(robotFile);
    return parseFromSDFString(sdfString, robotName, filename);
}

geometry_msgs::Pose SDFParser::processPoseElement(sdf::ElementPtr& poseElement) const
{
    std::string poseStr;
    geometry_msgs::Pose pose;
    if (poseElement) {
        if (poseElement->GetValue()) {
            poseStr = poseElement->GetValue()->GetAsString();
            std::vector<std::string> elems;
            split(poseStr, ' ', elems);
            pose.position.x = atof(elems[0].c_str());
            pose.position.y = atof(elems[1].c_str());
            pose.position.z = atof(elems[2].c_str());
            Quaternionf quat = math::eulerAnglesToQuaternion(atof(elems[3].c_str()),
                               atof(elems[4].c_str()),
                               atof(elems[5].c_str()));
            pose.orientation.x = quat.x();
            pose.orientation.y = quat.y();
            pose.orientation.z = quat.z();
            pose.orientation.w = quat.w();
        } else {
            oppt::WARNING("Pose of SDF element is malformed!");
        }
    }

    return pose;
}

bool SDFParser::processMaterialElement(std::shared_ptr<visualization_msgs::Marker>& marker,
                                       sdf::ElementPtr& visualElement) const
{
    marker->color.a = 1.0;
    marker->color.r = 0.1;
    marker->color.g = 0.1;
    marker->color.b = 0.1;
    sdf::ElementPtr materialElement = visualElement->GetElement("material");
    if (materialElement) {
        if (materialElement->HasElement("ambient")) {
            sdf::ElementPtr ambientElement = materialElement->GetElement("ambient");
            std::string colorString = ambientElement->GetValue()->GetAsString().c_str();
            VectorString colorVec;
            split(colorString, ' ', colorVec);
            marker->color.r = atof(colorVec[0].c_str());
            marker->color.g = atof(colorVec[1].c_str());
            marker->color.b = atof(colorVec[2].c_str());
            marker->color.a = atof(colorVec[3].c_str());
        }
    }

    return true;
}

SDFMarkersSharedPtr SDFParser::textToMarkers(const std::string& text) const
{
    SDFMarkersSharedPtr textMarkers(new SDFMarkers());
    MarkerSharedPtr marker = std::make_shared<visualization_msgs::Marker>();
    ////////////////////////////////////////
    geometry_msgs::Pose markerPose;
    markerPose.position.x = 0;
    markerPose.position.y = 0;
    markerPose.position.z = 0;

    markerPose.orientation.x = 0;
    markerPose.orientation.y = 0;
    markerPose.orientation.z = 0;
    markerPose.orientation.w = 1;

    marker->pose = markerPose;
    marker->id = 0;
    marker->type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker->action = visualization_msgs::Marker::ADD;
    marker->text = text;

    marker->scale.x = 1;
    marker->scale.y = 1;
    marker->scale.z = 1;

    marker->color.r = 1;
    marker->color.g = 1;
    marker->color.b = 1;
    marker->color.a = 1;
    ////////////////////////////////////////
    textMarkers->markerNames.push_back("textMarker");
    textMarkers->markers.push_back(marker);
    return textMarkers;
}

MarkerSharedPtr SDFParser::sphereToMarker(GeometrySharedPtr& sphereGeometry) const
{
    const geometric::Sphere* sphere = static_cast<const geometric::Sphere*>(sphereGeometry.get());
    MarkerSharedPtr marker =
        std::make_shared<visualization_msgs::Marker>();
    FloatType radius = sphere->getRadius();
    Matrixdf pose = sphere->getPose();
    VectorFloat color = sphere->getColor();
    Matrix3f rotationMatrix = pose.block<3, 3>(0, 0);
    Quaternionf quat = math::rotationMatrixToQuaternion(rotationMatrix);

    geometry_msgs::Pose markerPose;
    markerPose.position.x = pose(0, 3);
    markerPose.position.y = pose(1, 3);
    markerPose.position.z = pose(2, 3);

    markerPose.orientation.x = quat.x();
    markerPose.orientation.y = quat.y();
    markerPose.orientation.z = quat.z();
    markerPose.orientation.w = quat.w();

    marker->pose = markerPose;
    marker->id = 0;
    marker->type = visualization_msgs::Marker::SPHERE;
    marker->action = visualization_msgs::Marker::ADD;
    marker->scale.x = 2.0 * radius;
    marker->scale.y = 2.0 * radius;
    marker->scale.z = 2.0 * radius;
    marker->color.r = color[0];
    marker->color.g = color[1];
    marker->color.b = color[2];
    marker->color.a = color[3];
    return marker;
}

MarkerSharedPtr SDFParser::cylinderToMarker(GeometrySharedPtr& cylinderGeometry) const
{
    const geometric::Cylinder* cylinder = static_cast<const geometric::Cylinder*>(cylinderGeometry.get());
    MarkerSharedPtr marker =
        std::make_shared<visualization_msgs::Marker>();
    FloatType radius = cylinder->getRadius();
    FloatType length = cylinder->getLength();
    Matrixdf pose = cylinder->getPose();
    VectorFloat color = cylinder->getColor();
    Matrix3f rotationMatrix = pose.block<3, 3>(0, 0);
    Quaternionf quat = math::rotationMatrixToQuaternion(rotationMatrix);

    geometry_msgs::Pose markerPose;
    markerPose.position.x = pose(0, 3);
    markerPose.position.y = pose(1, 3);
    markerPose.position.z = pose(2, 3);

    markerPose.orientation.x = quat.x();
    markerPose.orientation.y = quat.y();
    markerPose.orientation.z = quat.z();
    markerPose.orientation.w = quat.w();

    marker->pose = markerPose;
    marker->id = 0;
    marker->type = visualization_msgs::Marker::CYLINDER;
    marker->action = visualization_msgs::Marker::ADD;
    marker->scale.x = 2.0 * radius;
    marker->scale.y = 2.0 * radius;
    marker->scale.z = length;
    marker->color.r = color[0];
    marker->color.g = color[1];
    marker->color.b = color[2];
    marker->color.a = color[3];
    return marker;
}

std::vector<MarkerSharedPtr> SDFParser::geometryToFrameMarkers(GeometrySharedPtr &geometry) const {
    MarkerSharedPtr markerX =
        std::make_shared<visualization_msgs::Marker>();
    MarkerSharedPtr markerY =
        std::make_shared<visualization_msgs::Marker>();
    MarkerSharedPtr markerZ =
        std::make_shared<visualization_msgs::Marker>();
    Matrixdf pose = geometry->getPose();    
    Matrix3f rotationMatrix = pose.block<3, 3>(0, 0);
    Quaternionf quat = math::rotationMatrixToQuaternion(rotationMatrix);

    Matrixdf xAxis = Matrixdf::Identity(4, 4);
    Matrixdf yAxis = Matrixdf::Identity(4, 4);
    Matrixdf zAxis = Matrixdf::Identity(4, 4);
    
    xAxis(0, 3) = 1.0;
    yAxis(0, 3) = 1.0;
    zAxis(0, 3) = 1.0;

    Matrixdf preRotationY = Matrixdf::Identity(4, 4);
    Matrixdf preRotationZ = Matrixdf::Identity(4, 4);
    preRotationZ << cos(-M_PI / 2.0), 0.0, sin(-M_PI / 2.0), 0.0,
                    0.0, 1.0, 0.0, 0.0,
                    -sin(-M_PI/2.0), 0.0, cos(-M_PI / 2.0), 0.0,
                    0.0, 0.0, 0.0, 1.0;
    preRotationY << cos(M_PI / 2.0), -sin(M_PI / 2.0), 0.0, 0.0,
                    sin(M_PI / 2.0), cos(M_PI / 2.0), 0.0, 0.0,
                    0.0, 0.0, 1.0, 0.0,
                    0.0, 0.0, 0.0, 1.0;

    Matrixdf xAxisTransformed = pose * xAxis;
    Matrixdf yAxisTransformed = pose * preRotationY * yAxis;
    Matrixdf zAxisTransformed = pose * preRotationZ * zAxis;

    cout << "name: " << geometry->getName() << endl;
    cout << "pose: " << endl << pose << endl;
    cout << "xAxisTransformed: " << endl << xAxisTransformed << endl;
    cout << "yAxisTransformed: " << endl << yAxisTransformed << endl;
    cout << "zAxisTransformed: " << endl << zAxisTransformed << endl;
    //getchar();

    Quaternionf xAxisRotation = math::rotationMatrixToQuaternion(xAxisTransformed.block<3, 3>(0, 0));
    Quaternionf yAxisRotation = math::rotationMatrixToQuaternion(yAxisTransformed.block<3, 3>(0, 0));
    Quaternionf zAxisRotation = math::rotationMatrixToQuaternion(zAxisTransformed.block<3, 3>(0, 0));

    geometry_msgs::Pose markerXPose, markerYPose, markerZPose;


    markerXPose.position.x = pose(0, 3);
    markerXPose.position.y = pose(1, 3);
    markerXPose.position.z = pose(2, 3);

    markerYPose.position.x = pose(0, 3);
    markerYPose.position.y = pose(1, 3);
    markerYPose.position.z = pose(2, 3);

    markerZPose.position.x = pose(0, 3);
    markerZPose.position.y = pose(1, 3);
    markerZPose.position.z = pose(2, 3);

    markerXPose.orientation.x = xAxisRotation.x();
    markerXPose.orientation.y = xAxisRotation.y();
    markerXPose.orientation.z = xAxisRotation.z();
    markerXPose.orientation.w = xAxisRotation.w();

    markerYPose.orientation.x = yAxisRotation.x();
    markerYPose.orientation.y = yAxisRotation.y();
    markerYPose.orientation.z = yAxisRotation.z();
    markerYPose.orientation.w = yAxisRotation.w();

    markerZPose.orientation.x = zAxisRotation.x();
    markerZPose.orientation.y = zAxisRotation.y();
    markerZPose.orientation.z = zAxisRotation.z();
    markerZPose.orientation.w = zAxisRotation.w();

    markerX->pose = markerXPose;
    markerX->id = 0;
    markerX->type =visualization_msgs::Marker::ARROW;
    markerX->action = visualization_msgs::Marker::ADD;
    markerX->scale.x = 0.25;
    markerX->scale.y = 0.01;
    markerX->scale.z = 0.01;
    markerX->color.r = 1.0;
    markerX->color.g = 0.0;
    markerX->color.b = 0.0;
    markerX->color.a = 1.0;

    markerY->pose = markerYPose;
    markerY->id = 0;
    markerY->type =visualization_msgs::Marker::ARROW;
    markerY->action = visualization_msgs::Marker::ADD;
    markerY->scale.x = 0.25;
    markerY->scale.y = 0.01;
    markerY->scale.z = 0.01;
    markerY->color.r = 0.0;
    markerY->color.g = 1.0;
    markerY->color.b = 0.0;
    markerY->color.a = 1.0;

    markerZ->pose = markerZPose;
    markerZ->id = 0;
    markerZ->type =visualization_msgs::Marker::ARROW;
    markerZ->action = visualization_msgs::Marker::ADD;
    markerZ->scale.x = 0.25;
    markerZ->scale.y = 0.01;
    markerZ->scale.z = 0.01;
    markerZ->color.r = 0.0;
    markerZ->color.g = 0.0;
    markerZ->color.b = 1.0;
    markerZ->color.a = 1.0;


    return std::vector<MarkerSharedPtr>({markerX, markerY, markerZ});
    //return std::vector<MarkerSharedPtr>();

}

MarkerSharedPtr SDFParser::boxToMarker(GeometrySharedPtr& boxGeometry) const
{
    const geometric::Box* box = static_cast<const geometric::Box*>(boxGeometry.get());
    MarkerSharedPtr marker =
        std::make_shared<visualization_msgs::Marker>();
    Matrixdf pose = box->getPose();
    VectorFloat dimensions = box->getDimensions();
    VectorFloat color = box->getColor();
    Matrix3f rotationMatrix = pose.block<3, 3>(0, 0);
    Quaternionf quat = math::rotationMatrixToQuaternion(rotationMatrix);

    geometry_msgs::Pose markerPose;
    markerPose.position.x = pose(0, 3);
    markerPose.position.y = pose(1, 3);
    markerPose.position.z = pose(2, 3);

    markerPose.orientation.x = quat.x();
    markerPose.orientation.y = quat.y();
    markerPose.orientation.z = quat.z();
    markerPose.orientation.w = quat.w();

    marker->pose = markerPose;
    marker->id = 0;
    marker->type = visualization_msgs::Marker::CUBE;
    marker->action = visualization_msgs::Marker::ADD;
    marker->scale.x = dimensions[0];
    marker->scale.y = dimensions[1];
    marker->scale.z = dimensions[2];
    marker->color.r = color[0];
    marker->color.g = color[1];
    marker->color.b = color[2];
    marker->color.a = color[3];
    return marker;
}

MarkerSharedPtr SDFParser::meshToMarker(GeometrySharedPtr& meshGeometry) const
{
    const geometric::Mesh* mesh = static_cast<const geometric::Mesh*>(meshGeometry.get());
    MarkerSharedPtr marker =
        std::make_shared<visualization_msgs::Marker>();
    Matrixdf pose = mesh->getPose();
    VectorFloat color = mesh->getColor();
    std::string meshUri = mesh->getMeshUri();
    meshUri = oppt::resources::FindFile(meshUri);
    VectorFloat scale = mesh->getScale();
    Matrix3f rotationMatrix = pose.block<3, 3>(0, 0);
    Quaternionf quat = math::rotationMatrixToQuaternion(rotationMatrix);

    geometry_msgs::Pose markerPose;
    markerPose.position.x = pose(0, 3);
    markerPose.position.y = pose(1, 3);
    markerPose.position.z = pose(2, 3);

    markerPose.orientation.x = quat.x();
    markerPose.orientation.y = quat.y();
    markerPose.orientation.z = quat.z();
    markerPose.orientation.w = quat.w();

    marker->pose = markerPose;
    marker->id = 0;
    marker->type = visualization_msgs::Marker::MESH_RESOURCE;
    marker->action = visualization_msgs::Marker::ADD;
    marker->scale.x = scale[0];
    marker->scale.y = scale[1];
    marker->scale.z = scale[2];
    marker->mesh_resource = "file://" + meshUri;
    marker->color.r = color[0];
    marker->color.g = color[1];
    marker->color.b = color[2];
    marker->color.a = color[3];
    marker->mesh_use_embedded_materials = true;
    return marker;
}

bool SDFParser::processSphereElement(sdf::ElementPtr& sphereElement,
                                     sdf::ElementPtr& poseElement,
                                     sdf::ElementPtr& visualElement,
                                     SDFMarkersSharedPtr& environmentMarkers) const
{
    sdf::ElementPtr radiusElement = sphereElement->GetElement("radius");
    if (!radiusElement) {
        oppt::WARNING("Sphere has no radius element");
        return false;
    }

    FloatType radius = atof(radiusElement->GetValue()->GetAsString().c_str());
    std::shared_ptr<visualization_msgs::Marker> marker =
        std::make_shared<visualization_msgs::Marker>();
    geometry_msgs::Pose pose = processPoseElement(poseElement);
    marker->pose = pose;
    marker->id = getMarkerId(environmentMarkers);
    marker->type = visualization_msgs::Marker::SPHERE;
    marker->action = visualization_msgs::Marker::ADD;
    marker->scale.x = 2 * radius;
    marker->scale.y = 2 * radius;
    marker->scale.z = 2 * radius;
    processMaterialElement(marker, visualElement);
    environmentMarkers->markers.push_back(marker);
    return true;

}

bool SDFParser::processCylinderElement(sdf::ElementPtr& cylinderElement,
                                       sdf::ElementPtr& poseElement,
                                       sdf::ElementPtr& visualElement,
                                       SDFMarkersSharedPtr& environmentMarkers) const
{
    sdf::ElementPtr radiusElement = cylinderElement->GetElement("radius");
    sdf::ElementPtr lengthElement = cylinderElement->GetElement("length");
    if (!radiusElement)
        oppt::WARNING("Cylinder has no radius element");
    if (!lengthElement)
        oppt::WARNING("Cylinder has no length element");
    FloatType radius = atof(radiusElement->GetValue()->GetAsString().c_str());
    FloatType length = atof(lengthElement->GetValue()->GetAsString().c_str());
    std::shared_ptr<visualization_msgs::Marker> marker =
        std::make_shared<visualization_msgs::Marker>();
    geometry_msgs::Pose pose = processPoseElement(poseElement);
    marker->pose = pose;
    marker->id = getMarkerId(environmentMarkers);
    marker->type = visualization_msgs::Marker::CYLINDER;
    marker->action = visualization_msgs::Marker::ADD;
    marker->scale.x = 2 * radius;
    marker->scale.y = 2 * radius;
    marker->scale.z = length;
    processMaterialElement(marker, visualElement);
    environmentMarkers->markers.push_back(marker);
    return true;
}

MarkerSharedPtr SDFParser::geometryToMarker(GeometrySharedPtr& geometry, const Matrixdf& pose, const VectorFloat& color) const
{
    MarkerSharedPtr marker;
    std::string name;
    bool validGeometry = true;
    switch (geometry->getType()) {
    case geometric::BOX:
        marker = boxToMarker(geometry);
        name = geometry->getName();
        break;
    case geometric::SPHERE:
        marker = sphereToMarker(geometry);
        name = geometry->getName();
        break;
    case geometric::CYLINDER:
        marker = cylinderToMarker(geometry);
        name = geometry->getName();
        break;
    case geometric::MESH:
        marker = meshToMarker(geometry);
        name = geometry->getName();
        break;
    default:
        WARNING("SDFParser: geometriesToMarkers: geometry type not recognized: " + std::to_string(geometry->getType()));
        validGeometry = false;
        break;
    }

    // Set the given pose;
    Matrix3f rotationMatrix = pose.block<3, 3>(0, 0);
    Quaternionf quat = math::rotationMatrixToQuaternion(rotationMatrix);

    geometry_msgs::Pose markerPose;
    markerPose.position.x = pose(0, 3);
    markerPose.position.y = pose(1, 3);
    markerPose.position.z = pose(2, 3);

    markerPose.orientation.x = quat.x();
    markerPose.orientation.y = quat.y();
    markerPose.orientation.z = quat.z();
    markerPose.orientation.w = quat.w();
    marker->pose = markerPose;

    marker->color.r = color[0];
    marker->color.g = color[1];
    marker->color.b = color[2];
    marker->color.a = color[3];

    return marker;
}

SDFMarkersSharedPtr SDFParser::geometriesToMarkers(VectorGeometryPtr& geometries) const
{
    SDFMarkersSharedPtr environmentMarkers(new SDFMarkers());
    for (auto & geometry : geometries) {
        MarkerSharedPtr marker;
        std::string name;
        bool validGeometry = true;
        switch (geometry->getType()) {
        case geometric::BOX:
            marker = boxToMarker(geometry);
            name = geometry->getName();
            break;
        case geometric::SPHERE:
            marker = sphereToMarker(geometry);
            name = geometry->getName();
            break;
        case geometric::CYLINDER:
            marker = cylinderToMarker(geometry);
            name = geometry->getName();
            break;
        case geometric::MESH:
            marker = meshToMarker(geometry);
            name = geometry->getName();
            break;
        default:
            WARNING("SDFParser: geometriesToMarkers: geometry type not recognized: " + std::to_string(geometry->getType()));
            validGeometry = false;
            break;
        }        

        if (validGeometry) {
            std::vector<MarkerSharedPtr> frameMarkers;// = geometryToFrameMarkers(geometry);
            environmentMarkers->markerNames.push_back(name);
            environmentMarkers->markers.push_back(marker);
            for (size_t i = 0; i != frameMarkers.size(); ++i) {
                environmentMarkers->markerNames.push_back(name + "_frame_" + std::to_string(i));
                environmentMarkers->markers.push_back(frameMarkers[i]);
            }            
        }
    }

    return environmentMarkers;
}

bool SDFParser::processBoxElement(sdf::ElementPtr& boxElement,
                                  sdf::ElementPtr& poseElement,
                                  sdf::ElementPtr& visualElement,
                                  SDFMarkersSharedPtr& environmentMarkers) const
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

    std::shared_ptr<visualization_msgs::Marker> marker =
        std::make_shared<visualization_msgs::Marker>();
    geometry_msgs::Pose pose = processPoseElement(poseElement);
    marker->pose = pose;
    marker->id = getMarkerId(environmentMarkers);
    marker->type = visualization_msgs::Marker::CUBE;
    marker->action = visualization_msgs::Marker::ADD;
    marker->scale.x = 1.0;
    marker->scale.y = 1.0;
    marker->scale.z = 1.0;
    if (sizeElems.size() > 0) {
        marker->scale.x = atof(sizeElems[0].c_str());
        marker->scale.y = atof(sizeElems[1].c_str());
        marker->scale.z = atof(sizeElems[2].c_str());
    }

    processMaterialElement(marker, visualElement);
    environmentMarkers->markers.push_back(marker);
    return true;
}

bool SDFParser::processMeshElement(sdf::ElementPtr& meshElement,
                                   sdf::ElementPtr& poseElement,
                                   sdf::ElementPtr& visualElement,
                                   SDFMarkersSharedPtr& environmentMarkers,
                                   std::string& environmentFile) const
{
    if (meshElement->HasAttribute("scale")) {

    }

    std::string meshUri = meshElement->GetElement("uri")->GetValue()->GetAsString();
    meshUri = oppt::resources::FindFile(meshUri);
    std::string scaleStr = meshElement->GetElement("scale")->GetValue()->GetAsString();
    std::vector<std::string> scaleVecElemes;
    split(scaleStr, ' ', scaleVecElemes);
    std::shared_ptr<visualization_msgs::Marker> marker =
        std::make_shared<visualization_msgs::Marker>();
    geometry_msgs::Pose pose = processPoseElement(poseElement);
    marker->pose = pose;

    // Process scale here

    marker->id = getMarkerId(environmentMarkers);
    marker->type = visualization_msgs::Marker::MESH_RESOURCE;
    marker->action = visualization_msgs::Marker::ADD;
    marker->scale.x = 1.0;
    marker->scale.y = 1.0;
    marker->scale.z = 1.0;
    if (scaleVecElemes.size() > 0) {
        marker->scale.x = atof(scaleVecElemes[0].c_str());
        marker->scale.y = atof(scaleVecElemes[1].c_str());
        marker->scale.z = atof(scaleVecElemes[2].c_str());
    }

    marker->mesh_resource = "file://" + meshUri;
    marker->mesh_use_embedded_materials = true;
    processMaterialElement(marker, visualElement);
    environmentMarkers->markers.push_back(marker);
    return true;
}

int SDFParser::getMarkerId(SDFMarkersSharedPtr& environmentMarkers) const
{
    int markerId = 0;
    if (environmentMarkers->markers.size() > 0) {
        markerId = environmentMarkers->markers[environmentMarkers->markers.size() - 1]->id + 1;
    }

    return markerId;
}

}
