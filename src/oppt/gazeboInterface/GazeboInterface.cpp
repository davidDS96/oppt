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
#include "oppt/gazeboInterface/GazeboInterface.hpp"
#include <gazebo/gazebo.hh>
#include <boost/timer.hpp>
#include <time.h>

#include <sys/wait.h>
#include <unistd.h>
#include <signal.h>
#include <sys/types.h>
#include <cstdlib>
#include "oppt/opptCore/resources/resources.hpp"
#include "gazebo/OpptODEPhysics.hpp"
#include "ServerInterface.hpp"
#include "GazeboSubscriber.hpp"
#include <gazebo/physics/ode/ODEJoint.hh>


gazebo::Server* server;
boost::mutex serverMutex;

namespace oppt
{

std::vector<WorldPtr> allWorlds;

static boost::mutex odeMtx;

void onODEContact(const std::string& h)
{

}

GazeboInterface::GazeboInterface(std::string& gazeboWorldFile,
                                 std::string& modelFile,
                                 std::string& prefix,
                                 unsigned int threadId) :
    server_(nullptr),
    worldMap_(),
    robots_(),
    mainWorldName_(""),
    mainRobotName_(""),
    initialWorldStates_(),
    lastConstructedWorldStates_(),
    initialWorldModelMap_(),
    worldJointMap_(),
    worldRobotLinkMap_(),
    worldLinkMap_(),
    contactSubs_(),
    stateSpaceInformation_(nullptr),
    actionSpaceInformation_(nullptr),
    observationSpaceInformation_(nullptr),
    stateSpaceDimension_(0),
    addEntityConnection_(nullptr),
    addObstacleCallback_(nullptr),
    obstacleEventQueue_(new ObstacleEventQueue()),
    processObstaclesEventsThread_(nullptr),
    finishProcessingObstacles_(false),
    threadId_(threadId),
    sensorInterface_(new SensorInterface()),
    setStateFunctions_(),
    getStateFunctions_(),
    applyActionFunctions_(),
    getObservationFunctions_(),
    collisionCheckedLinks_(),
    collisionFunction_(nullptr),
    deferredPoseChanges_(),
    prefix_(prefix),
    saveUserData_(true),
    rootJoints_()
{
    checkSDFValidity(gazeboWorldFile, modelFile);
    if (!gazebo::physics::PhysicsFactory::IsRegistered("ode2")) {
        RegisterOpptODEPhysics();
    }
    mainWorldName_ = prefix + "_" + getWorldName(gazeboWorldFile, threadId_);
    mainRobotName_ = getRobotNameFromSDF(modelFile);
    initWorldFromFile(worldMap_, gazeboWorldFile, threadId_, mainWorldName_);
    for (auto & worldElem : worldMap_) {
        ModelPtr robotModel = findRobotModel(worldElem.second, mainRobotName_);
        initRobotModel(worldElem.first,
                       robotModel,
                       worldJointMap_,
                       worldRobotLinkMap_,
                       sensorInterface_);
        initWorldLinkMap(worldElem.first, worldLinkMap_);
    }

    for (auto & worldEntry : worldMap_) {
        std::string topicStr = "/gazebo/" + worldEntry.first + "/physics/contacts";
        gazebo::transport::NodePtr node(new gazebo::transport::Node());
        node->Init();
        contactSubs_[worldEntry.first] = node->Subscribe(topicStr, onODEContact);
        initialCumulativeAngles_ =
            static_cast<gazebo::physics::OpptODEPhysics*>(worldEntry.second->GetPhysicsEngine().get())->getCumulativeAngles();
        initialWorldStates_[worldEntry.first] = makeWorldState();
        lastConstructedWorldStates_[worldEntry.first] = initialWorldStates_[worldEntry.first];
    }

    collisionFunction_ = [this](void * const data) {
        auto world = worldMap_[mainWorldName_];
        world->GetPhysicsEngine()->UpdateCollision();
        bool collided = false;
        if (world->GetPhysicsEngine()->GetContactManager()->GetContactCount() > 0) {
            for (auto & contact : world->GetPhysicsEngine()->GetContactManager()->GetContacts()) {
                std::string collisionLink1 = getUnscopedName(contact->collision1->GetLink()->GetName());
                std::string collisionLink2 = getUnscopedName(contact->collision2->GetLink()->GetName());
                std::unique_ptr<ContactPoint> contactPoint(new ContactPoint());
                contactPoint->contactBody1 = collisionLink1;
                contactPoint->contactBody2 = collisionLink2;
                gazebo::math::Vector3 position = contact->positions[0];
                contactPoint->contactPointWorldPosition = VectorFloat( {position.x, position.y, position.z});
                static_cast<ContactInformation* const>(data)->contactPoints.push_back(std::move(contactPoint));
                if (contains(collisionCheckedLinks_, collisionLink1) || contains(collisionCheckedLinks_, collisionLink2)) {
                    collided = true;
                }
            }
        }

        oppt::CollisionReportSharedPtr collisionReport(new CollisionReport());
        collisionReport->collides = collided;

        return collisionReport;
    };

    lastSimTime_ = worldMap_[mainWorldName_]->GetSimTime();
    if (prefix_ == "exec") {
        gazeboSubscriber_ = std::unique_ptr<GazeboSubscriber> (new GazeboSubscriber(this, mainWorldName_));
    }
    makeInitialWorldModelMap();
    initRootJoints();
}

GazeboInterface::~GazeboInterface()
{
    gazeboSubscriber_.reset();
    finishProcessingObstacles_ = true;
    if (processObstaclesEventsThread_) {
        processObstaclesEventsThread_->join();
        delete processObstaclesEventsThread_;
    }

    //boost::mutex::scoped_lock scoped_lock(serverMutex);
    if (threadId_ == 0 && server) {
        gazebo::physics::stop_worlds();
        gazebo::physics::remove_worlds();
        while (gazebo::physics::worlds_running()) {
            usleep(1.0 * 1e6);
            LOGGING("Some worlds still running");
        }
        server->Fini();
        if (server) {
            //delete server;
        }
    }

    delete sensorInterface_;
    //LOGGING("Destroyed gazebo_interface");
}

void GazeboInterface::initRootJoints()
{
    for (auto & jointEntry : worldJointMap_.at(mainWorldName_)) {
        auto joint = jointEntry.second;
        gazebo::physics::LinkPtr parentLink = joint->GetParent();
        if (!parentLink) {
            rootJoints_[joint->GetName()] = joint;
        }
    }
}

std::string GazeboInterface::getWorldName(std::string& gazeboWorldFile,
        const unsigned int& threadId)
{
    sdf::SDFPtr sdfModel(new sdf::SDF());
    sdf::init(sdfModel);
    sdf::readFile(gazeboWorldFile, sdfModel);
    sdf::ElementPtr rootElement = sdfModel->Root();
    sdf::ElementPtr worldElement = rootElement->GetElement("world");
    std::string worldName = worldElement->Get<std::string> ("name");

    worldName += "_" + std::to_string(threadId);
    return worldName;
}

void GazeboInterface::initWorldFromFile(WorldMap& worldMap,
                                        const std::string& gazeboWorldFile,
                                        const unsigned int& threadId,
                                        std::string worldName)
{
    try {
        unsigned int inc = 0;
        if (!server) {
            server = new gazebo::Server();
            bool loaded = false;
            while (!loaded) {
                try {
                    LOGGING("Initializing gazebo server " + std::to_string(inc));
                    setupServer(0, NULL, inc);
                    //server->PreLoad();
                    loaded = true;
                } catch (gazebo::common::Exception& e) {
                    // If we can't setup a server with the given port number,
                    // we increase the port number and try again
                    cout << "Gazebo exception: " << e << endl;
                    inc++;
                    if (inc == 1000) {
                        ERROR("Couldn't initilize gzserver after trying 1000 different ports");
                    }
                }
            }
        }

        WorldPtr world = loadWorld(gazeboWorldFile, worldName);
        //world->GetPhysicsEngine()->InitForThread();
        static_cast<gazebo::physics::OpptODEPhysics*>(world->GetPhysicsEngine().get())->makeJointNameMap();
        std::string wn = world->GetName();
        worldMap[wn] = world;
        gazebo::sensors::run_once(true);
        //gazebo::physics::run_worlds();
        //gazebo::physics::pause_worlds(true);
    } catch (gazebo::common::Exception& e) {
        std::string msg = e.GetErrorStr();
        ERROR(msg);
    }
}

WorldPtr GazeboInterface::loadWorld(const std::string& gazeboWorldFile,
                                    std::string worldName)
{
    gazebo::physics::WorldPtr world;

    // Load the world file
    sdf::SDFPtr sdf(new sdf::SDF);
    if (!sdf::init(sdf)) {
        gzerr << "Unable to initialize sdf\n";
        ERROR("GazeboInterface: loadWorld: Unable to initialize sdf\n");
    }

    sdf::readFile(gazeboWorldFile, sdf);
    world = gazebo::physics::create_world();
    allWorlds.push_back(world);
    sdf::ElementPtr worldElement = sdf->Root()->GetElement("world");

    // Make sure that only ODE is allowed as physics engine
    // Change it to ode2
    sdf::ElementPtr physicsElement = worldElement->GetElement("physics");
    if (!physicsElement) {
        ERROR("SDF has no physics element?!");
    }
    sdf::ParamPtr physicsTypeParam = physicsElement->GetAttribute("type");
    if (!physicsTypeParam) {
        ERROR("No physics type param?!");
    }
    std::string physicsTypeStr = physicsTypeParam->GetAsString();
    if (physicsTypeStr != "ode" && physicsTypeStr != "ode2") {
        ERROR("You have selected an unsupported physics engine in your SDF file. Currently only ODE is supported");
    }
    physicsTypeParam->Set<std::string> ("ode2");
    sdf::ParamPtr nameParam = worldElement->GetAttribute("name");
    nameParam->Set<std::string> (worldName);
    gazebo::physics::load_world(world, worldElement);
    gazebo::physics::init_world(world);
    world->SetPaused(true);
    world->GetPhysicsEngine()->UpdateCollision();
    return world;
}

void GazeboInterface::initWorldLinkMap(const std::string& worldName, WorldLinkMap& worldLinkMap)
{
    worldLinkMap[worldName] = std::unordered_map<std::string, gazebo::physics::LinkPtr>();
    auto world = worldMap_.at(worldName);
    auto models = world->GetModels();
    for (auto & model : models) {
        auto links = model->GetLinks();
        for (auto & link : links) {
            std::string scopedName = link->GetName();
            std::string unscopedName = scopedName;
            if (scopedName.find("::") != std::string::npos) {
                VectorString nameElems;
                split(scopedName, "::", nameElems);
                unscopedName = nameElems[nameElems.size() - 1];
            }

            worldLinkMap[worldName][unscopedName] = link;
        }
    }
}

std::vector<LinkPtr> GazeboInterface::getLinks() const
{
    std::vector<LinkPtr> links(worldLinkMap_.at(mainWorldName_).size());
    size_t c = 0;
    for (auto & linkEntry : worldLinkMap_.at(mainWorldName_)) {
        links[c] = linkEntry.second;
        c++;
    }

    return links;
}

std::vector<JointPtr> GazeboInterface::getJoints() const
{
    std::vector<JointPtr> joints;
    auto world = worldMap_.at(mainWorldName_);
    auto models = world->GetModels();
    for (auto & model : models) {
        auto modelJoints = model->GetJoints();
        for (auto & modelJoint : modelJoints) {
            joints.push_back(modelJoint);
        }
    }

    return joints;
}

void GazeboInterface::initRobotModel(const std::string& worldName,
                                     ModelPtr& model,
                                     WorldJointMap& worldJointMap,
                                     WorldLinkMap& worldRobotLinkMap,
                                     SensorInterface* sensorInterface)
{
    //dBodySetMovedCallback(0, 0);
    sensorInterface->init(worldName);

    worldJointMap[worldName] = std::unordered_map<std::string, gazebo::physics::JointPtr>();
    worldRobotLinkMap[worldName] = std::unordered_map<std::string, gazebo::physics::LinkPtr>();

    if (model) {
        std::vector<gazebo::physics::JointPtr> jointsTemp = model->GetJoints();
        for (size_t i = 0; i < jointsTemp.size(); i++) {
            std::string scopedName = jointsTemp[i]->GetName();
            std::string unscopedName = scopedName;
            if (scopedName.find("::") != std::string::npos) {
                VectorString nameElems;
                split(scopedName, "::", nameElems);
                unscopedName = nameElems[nameElems.size() - 1];
            }

            worldJointMap[worldName][unscopedName] = jointsTemp[i];
        }

        for (auto & link : model->GetLinks()) {
            std::string scopedName = link->GetName();
            std::string unscopedName = scopedName;
            if (scopedName.find("::") != std::string::npos) {
                VectorString nameElems;
                split(scopedName, "::", nameElems);
                unscopedName = nameElems[nameElems.size() - 1];
            }

            worldRobotLinkMap[worldName][unscopedName] = link;
        }
    }
}

std::string GazeboInterface::getRobotNameFromSDF(const std::string& robotModelFile) const
{
    sdf::SDFPtr sdfModel(new sdf::SDF());
    sdf::init(sdfModel);
    sdf::readFile(robotModelFile, sdfModel);
    sdf::ElementPtr rootElement = sdfModel->Root();
    sdf::ElementPtr modelElement = rootElement->GetElement("model");
    if (!modelElement) {
        ERROR("GazeboInterfacel: SDF model file has no model element");
    }

    return modelElement->Get<std::string> ("name");
}

void GazeboInterface::checkSDFValidity(const std::string& worldFile, const std::string& modelFile)
{
    std::string robotName = getRobotNameFromSDF(modelFile);
    sdf::SDFPtr sdfWorldModel(new sdf::SDF());
    sdf::init(sdfWorldModel);
    sdf::readFile(worldFile, sdfWorldModel);
    sdf::ElementPtr rootElement = sdfWorldModel->Root();
    sdf::ElementPtr worldElement = rootElement->GetElement("world");
    sdf::ElementPtr modelElement = worldElement->GetElement("model");
    bool robotInWorld = false;
    while (modelElement) {
        if (modelElement->HasAttribute("name")) {
            auto modelName = modelElement->Get<std::string> ("name");
            if (modelName == robotName) {
                robotInWorld = true;
                auto linkElem = modelElement->GetElement("link");
                auto nameAttr = linkElem->GetAttribute("name");
                auto linkNameStr = nameAttr->GetAsString();
                VectorString elems;
                split(linkNameStr, "::", elems);
                if (elems.size() > 0) {
                    if (elems[0] != robotName) {
                        ERROR("Robot name missmatch in your environment SDF file. Make sure that the robot name and the included URI match");
                    }
                }
            }
        }
        modelElement = modelElement->GetNextElement();
    }

    if (!robotInWorld) {
        ERROR("The robot model provided by '" + modelFile + "' is not referenced in your environment file");
    }
}

void GazeboInterface::setInteractive(const bool& interactive)
{
    if (interactive && !worldRunning_ && prefix_ == "exec") {
        auto world = worldMap_[mainWorldName_];
        world->Run();
        world->SetPaused(true);
        processObstaclesEventsThread_ = new boost::thread(&GazeboInterface::processObstacleEvents,
                this,
                obstacleEventQueue_);
        std::string poseTopicStr = "/gazebo/" + mainWorldName_ + "/pose/info";
        gazebo::transport::NodePtr node(new gazebo::transport::Node());
        node->Init();
        worldRunning_ = true;
    } else {
        processObstaclesEventsThread_ = NULL;
    }
}

void GazeboInterface::setDirtyCollisionFunction(DirtyCollisionFunction dirtyCollisionFunction)
{
    collisionFunction_ = dirtyCollisionFunction;
}

void GazeboInterface::removeObstaclesFromWorld(WorldPtr& world)
{
    std::string modelName;
    size_t currentModelSize = world->GetModels().size();
    size_t numModelsToDelete = 0;
    for (auto & model : world->GetModels()) {
        modelName = model->GetName();
        if (modelName != mainRobotName_) {
            numModelsToDelete += 1;
            gazebo::transport::requestNoReply(world->GetName(), "entity_delete", modelName);
        }
    }
    while (world->GetModels().size() > (currentModelSize - numModelsToDelete)) {
        usleep(0.001 * 1e6);
    }
}

LinkPtr GazeboInterface::findRootLink(const ModelPtr& model)
{
    std::vector<JointPtr> joints = model->GetJoints();
    VectorString parents;
    VectorString children;
    for (auto & joint : joints) {
        parents.push_back(joint->GetParent()->GetName());
        children.push_back(joint->GetChild()->GetName());
    }

    for (auto & parent : parents) {
        bool found = false;
        for (auto & child : children) {
            if (parent == child) {
                found = true;
                break;
            }
        }

        if (!found) {
            return model->GetLink(parent);
        }
    }

    return nullptr;
}

void GazeboInterface::processObstacleEvents(std::shared_ptr<ObstacleEventQueue>& obstacleEventQueue)
{
    auto world = worldMap_[mainWorldName_];
    while (true) {
        usleep(0.005 * 1e6);
        boost::mutex::scoped_lock scoped_lock(mtx_);
        if (obstacleEventQueue->size() > 0) {
            auto obstacleEvent = obstacleEventQueue->front();
            std::string obstacleScopedName = obstacleEvent.second;
            VectorString obstacleNameElems;
            split(obstacleScopedName, "::", obstacleNameElems);
            std::string worldName = obstacleNameElems[0];
            std::string obstacleName = obstacleNameElems[obstacleNameElems.size() - 1];
            if (obstacleEvent.first == "add") {
                // The name of the world in which the change occured
                std::vector<ModelPtr> models = world->GetModels();
                for (auto & model : models) {
                    if (model->GetName() == obstacleName) {
                        // For now we set the model to static by removing the current
                        // model instance and replacing it with a static version
                        std::string modelName = model->GetName();
                        std::vector<ModelPtr> currentModelsT = world->GetModels();
                        sdf::ElementPtr modelElem = model->GetSDF();
                        std::string currentName = modelElem->Get<std::string> ("name");
                        sdf::ElementPtr staticElement = modelElem->GetElement("static");
                        staticElement->Set(true);
                        std::string sdfString = modelElem->ToString("");
                        gazeboSubscriber_->suppressProcessing(true);

                        // Temporarily disconnect event
                        gazebo::event::Events::DisconnectAddEntity(addEntityConnection_);
                        addObstacleFromSDFString(sdfString);
                        addEntityConnection_ =
                            gazebo::event::Events::ConnectAddEntity(std::bind(&GazeboInterface::onAddObstacle,
                                    this,
                                    std::placeholders::_1));
                        addObstacleCallback_(sdfString);
                        gazeboSubscriber_->suppressProcessing(false);
                    }
                }
            }

            obstacleEventQueue->pop();
        }
        if (finishProcessingObstacles_) {
            break;
        }
    }

}

void GazeboInterface::onAddObstacle(const std::string& str)
{
    bool contains = false;
    auto models = worldMap_[mainWorldName_]->GetModels();
    for (auto & model : models) {
        if (model->GetScopedName() == str) {
            contains = true;
            break;
        }
    }
    if (!contains && mainWorldName_.find("exec") != std::string::npos) {
        boost::mutex::scoped_lock scoped_lock(mtx_);
        std::string nonConstStr(str);
        auto pair = std::make_pair("add", nonConstStr);
        obstacleEventQueue_->push(pair);
    }
}

void GazeboInterface::onRemoveObstacle(const std::string& str)
{
    LOGGING("ON REMOVE OBSTACLE " + str + " " + prefix_);
    bool contains = false;
    auto models = worldMap_[mainWorldName_]->GetModels();
    for (auto & model : models) {
        if (model->GetScopedName() == str) {
            contains = true;
            break;
        }
    }
    if (mainWorldName_.find("exec") != std::string::npos) {
        boost::mutex::scoped_lock scoped_lock(mtx_);
        std::string nonConstStr(str);
        auto pair = std::make_pair("remove", nonConstStr);
        obstacleEventQueue_->push(pair);
    }
}

void GazeboInterface::registerOnAddObstacleCallback(std::function<void (const std::string&) >& callback)
{
    addObstacleCallback_ = callback;
    addEntityConnection_ =
        gazebo::event::Events::ConnectAddEntity(std::bind(&GazeboInterface::onAddObstacle,
                this,
                std::placeholders::_1));
}

void GazeboInterface::registerOnRemoveObstacleCallback(std::function<void (const std::string&) >& callback)
{
    gazeboSubscriber_->setRemoveObstacleCallback(callback);
}

void GazeboInterface::registerOnPoseChangedCallback(std::function<void (const std::string&, const VectorFloat&) >& callback)
{
    gazeboSubscriber_->setObstaclePoseChangedCallback(callback);
}

ModelPtr GazeboInterface::findRobotModel(WorldPtr& world, std::string& robotName)
{
    std::vector<ModelPtr> models = world->GetModels();
    ModelPtr model;
    for (auto & m : models) {
        model = findRobotModelImpl(m, robotName);
        if (model) {
            return model;
        }
    }

    return model;
}

ModelPtr GazeboInterface::findRobotModelImpl(const ModelPtr& model, std::string& robotName)
{
    if (model->GetName() == robotName) {
        return model;
    }

    const std::vector<ModelPtr> nestedModels = model->NestedModels();
    ModelPtr mm;
    for (auto & nestedModel : nestedModels) {
        mm = findRobotModelImpl(nestedModel, robotName);
        if (mm) {
            return mm;
        }
    }

    return mm;

}

void GazeboInterface::reset()
{
    // Temporarily disconnect event
    if (prefix_ == "exec") {
        gazeboSubscriber_->suppressProcessing(true);
        gazebo::event::Events::DisconnectAddEntity(addEntityConnection_);
    }

    // Remove models that have been added during runtime
    for (auto model : worldMap_[mainWorldName_]->GetModels()) {
        if (model->GetName() == mainRobotName_) {
            continue;
        }
        bool found = false;
        for (auto & entry : initialWorldModelMap_) {
            if (entry.first == model->GetName()) {
                found = true;
                break;
            }
        }

        if (!found) {
            removeObstacle(model->GetName());
        }

    }

    // Add models that have been removed during runtime
    for (auto & entry : initialWorldModelMap_) {
        bool found = false;
        for (auto model : worldMap_[mainWorldName_]->GetModels()) {
            if (model->GetName() == mainRobotName_) {
                continue;
            }
            if (entry.first == model->GetName()) {
                found = true;
                break;
            }
        }

        if (!found) {
            addObstacleFromSDFString(entry.second);
        }
    }

    // Set the initial world state
    deferredPoseChanges_.clear();
    setWorldState(initialWorldStates_[mainWorldName_], true);
    if (prefix_ == "exec") {
        addEntityConnection_ =
            gazebo::event::Events::ConnectAddEntity(std::bind(&GazeboInterface::onAddObstacle,
                    this,
                    std::placeholders::_1));
        gazeboSubscriber_->suppressProcessing(false);
    }
}

void GazeboInterface::makeInitialWorldModelMap()
{
    WorldPtr world = worldMap_[mainWorldName_];
    world->Reset();
    world->ResetPhysicsStates();
    auto models = world->GetModels();
    for (auto & model : models) {
        if (model->GetName() != mainRobotName_) {
            auto sdfElement = model->GetSDF();
            const std::string sdfString = sdfElement->ToString("");
            initialWorldModelMap_[model->GetName()] = sdfString;
        }
    }
}

void GazeboInterface::setObservationSpaceDimension(const unsigned int& dimension)
{
    observationSpaceDimension_ = dimension;
}

void GazeboInterface::makeInitialWorldState(const VectorFloat& initialStateVec, const std::string &worldName, const bool& fullReset)
{
    VectorString worldNames( {mainWorldName_});
    for (auto & worldName : worldNames) {
        WorldPtr world = worldMap_[worldName];
        if (!world) {
            WARNING("world '" + worldName + "' is null");
            return;
        }

        if (fullReset) {
            world->Reset();
            world->ResetPhysicsStates();
            world->ResetTime();
        }

        static_cast<gazebo::physics::OpptODEPhysics*>(world->GetPhysicsEngine().get())->setCumulativeAngles(initialCumulativeAngles_);
        for (auto & setStateFunction : setStateFunctions_) {
            setStateFunction(initialStateVec, worldName);
        }

        initialWorldStates_[worldName] =
            makeWorldState();
        lastConstructedWorldStates_[worldName] = initialWorldStates_.at(worldName);
    }
}

const VectorString GazeboInterface::getJointNames() const
{
    // TODO: Return all the joint names, not just the robot joints
    VectorString jointNames;
    for (auto const & entry : worldJointMap_.at(mainWorldName_)) {
        jointNames.push_back(entry.first);
    }

    return jointNames;
}

const VectorString GazeboInterface::getRobotJointNames() const
{
    VectorString jointNames;
    for (auto const & entry : worldJointMap_.at(mainWorldName_)) {
        jointNames.push_back(entry.first);
    }

    return jointNames;
}

const VectorString GazeboInterface::getRobotLinkNames() const
{
    VectorString linkNames;
    for (auto const & entry : worldRobotLinkMap_.at(mainWorldName_)) {
        linkNames.push_back(entry.first);
    }

    return linkNames;
}

const VectorString GazeboInterface::getLinkNames() const
{
    VectorString linkNames;
    WorldPtr world = worldMap_.at(mainWorldName_);
    auto models = world->GetModels();
    for (auto & model : models) {
        auto links = model->GetLinks();
        for (auto & link : links) {
            std::string scopedName = link->GetName();
            std::string unscopedName = scopedName;
            if (scopedName.find("::") != std::string::npos) {
                VectorString nameElems;
                split(scopedName, "::", nameElems);
                unscopedName = nameElems[1];
            }
            linkNames.push_back(unscopedName);
        }
    }

    return linkNames;
}

GazeboWorldStatePtr GazeboInterface::getInitialWorldState(std::string worldName)
{
    if (worldName == "") {
        worldName = mainWorldName_;
    }

    return initialWorldStates_[worldName];
}

void GazeboInterface::addObstacleFromSDFString(std::string& sdfString)
{
    if (sdfString.find("<sdf version") == std::string::npos)
        sdfString = "<sdf version='1.6'>" + sdfString + "</sdf>";
    sdf::SDFPtr sdfModel(new sdf::SDF());
    sdf::init(sdfModel);
    sdf::readString(sdfString, sdfModel);
    if (!sdfModel) {
        oppt::ERROR("Could not parse SDF file");
    }

    WorldPtr world = worldMap_[mainWorldName_];
    sdf::ElementPtr rootElement = sdfModel->Root();
    sdf::ElementPtr modelElement = rootElement->GetElement("model");
    if (modelElement) {
        std::string modelName = modelElement->Get<std::string> ("name");

        // Remove existing obstacle with same name
        removeObstacle(modelName);
        usleep(200000);
        unsigned int numModelsBeforeInsert = world->GetModelCount();
        world->InsertModelString(sdfString);
        fakeStep();
        bool modelInserted = false;
        std::vector<ModelPtr> currentModels;
        unsigned int currentModelCount;
        while (!modelInserted) {
            currentModelCount = world->GetModelCount();
            auto models = world->GetModels();
            if (currentModelCount == numModelsBeforeInsert + 1) {
                modelInserted = true;
            }

            usleep(1e5);
        }

    } else {
        ERROR("String has no modelElement!");
    }
}

void GazeboInterface::fakeStep()
{
    if (mainWorldName_.find("exec") == std::string::npos) {
        auto world = worldMap_[mainWorldName_];
        //world->ProcessMessages();
        bool physicsEnabled = world->GetEnablePhysicsEngine();
        auto physicsEngine =
            static_cast<gazebo::physics::OpptODEPhysics*>(world->GetPhysicsEngine().get());
        bool collisionsBlocked = physicsEngine->collionCheckBlocked();
        physicsEngine->blockCollisionCheck(true);
        //physicsEngine->enableJoints(false);
        world->EnablePhysicsEngine(false);
        std::map<std::string, bool> implicitSpringDamperUsed;

        auto jointMap = worldJointMap_.at(mainWorldName_);
        for (auto & nameJointPair : jointMap) {
            implicitSpringDamperUsed[nameJointPair.first] =
                static_cast<gazebo::physics::ODEJoint*>(nameJointPair.second.get())->UsesImplicitSpringDamper();
            static_cast<gazebo::physics::ODEJoint*>(nameJointPair.second.get())->UseImplicitSpringDamper(false);

        }

        world->Run(1);
        usleep(100000);
        world->Stop();

        for (auto & nameJointPair : jointMap) {
            static_cast<gazebo::physics::ODEJoint*>(nameJointPair.second.get())->UseImplicitSpringDamper(implicitSpringDamperUsed[nameJointPair.first]);
        }
        world->EnablePhysicsEngine(physicsEnabled);
        //physicsEngine->enableJoints(true);
        physicsEngine->blockCollisionCheck(collisionsBlocked);
    }
}

void GazeboInterface::removeObstacle(std::string name)
{
    auto world = worldMap_[mainWorldName_];
    unsigned int currentModelSize = 0;
    std::vector<ModelPtr> currentModels = world->GetModels();
    bool modelFound = false;
    for (auto & model : currentModels) {
        if (model->GetName() == name) {
            modelFound = true;
            currentModelSize = world->GetModelCount();
            LOGGING("Remove model: " + mainWorldName_ + ": " + name);
            world->RemoveModel(name);
            fakeStep();
            break;
        }
    }

    //std::vector<ModelPtr> newModels = world->GetModels();
    if (modelFound) {
        while (currentModelSize == world->GetModelCount()) {
            usleep(1e2);
        }
    }
}

void GazeboInterface::changeModelPose(const std::string& name, const VectorFloat& poseVec)
{
    auto world = worldMap_[mainWorldName_];
    std::vector<ModelPtr> currentModels = world->GetModels();
    for (auto & model : currentModels) {
        if (model->GetName() == name) {
            auto p = std::pair<const std::string, const VectorFloat> {name, poseVec};
            deferredPoseChanges_.push_back(p);
            break;
        }
    }
}

void GazeboInterface::applyDeferredPoseChanges()
{
    if (deferredPoseChanges_.size() > 0) {
        auto world = worldMap_[mainWorldName_];
        std::vector<ModelPtr> currentModels = world->GetModels();
        for (auto & poseChange : deferredPoseChanges_) {
            for (auto & model : currentModels) {
                if (model->GetName() == poseChange.first) {
                    gazebo::math::Vector3 newPosition(poseChange.second[0], poseChange.second[1], poseChange.second[2]);
                    gazebo::math::Quaternion newOrientation(poseChange.second[6], poseChange.second[3], poseChange.second[4], poseChange.second[5]);
                    gazebo::math::Pose newPose(newPosition, newOrientation);
                    model->SetWorldPose(newPose);
                    break;
                }
            }
        }

        deferredPoseChanges_.clear();
    }
}

void GazeboInterface::setSoftLimitThreshold(const FloatType& softLimitThreshold)
{
    auto jointMap = worldJointMap_.at(mainWorldName_);
    for (auto & nameJointPair : jointMap) {
        FloatType lowerSoftLimitPosition =
            nameJointPair.second->GetLowerLimit(0).Radian() + std::fabs(nameJointPair.second->GetLowerLimit(0).Radian()) * softLimitThreshold;

        FloatType upperSoftLimitPosition =
            nameJointPair.second->GetUpperLimit(0).Radian() - std::fabs(nameJointPair.second->GetUpperLimit(0).Radian()) * softLimitThreshold;

        FloatType softLimitVelocity =
            nameJointPair.second->GetVelocityLimit(0) - std::fabs(nameJointPair.second->GetVelocityLimit(0)) * softLimitThreshold;

        nameJointPair.second->SetLowerLimit(0, lowerSoftLimitPosition);
        nameJointPair.second->SetUpperLimit(0, upperSoftLimitPosition);
        nameJointPair.second->SetVelocityLimit(0, softLimitVelocity);
    }
}

void GazeboInterface::getStateLimits(VectorFloat& lowerStateLimits, VectorFloat& upperStateLimits)
{
    lowerStateLimits.clear();
    upperStateLimits.clear();
    auto jointMap = worldJointMap_.at(mainWorldName_);
    for (auto & jointName : stateSpaceInformation_->jointPositions) {
        for (auto & nameJointPair : jointMap) {
            if (nameJointPair.first.find(jointName) != std::string::npos) {
                lowerStateLimits.push_back(nameJointPair.second->GetLowerLimit(0).Radian());
                upperStateLimits.push_back(nameJointPair.second->GetUpperLimit(0).Radian());
            }
        }
    }

    for (auto & jointName : stateSpaceInformation_->jointVelocities) {
        for (auto & nameJointPair : jointMap) {
            if (nameJointPair.first.find(jointName) != std::string::npos) {
                lowerStateLimits.push_back(-nameJointPair.second->GetVelocityLimit(0));
                upperStateLimits.push_back(nameJointPair.second->GetVelocityLimit(0));
            }
        }
    }

    for (size_t i = 0; i < stateSpaceInformation_->containedLinkPoses.size(); i++) {
        for (size_t j = 0; j < 6; j++) {
            lowerStateLimits.push_back(stateSpaceInformation_->containedLinkPosesLowerLimits[i][j]);
            upperStateLimits.push_back(stateSpaceInformation_->containedLinkPosesUpperLimits[i][j]);
        }
    }

    for (size_t i = 0; i != stateSpaceInformation_->containedLinkPositionsX.size(); ++i) {
        lowerStateLimits.push_back(stateSpaceInformation_->containedLinkPositionsXLimits[i][0]);
        upperStateLimits.push_back(stateSpaceInformation_->containedLinkPositionsXLimits[i][1]);
    }

    for (size_t i = 0; i != stateSpaceInformation_->containedLinkPositionsY.size(); ++i) {
        lowerStateLimits.push_back(stateSpaceInformation_->containedLinkPositionsYLimits[i][0]);
        upperStateLimits.push_back(stateSpaceInformation_->containedLinkPositionsYLimits[i][1]);
    }

    for (size_t i = 0; i != stateSpaceInformation_->containedLinkPositionsZ.size(); ++i) {
        lowerStateLimits.push_back(stateSpaceInformation_->containedLinkPositionsZLimits[i][0]);
        upperStateLimits.push_back(stateSpaceInformation_->containedLinkPositionsZLimits[i][1]);
    }

    for (size_t i = 0; i != stateSpaceInformation_->containedLinkOrientationsX.size(); ++i) {
        lowerStateLimits.push_back(stateSpaceInformation_->containedLinkOrientationsXLimits[i][0]);
        upperStateLimits.push_back(stateSpaceInformation_->containedLinkOrientationsXLimits[i][1]);
    }

    for (size_t i = 0; i != stateSpaceInformation_->containedLinkOrientationsY.size(); ++i) {
        lowerStateLimits.push_back(stateSpaceInformation_->containedLinkOrientationsYLimits[i][0]);
        upperStateLimits.push_back(stateSpaceInformation_->containedLinkOrientationsYLimits[i][1]);
    }

    for (size_t i = 0; i != stateSpaceInformation_->containedLinkOrientationsZ.size(); ++i) {
        lowerStateLimits.push_back(stateSpaceInformation_->containedLinkOrientationsZLimits[i][0]);
        upperStateLimits.push_back(stateSpaceInformation_->containedLinkOrientationsZLimits[i][1]);
    }

    for (size_t i = 0; i != stateSpaceInformation_->containedLinkLinearVelocitiesX.size(); ++i) {
        lowerStateLimits.push_back(stateSpaceInformation_->containedLinkLinearVelocitiesXLimits[i][0]);
        upperStateLimits.push_back(stateSpaceInformation_->containedLinkLinearVelocitiesXLimits[i][1]);
    }

    for (size_t i = 0; i != stateSpaceInformation_->containedLinkLinearVelocitiesY.size(); ++i) {
        lowerStateLimits.push_back(stateSpaceInformation_->containedLinkLinearVelocitiesYLimits[i][0]);
        upperStateLimits.push_back(stateSpaceInformation_->containedLinkLinearVelocitiesYLimits[i][1]);
    }

    for (size_t i = 0; i != stateSpaceInformation_->containedLinkLinearVelocitiesZ.size(); ++i) {
        lowerStateLimits.push_back(stateSpaceInformation_->containedLinkLinearVelocitiesZLimits[i][0]);
        upperStateLimits.push_back(stateSpaceInformation_->containedLinkLinearVelocitiesZLimits[i][1]);
    }

    for (size_t i = 0; i < stateSpaceInformation_->containedLinkVelocitiesLinear.size(); i++) {
        for (size_t j = 0; j < 3; j++) {
            //stateSpaceInformation_->containedLinkVelocitiesLinearLimits[i][j]
            lowerStateLimits.push_back(stateSpaceInformation_->containedLinkVelocitiesLinearLimits[i][0]);
            upperStateLimits.push_back(stateSpaceInformation_->containedLinkVelocitiesLinearLimits[i][1]);
        }
    }

    for (size_t i = 0; i < stateSpaceInformation_->containedLinkVelocitiesAngular.size(); i++) {
        for (size_t j = 0; j < 3; j++) {
            lowerStateLimits.push_back(stateSpaceInformation_->containedLinkVelocitiesAngularLimits[i][0]);
            upperStateLimits.push_back(stateSpaceInformation_->containedLinkVelocitiesAngularLimits[i][1]);
        }
    }
}

void GazeboInterface::getObservationLimits(VectorFloat& lowerObservationLimits, VectorFloat& upperObservationLimits)
{
    lowerObservationLimits.clear();
    upperObservationLimits.clear();
    auto jointMap = worldJointMap_.at(mainWorldName_);
    for (auto & jointName : observationSpaceInformation_->jointPositions) {
        for (auto & nameJointPair : jointMap) {
            if (nameJointPair.first.find(jointName) != std::string::npos) {
                lowerObservationLimits.push_back(nameJointPair.second->GetLowerLimit(0).Radian());
                upperObservationLimits.push_back(nameJointPair.second->GetUpperLimit(0).Radian());
            }
        }
    }

    for (auto & jointName : observationSpaceInformation_->jointVelocities) {
        for (auto & nameJointPair : jointMap) {
            if (nameJointPair.first.find(jointName) != std::string::npos) {
                lowerObservationLimits.push_back(-nameJointPair.second->GetVelocityLimit(0));
                upperObservationLimits.push_back(nameJointPair.second->GetVelocityLimit(0));
            }
        }
    }

    for (size_t i = 0; i < observationSpaceInformation_->containedLinkPoses.size(); i++) {
        for (size_t j = 0; j < 6; j++) {
            lowerObservationLimits.push_back(observationSpaceInformation_->containedLinkPosesLowerLimits[i][j]);
            upperObservationLimits.push_back(observationSpaceInformation_->containedLinkPosesUpperLimits[i][j]);
        }
    }

    for (size_t i = 0; i != observationSpaceInformation_->containedLinkPositionsX.size(); ++i) {
        lowerObservationLimits.push_back(observationSpaceInformation_->containedLinkPositionsXLimits[i][0]);
        upperObservationLimits.push_back(observationSpaceInformation_->containedLinkPositionsXLimits[i][1]);
    }

    for (size_t i = 0; i != observationSpaceInformation_->containedLinkPositionsY.size(); ++i) {
        lowerObservationLimits.push_back(observationSpaceInformation_->containedLinkPositionsYLimits[i][0]);
        upperObservationLimits.push_back(observationSpaceInformation_->containedLinkPositionsYLimits[i][1]);
    }

    for (size_t i = 0; i != observationSpaceInformation_->containedLinkPositionsZ.size(); ++i) {
        lowerObservationLimits.push_back(observationSpaceInformation_->containedLinkPositionsZLimits[i][0]);
        upperObservationLimits.push_back(observationSpaceInformation_->containedLinkPositionsZLimits[i][1]);
    }

    for (size_t i = 0; i != observationSpaceInformation_->containedLinkOrientationsX.size(); ++i) {
        lowerObservationLimits.push_back(observationSpaceInformation_->containedLinkOrientationsXLimits[i][0]);
        upperObservationLimits.push_back(observationSpaceInformation_->containedLinkOrientationsXLimits[i][1]);
    }

    for (size_t i = 0; i != observationSpaceInformation_->containedLinkOrientationsY.size(); ++i) {
        lowerObservationLimits.push_back(observationSpaceInformation_->containedLinkOrientationsYLimits[i][0]);
        upperObservationLimits.push_back(observationSpaceInformation_->containedLinkOrientationsYLimits[i][1]);
    }

    for (size_t i = 0; i != observationSpaceInformation_->containedLinkOrientationsZ.size(); ++i) {
        lowerObservationLimits.push_back(observationSpaceInformation_->containedLinkOrientationsZLimits[i][0]);
        upperObservationLimits.push_back(observationSpaceInformation_->containedLinkOrientationsZLimits[i][1]);
    }

    for (size_t i = 0; i != observationSpaceInformation_->containedLinkLinearVelocitiesX.size(); ++i) {
        lowerObservationLimits.push_back(observationSpaceInformation_->containedLinkLinearVelocitiesXLimits[i][0]);
        upperObservationLimits.push_back(observationSpaceInformation_->containedLinkLinearVelocitiesXLimits[i][1]);
    }

    for (size_t i = 0; i != observationSpaceInformation_->containedLinkLinearVelocitiesY.size(); ++i) {
        lowerObservationLimits.push_back(observationSpaceInformation_->containedLinkLinearVelocitiesYLimits[i][0]);
        upperObservationLimits.push_back(observationSpaceInformation_->containedLinkLinearVelocitiesYLimits[i][1]);
    }

    for (size_t i = 0; i != observationSpaceInformation_->containedLinkLinearVelocitiesZ.size(); ++i) {
        lowerObservationLimits.push_back(observationSpaceInformation_->containedLinkLinearVelocitiesZLimits[i][0]);
        upperObservationLimits.push_back(observationSpaceInformation_->containedLinkLinearVelocitiesZLimits[i][1]);
    }

    for (size_t i = 0; i != observationSpaceInformation_->containedLinkVelocitiesLinear.size(); ++i) {
        for (size_t j = 0; j != 3; ++j) {
            //observationSpaceInformation_->containedLinkVelocitiesLinearLimits[i][j]
            lowerObservationLimits.push_back(observationSpaceInformation_->containedLinkVelocitiesLinearLimits[i][0]);
            upperObservationLimits.push_back(observationSpaceInformation_->containedLinkVelocitiesLinearLimits[i][1]);
        }
    }

    for (size_t i = 0; i != observationSpaceInformation_->containedLinkVelocitiesAngular.size(); ++i) {
        for (size_t j = 0; j != 3; ++j) {
            lowerObservationLimits.push_back(observationSpaceInformation_->containedLinkVelocitiesAngularLimits[i][0]);
            upperObservationLimits.push_back(observationSpaceInformation_->containedLinkVelocitiesAngularLimits[i][1]);
        }
    }

    VectorFloat lowerLimitsSensors, upperLimitsSensors;
    sensorInterface_->getObservationLimits(lowerLimitsSensors, upperLimitsSensors);
    lowerObservationLimits.insert(lowerObservationLimits.end(), lowerLimitsSensors.begin(), lowerLimitsSensors.end());
    upperObservationLimits.insert(upperObservationLimits.end(), upperLimitsSensors.begin(), upperLimitsSensors.end());
}

void GazeboInterface::getActionLimits(VectorFloat& lowerLimits, VectorFloat& upperLimits)
{
    lowerLimits.clear();
    upperLimits.clear();
    gazebo::physics::JointPtr joint;
    for (auto & torqueControlledJoint : actionSpaceInformation_->torqueControlledJoints) {
        joint = worldJointMap_[mainWorldName_][torqueControlledJoint];
        lowerLimits.push_back(-joint->GetEffortLimit(0));
        upperLimits.push_back(joint->GetEffortLimit(0));
    }

    for (auto & velocityControlledJoint : actionSpaceInformation_->velocityControlledJoints) {
        joint = worldJointMap_[mainWorldName_][velocityControlledJoint];
        lowerLimits.push_back(-joint->GetVelocityLimit(0));
        upperLimits.push_back(joint->GetVelocityLimit(0));
    }

    for (auto & positionControlledJoint : actionSpaceInformation_->positionControlledJoints) {
        joint = worldJointMap_[mainWorldName_][positionControlledJoint];
        lowerLimits.push_back(joint->GetLowerLimit(0).Radian());
        upperLimits.push_back(joint->GetUpperLimit(0).Radian());
    }
}

void GazeboInterface::getRobotBoundingBoxesDirty(BoundingBoxes* boundingBoxes)
{
    if (cachedCollisionLinks_.size() == 0) {
        LinkMap lm = worldRobotLinkMap_[mainWorldName_];
        for (size_t i = 0; i < boundingBoxes->linkNames.size(); i++) {
            LinkPtr linkPtr = lm[boundingBoxes->linkNames[i]];
            cachedCollisionLinks_.push_back(linkPtr);
        }
    }

    gazebo::math::Box collisionBoundingBox;
    gazebo::math::Vector3 center;
    FloatType lenX, lenY, lenZ;
    if (boundingBoxes->linkNames.size() == 0) {
        return;
    }

    boundingBoxes->boxes = std::vector<VectorFloat> (boundingBoxes->linkNames.size());
    boundingBoxes->worldPoses = std::vector<VectorFloat> (boundingBoxes->linkNames.size());
    unsigned int idx = 0;
    for (size_t i = 0; i != cachedCollisionLinks_.size(); i++) {
        for (auto & collision : cachedCollisionLinks_[i]->GetCollisions()) {
            collision->Update();
        }

        //VectorFloat linkBoundingBox(6);
        VectorFloat linkWorldPoseVec(7);
        collisionBoundingBox = cachedCollisionLinks_[i]->GetCollisionBoundingBox();
        center = collisionBoundingBox.GetCenter();
        lenX = collisionBoundingBox.GetXLength();
        lenY = collisionBoundingBox.GetYLength();
        lenZ = collisionBoundingBox.GetZLength();
        if (lenX > 0 || lenY > 0 || lenZ > 0) {
            // Filter out dimensionless links
            gazebo::math::Pose linkCollisionWorldPose =
                cachedCollisionLinks_[i]->GetCollisions() [0]->GetInitialRelativePose() * cachedCollisionLinks_[i]->GetWorldPose();
            linkWorldPoseVec[0] = linkCollisionWorldPose.pos.x;
            linkWorldPoseVec[1] = linkCollisionWorldPose.pos.y;
            linkWorldPoseVec[2] = linkCollisionWorldPose.pos.z;
            linkWorldPoseVec[3] = linkCollisionWorldPose.rot.x;
            linkWorldPoseVec[4] = linkCollisionWorldPose.rot.y;
            linkWorldPoseVec[5] = linkCollisionWorldPose.rot.z;
            linkWorldPoseVec[6] = linkCollisionWorldPose.rot.w;
            boundingBoxes->worldPoses[i] = linkWorldPoseVec;
            idx++;
        } else {
            auto s = getState_(mainWorldName_);
            printVector(s, "s");
            LOGGING("dim 0 " + cachedCollisionLinks_[i]->GetName());
            getchar();
        }
    }

    boundingBoxes->linkNames.resize(idx);
    boundingBoxes->worldPoses.resize(idx);
    boundingBoxes->boxes.resize(idx);
}

void GazeboInterface::getRobotBoundingBoxes(const VectorFloat& stateVec, GazeboWorldStatePtr& worldState, BoundingBoxes* boundingBoxes)
{
    if (worldState) {
        if (* (lastConstructedWorldStates_[mainWorldName_].get()) != * (worldState.get()))
            setWorldState(worldState);
    }

    setStateManuallyNew(stateVec, mainWorldName_);
    getRobotBoundingBoxesDirty(boundingBoxes);
    auto vecAfterSet = getState_(mainWorldName_);
}

VectorFloat GazeboInterface::getLinkPosition(const VectorFloat& robotState,
        const GazeboWorldStatePtr& worldState,
        const std::string& linkName)
{
    if (worldState) {
        if (* (lastConstructedWorldStates_[mainWorldName_].get()) != * (worldState.get()))
            setWorldState(worldState);
    }

    setStateManuallyNew(robotState, mainWorldName_);
    const gazebo::math::Pose linkPose =
        worldRobotLinkMap_[mainWorldName_][linkName]->GetWorldPose();
    VectorFloat position(3);
    position[0] = linkPose.pos[0];
    position[1] = linkPose.pos[1];
    position[2] = linkPose.pos[2];
    return position;
}

Matrixdf GazeboInterface::getLinkPose(const VectorFloat& robotState,
                                      const GazeboWorldStatePtr& worldState,
                                      const std::string& linkName)
{
    if (worldState) {
        if (* (lastConstructedWorldStates_[mainWorldName_].get()) != * (worldState.get()))
            setWorldState(worldState);
    }

    setStateManuallyNew(robotState, mainWorldName_);
    const gazebo::math::Pose linkPose =
        worldRobotLinkMap_[mainWorldName_][linkName]->GetWorldPose();
    Quaternionf resQuat(linkPose.rot.w, linkPose.rot.x, linkPose.rot.y, linkPose.rot.z);
    Matrix3f resRotMat = math::quaternionToRotationMatrix(resQuat);
    Matrixdf resMat = Matrixdf::Identity(4, 4);
    resMat.block<3, 3> (0, 0) = resRotMat;
    resMat(0, 3) = linkPose.pos.x;
    resMat(1, 3) = linkPose.pos.y;
    resMat(2, 3) = linkPose.pos.z;
    return resMat;
}

std::vector<VectorFloat> GazeboInterface::getLinksCOGPose(const VectorFloat& robotState,
        const GazeboWorldStatePtr& worldState,
        const VectorString& linkNames)
{
    if (worldState) {
        if (* (lastConstructedWorldStates_[mainWorldName_].get()) != * (worldState.get()))
            setWorldState(worldState);
    }

    setStateManuallyNew(robotState, mainWorldName_);
    return getLinksCOGPoseDirty(linkNames);
}

std::vector<VectorFloat> GazeboInterface::getLinksCOGPoseDirty(const VectorString& linkNames)
{
    std::vector<VectorFloat> poses(linkNames.size());
    for (size_t i = 0; i != linkNames.size(); ++i) {
        VectorFloat pose(7);
        const gazebo::math::Pose linkPose =
            worldRobotLinkMap_.at(mainWorldName_).at(linkNames[i])->GetWorldCoGPose();
        pose[0] = linkPose.pos.x;
        pose[1] = linkPose.pos.y;
        pose[2] = linkPose.pos.z;
        pose[3] = linkPose.rot.w;
        pose[4] = linkPose.rot.x;
        pose[5] = linkPose.rot.y;
        pose[6] = linkPose.rot.z;
        poses[i] = pose;
    }

    return poses;
}

std::vector<VectorFloat> GazeboInterface::getLinksCollisionWorldPoses(const VectorFloat& robotState,
        const GazeboWorldStatePtr& worldState,
        const VectorString& linkNames)
{
    if (worldState) {
        if (* (lastConstructedWorldStates_[mainWorldName_].get()) != * (worldState.get()))
            setWorldState(worldState);
    }

    setStateManuallyNew(robotState, mainWorldName_);
    return getLinksCollisionWorldPosesDirty(linkNames);

}

std::vector<VectorFloat> GazeboInterface::getLinksCollisionWorldPosesDirty(const VectorString& linkNames)
{
    std::vector<VectorFloat> poses;
    for (size_t i = 0; i != linkNames.size(); ++i) {
        auto link = worldRobotLinkMap_.at(mainWorldName_).at(linkNames[i]);
        auto linkPose = link->GetWorldPose();
        auto collisions = link->GetCollisions();
        for (auto & collision : collisions) {
            auto collisionPose = collision->GetInitialRelativePose() + linkPose;
            VectorFloat pose(7, 0.0);
            pose[0] = collisionPose.pos.x;
            pose[1] = collisionPose.pos.y;
            pose[2] = collisionPose.pos.z;
            pose[3] = collisionPose.rot.w;
            pose[4] = collisionPose.rot.x;
            pose[5] = collisionPose.rot.y;
            pose[6] = collisionPose.rot.z;
            poses.push_back(pose);
        }
    }

    return poses;
}

std::string GazeboInterface::getRobotName() const
{
    return mainRobotName_;
}

void GazeboInterface::getRobotVisualPoses(VectorFloat& state,
        const VectorString& visualNames,
        std::vector<Matrixdf>& visualPoses,
        const GazeboWorldStatePtr &worldState)
{
    if (worldState)
        setWorldState(worldState);
    setStateManuallyNew(state, mainWorldName_);
    visualPoses = std::vector<Matrixdf> (visualNames.size());
    for (size_t i = 0; i != visualNames.size(); ++i) {
        uint32_t visualId;
        for (auto & linkEntry : worldRobotLinkMap_[mainWorldName_]) {
            if (linkEntry.second->VisualId(visualNames[i], visualId)) {
                ignition::math::Pose3d visualPose;
                linkEntry.second->VisualPose(visualId, visualPose);
                gazebo::math::Pose linkPose = linkEntry.second->GetWorldPose();
                ignition::math::Pose3d linkPoseIgn(linkPose.pos.x,
                                                   linkPose.pos.y,
                                                   linkPose.pos.z,
                                                   linkPose.rot.w,
                                                   linkPose.rot.x,
                                                   linkPose.rot.y,
                                                   linkPose.rot.z);
                ignition::math::Pose3d resPose = visualPose * linkPoseIgn;
                Quaternionf resQuat(resPose.Rot().W(), resPose.Rot().X(), resPose.Rot().Y(), resPose.Rot().Z());
                Matrix3f resRotMat = math::quaternionToRotationMatrix(resQuat);
                Matrixdf resMat = Matrixdf::Identity(4, 4);
                resMat.block<3, 3> (0, 0) = resRotMat;
                resMat(0, 3) = resPose.Pos().X();
                resMat(1, 3) = resPose.Pos().Y();
                resMat(2, 3) = resPose.Pos().Z();
                visualPoses[i] = resMat;
                break;
            }
        }
    }
}

void GazeboInterface::setStateSpaceInformation(const StateSpaceInformationPtr& stateSpaceInformation)
{
    stateSpaceInformation_ = stateSpaceInformation;
    VectorString jointPositionVec = stateSpaceInformation_->jointPositions;
    VectorString jointVelocityVec = stateSpaceInformation_->jointVelocities;
    VectorString linkPosesVec = stateSpaceInformation_->containedLinkPoses;
    VectorString linkPositionsXVec = stateSpaceInformation_->containedLinkPositionsX;
    VectorString linkPositionsYVec = stateSpaceInformation_->containedLinkPositionsY;
    VectorString linkPositionsZVec = stateSpaceInformation_->containedLinkPositionsZ;
    VectorString linkOrientationsXVec = stateSpaceInformation_->containedLinkOrientationsX;
    VectorString linkOrientationsYVec = stateSpaceInformation_->containedLinkOrientationsY;
    VectorString linkOrientationsZVec = stateSpaceInformation_->containedLinkOrientationsZ;
    VectorString linkVelocitiesLinearVec = stateSpaceInformation_->containedLinkVelocitiesLinear;
    VectorString linkVelocitiesAngularVec = stateSpaceInformation_->containedLinkVelocitiesAngular;
    std::vector<VectorString> redundantJoints = stateSpaceInformation_->redundantJoints;
    unsigned int startIndex = 0;

    std::unordered_map<std::string, VectorString> redundancyMapPosition = generateRedundancyMap(jointPositionVec, redundantJoints);
    std::unordered_map<std::string, VectorString> redundancyMapVelocity = generateRedundancyMap(jointVelocityVec, redundantJoints);

    if (jointPositionVec.size() > 0) {
        SetStateFunction setPosition = [this, jointPositionVec, redundancyMapPosition, startIndex](const VectorFloat & stateVec, const std::string & worldName) {
            VectorString redJoints;
            std::string redundantJoint;
            FloatType jaToSet = 0;
            JointMap jm = worldJointMap_[worldName];
            for (size_t i = 0; i < jointPositionVec.size(); i++) {
                JointPtr jp = jm[jointPositionVec[i]];
                jaToSet = stateVec[startIndex + i];
                jp->SetPosition(0, jaToSet);
                if (redundancyMapPosition.count(jointPositionVec[i]) == 1) {
                    // Need to set the state of the redundant joints as well
                    redJoints = redundancyMapPosition.at(jointPositionVec[i]);
                    for (size_t j = 0; j < redJoints.size(); j++) {
                        redundantJoint = redJoints[j];
                        jm[redundantJoint]->SetPosition(0, jaToSet);
                    }
                }
            }

        };

        GetStateFunction getPosition = [this, jointPositionVec, startIndex](VectorFloat & stateVec, const std::string & worldName) {
            JointMap jm = worldJointMap_[worldName];
            for (size_t i = startIndex; i < jointPositionVec.size(); i++) {
                stateVec[i] = jm[jointPositionVec[i]]->GetAngle(0).Radian();
            }
        };

        setStateFunctions_.push_back(setPosition);
        getStateFunctions_.push_back(getPosition);
        startIndex += jointPositionVec.size();
    }

    if (jointVelocityVec.size() > 0) {
        SetStateFunction setVelocity = [this, jointVelocityVec, redundancyMapVelocity, startIndex](const VectorFloat & stateVec, const std::string & worldName) {
            VectorString redJoints;
            std::string redundantJoint;
            JointMap jm = worldJointMap_[worldName];
            for (size_t i = 0; i < jointVelocityVec.size(); i++) {
                setJointVelocityRecursive(jm[jointVelocityVec[i]], stateVec[startIndex + i]);

                if (redundancyMapVelocity.count(jointVelocityVec[i]) == 1) {
                    redJoints = redundancyMapVelocity.at(jointVelocityVec[i]);
                    for (size_t j = 0; j < redJoints.size(); j++) {
                        redundantJoint = redJoints[j];
                        setJointVelocityRecursive(jm[redundantJoint], stateVec[startIndex + i]);
                    }
                }

            }
        };

        GetStateFunction getVelocity = [this, jointVelocityVec, startIndex](VectorFloat & stateVec, const std::string & worldName) {
            JointMap jm = worldJointMap_[worldName];
            for (size_t i = 0; i < jointVelocityVec.size(); i++) {
                stateVec[startIndex + i] = jm[jointVelocityVec[i]]->GetVelocity(0);
            }
        };

        setStateFunctions_.push_back(setVelocity);
        getStateFunctions_.push_back(getVelocity);
        startIndex += jointVelocityVec.size();
    }

    if (linkPosesVec.size() > 0) {
        SetStateFunction setLinkPoses = [this, linkPosesVec, startIndex](const VectorFloat & stateVec, const std::string & worldName) {
            unsigned int idx = startIndex;
            LinkMap lm = worldLinkMap_[worldName];
            for (auto & link : linkPosesVec) {
                gazebo::math::Pose linkPose(stateVec[idx],
                                            stateVec[idx + 1],
                                            stateVec[idx + 2],
                                            stateVec[idx + 3],
                                            stateVec[idx + 4],
                                            stateVec[idx + 5]);
                auto lmEntry = lm[link];
                lmEntry->SetWorldPose(linkPose);
                idx += 6;
            }
        };

        GetStateFunction getLinkPoses = [this, linkPosesVec, startIndex](VectorFloat & stateVec, const std::string & worldName) {
            unsigned int idx = startIndex;
            gazebo::math::Pose pose;
            LinkMap lm = worldLinkMap_[worldName];
            for (auto & link : linkPosesVec) {
                pose = lm[link]->GetWorldPose();
                gazebo::math::Vector3 orientation = pose.rot.GetAsEuler();
                stateVec[idx] = pose.pos.x;
                stateVec[idx + 1] = pose.pos.y;
                stateVec[idx + 2] = pose.pos.z;
                stateVec[idx + 3] = orientation.x;
                stateVec[idx + 4] = orientation.y;
                stateVec[idx + 5] = orientation.z;
                idx += 6;
            }
        };

        setStateFunctions_.push_back(setLinkPoses);
        getStateFunctions_.push_back(getLinkPoses);
        startIndex += linkPosesVec.size() * 6;
    }

    if (linkPositionsXVec.size() > 0) {
        SetStateFunction setLinkPositionsX = [this, linkPositionsXVec, startIndex](const VectorFloat & stateVec, const std::string & worldName) {
            unsigned int idx = startIndex;
            LinkMap lm = worldLinkMap_[worldName];
            for (auto & link : linkPositionsXVec) {
                auto lmEntry = lm[link];
                auto currentPose = lmEntry->GetWorldPose();
                gazebo::math::Vector3 newPosition(stateVec[idx], currentPose.pos.y, currentPose.pos.z);
                currentPose.Set(newPosition, currentPose.rot);
                lmEntry->SetWorldPose(currentPose);
                idx += 1;
            }

        };

        GetStateFunction getLinkPositionsX = [this, linkPositionsXVec, startIndex](VectorFloat & stateVec, const std::string & worldName) {
            unsigned int idx = startIndex;
            gazebo::math::Pose pose;
            LinkMap lm = worldLinkMap_[worldName];
            for (auto & link : linkPositionsXVec) {
                pose = lm[link]->GetWorldPose();
                stateVec[idx] = pose.pos.x;
                idx += 1;
            }
        };

        setStateFunctions_.push_back(setLinkPositionsX);
        getStateFunctions_.push_back(getLinkPositionsX);
        startIndex += linkPositionsXVec.size();
    }

    if (linkPositionsYVec.size() > 0) {
        SetStateFunction setLinkPositionsY = [this, linkPositionsYVec, startIndex](const VectorFloat & stateVec, const std::string & worldName) {
            unsigned int idx = startIndex;
            LinkMap lm = worldLinkMap_[worldName];
            for (auto & link : linkPositionsYVec) {
                auto lmEntry = lm[link];
                auto currentPose = lmEntry->GetWorldPose();
                gazebo::math::Vector3 newPosition(currentPose.pos.x, stateVec[idx], currentPose.pos.z);
                currentPose.Set(newPosition, currentPose.rot);
                lmEntry->SetWorldPose(currentPose);
                idx += 1;
            }

        };

        GetStateFunction getLinkPositionsY = [this, linkPositionsYVec, startIndex](VectorFloat & stateVec, const std::string & worldName) {
            unsigned int idx = startIndex;
            gazebo::math::Pose pose;
            LinkMap lm = worldLinkMap_[worldName];
            for (auto & link : linkPositionsYVec) {
                pose = lm[link]->GetWorldPose();
                stateVec[idx] = pose.pos.y;
                idx += 1;
            }
        };

        setStateFunctions_.push_back(setLinkPositionsY);
        getStateFunctions_.push_back(getLinkPositionsY);
        startIndex += linkPositionsYVec.size();
    }

    if (linkPositionsZVec.size() > 0) {
        SetStateFunction setLinkPositionsZ = [this, linkPositionsZVec, startIndex](const VectorFloat & stateVec, const std::string & worldName) {
            unsigned int idx = startIndex;
            LinkMap lm = worldLinkMap_[worldName];
            for (auto & link : linkPositionsZVec) {
                auto lmEntry = lm[link];
                auto currentPose = lmEntry->GetWorldPose();
                gazebo::math::Vector3 newPosition(currentPose.pos.x, currentPose.pos.y, stateVec[idx]);
                currentPose.Set(newPosition, currentPose.rot);
                lmEntry->SetWorldPose(currentPose);
                idx += 1;
            }

        };

        GetStateFunction getLinkPositionsZ = [this, linkPositionsZVec, startIndex](VectorFloat & stateVec, const std::string & worldName) {
            unsigned int idx = startIndex;
            gazebo::math::Pose pose;
            LinkMap lm = worldLinkMap_[worldName];
            for (auto & link : linkPositionsZVec) {
                pose = lm[link]->GetWorldPose();
                stateVec[idx] = pose.pos.z;
                idx += 1;
            }
        };

        setStateFunctions_.push_back(setLinkPositionsZ);
        getStateFunctions_.push_back(getLinkPositionsZ);
        startIndex += linkPositionsZVec.size();
    }

    if (linkOrientationsXVec.size() > 0) {
        SetStateFunction setLinkOrientationsX = [this, linkOrientationsXVec, startIndex](const VectorFloat & stateVec, const std::string & worldName) {
            unsigned int idx = startIndex;
            LinkMap lm = worldLinkMap_[worldName];
            for (auto & link : linkOrientationsXVec) {
                auto lmEntry = lm[link];
                auto currentPose = lmEntry->GetWorldPose();
                auto currentEuler = currentPose.rot.GetAsEuler();
                gazebo::math::Vector3 newRot(stateVec[idx], currentEuler[1], currentEuler[2]);
                currentPose.Set(currentPose.pos, newRot);
                lmEntry->SetWorldPose(currentPose);
                idx += 1;
            }

        };

        GetStateFunction getLinkOrientationsX = [this, linkOrientationsXVec, startIndex](VectorFloat & stateVec, const std::string & worldName) {
            unsigned int idx = startIndex;
            gazebo::math::Pose pose;
            LinkMap lm = worldLinkMap_[worldName];
            for (auto & link : linkOrientationsXVec) {
                pose = lm[link]->GetWorldPose();
                stateVec[idx] = pose.rot.GetAsEuler().x;
                idx += 1;
            }
        };

        setStateFunctions_.push_back(setLinkOrientationsX);
        getStateFunctions_.push_back(getLinkOrientationsX);
        startIndex += linkOrientationsXVec.size();
    }

    if (linkOrientationsYVec.size() > 0) {
        SetStateFunction setLinkOrientationsY = [this, linkOrientationsYVec, startIndex](const VectorFloat & stateVec, const std::string & worldName) {
            unsigned int idx = startIndex;
            LinkMap lm = worldLinkMap_[worldName];
            for (auto & link : linkOrientationsYVec) {
                auto lmEntry = lm[link];
                auto currentPose = lmEntry->GetWorldPose();
                auto currentEuler = currentPose.rot.GetAsEuler();
                gazebo::math::Vector3 newRot(currentEuler[0], stateVec[idx], currentEuler[2]);
                currentPose.Set(currentPose.pos, newRot);
                lmEntry->SetWorldPose(currentPose);
                idx += 1;
            }

        };

        GetStateFunction getLinkOrientationsY = [this, linkOrientationsYVec, startIndex](VectorFloat & stateVec, const std::string & worldName) {
            unsigned int idx = startIndex;
            gazebo::math::Pose pose;
            LinkMap lm = worldLinkMap_[worldName];
            for (auto & link : linkOrientationsYVec) {
                pose = lm[link]->GetWorldPose();
                stateVec[idx] = pose.rot.GetAsEuler().y;
                idx += 1;
            }
        };

        setStateFunctions_.push_back(setLinkOrientationsY);
        getStateFunctions_.push_back(getLinkOrientationsY);
        startIndex += linkOrientationsYVec.size();
    }

    if (linkOrientationsZVec.size() > 0) {
        SetStateFunction setLinkOrientationsZ = [this, linkOrientationsZVec, startIndex](const VectorFloat & stateVec, const std::string & worldName) {
            unsigned int idx = startIndex;
            LinkMap lm = worldLinkMap_[worldName];
            for (auto & link : linkOrientationsZVec) {
                auto lmEntry = lm[link];
                auto currentPose = lmEntry->GetWorldPose();
                auto currentEuler = currentPose.rot.GetAsEuler();
                gazebo::math::Vector3 newRot(currentEuler[0], currentEuler[1], stateVec[idx]);
                currentPose.Set(currentPose.pos, newRot);
                lmEntry->SetWorldPose(currentPose);
                idx += 1;
            }

        };

        GetStateFunction getLinkOrientationsZ = [this, linkOrientationsZVec, startIndex](VectorFloat & stateVec, const std::string & worldName) {
            unsigned int idx = startIndex;
            gazebo::math::Pose pose;
            LinkMap lm = worldLinkMap_[worldName];
            for (auto & link : linkOrientationsZVec) {
                pose = lm[link]->GetWorldPose();
                stateVec[idx] = pose.rot.GetAsEuler().z;
                idx += 1;
            }
        };

        setStateFunctions_.push_back(setLinkOrientationsZ);
        getStateFunctions_.push_back(getLinkOrientationsZ);
        startIndex += linkOrientationsZVec.size();
    }

    if (linkVelocitiesLinearVec.size() > 0) {
        SetStateFunction setLinearVelocities = [this, linkVelocitiesLinearVec, startIndex](const VectorFloat & stateVec, const std::string & worldName) {
            unsigned int idx = startIndex;
            LinkMap lm = worldLinkMap_[worldName];
            for (auto & link : linkVelocitiesLinearVec) {
                gazebo::math::Vector3 vel(stateVec[idx], stateVec[idx + 1], stateVec[idx + 2]);
                lm[link]->SetLinearVel(vel);
                idx += 3;
            }
        };

        GetStateFunction getLinearVelocities = [this, linkVelocitiesLinearVec, startIndex](VectorFloat & stateVec, const std::string & worldName) {
            unsigned int idx = startIndex;
            gazebo::math::Vector3 vel;
            LinkMap lm = worldLinkMap_[worldName];
            for (auto & link : linkVelocitiesLinearVec) {
                vel = lm[link]->GetWorldLinearVel();
                stateVec[idx] = vel.x;
                stateVec[idx + 1] = vel.y;
                stateVec[idx + 2] = vel.z;
                idx += 3;
            }
        };

        setStateFunctions_.push_back(setLinearVelocities);
        getStateFunctions_.push_back(getLinearVelocities);
        startIndex += linkVelocitiesLinearVec.size() * 3;
    }

    if (linkVelocitiesAngularVec.size() > 0) {
        SetStateFunction setAngularVelocities = [this, linkVelocitiesAngularVec, startIndex](const VectorFloat & stateVec, const std::string & worldName) {
            unsigned int idx = startIndex;
            LinkMap lm = worldLinkMap_[worldName];
            for (auto & link : linkVelocitiesAngularVec) {
                gazebo::math::Vector3 vel(stateVec[idx], stateVec[idx + 1], stateVec[idx + 2]);
                lm[link]->SetAngularVel(vel);
                idx += 3;
            }
        };

        GetStateFunction getAngularVelocities = [this, linkVelocitiesAngularVec, startIndex](VectorFloat & stateVec, const std::string & worldName) {
            unsigned int idx = startIndex;
            gazebo::math::Vector3 vel;
            LinkMap lm = worldLinkMap_[worldName];
            for (auto & link : linkVelocitiesAngularVec) {
                vel = lm[link]->GetWorldAngularVel();
                stateVec[idx] = vel.x;
                stateVec[idx + 1] = vel.y;
                stateVec[idx + 2] = vel.z;
                idx += 3;
            }
        };

        setStateFunctions_.push_back(setAngularVelocities);
        getStateFunctions_.push_back(getAngularVelocities);
        startIndex += linkVelocitiesAngularVec.size() * 3;
    }

    stateSpaceDimension_ = startIndex;
}

VectorFloat GazeboInterface::getState_(const std::string& worldName)
{
    VectorFloat stateVec(stateSpaceDimension_);
    for (auto & getStateFunction : getStateFunctions_) {
        getStateFunction(stateVec, worldName);
    }

    return stateVec;
}

std::unordered_map<std::string, VectorString> GazeboInterface::generateRedundancyMap(const VectorString& joints,
        const std::vector<VectorString>& redundantJoints) const
{
    std::unordered_map<std::string, VectorString> redundancyMap;
    for (size_t i = 0; i < joints.size(); i++) {
        for (size_t j = 0; j < redundantJoints.size(); j++) {
            if (contains(redundantJoints[j], joints[i])) {
                VectorString red;
                red.assign(redundantJoints[j].begin() + 1, redundantJoints[j].end());
                redundancyMap[joints[i]] = red;
            }
        }
    }
    return redundancyMap;
}

void GazeboInterface::setActionSpaceInformation(const ActionSpaceInformationPtr& actionSpaceInformation)
{
    actionSpaceInformation_ = actionSpaceInformation;
    VectorString torqueControlledJoints = actionSpaceInformation_->torqueControlledJoints;
    VectorString velocityControlledJoints = actionSpaceInformation_->velocityControlledJoints;
    VectorString positionControlledJoints = actionSpaceInformation_->positionControlledJoints;
    std::vector<VectorString> redundantJoints = actionSpaceInformation_->redundantJoints;
    unsigned int startIndex = 0;
    std::unordered_map<std::string, VectorString> redundancyMapTorque = generateRedundancyMap(torqueControlledJoints, redundantJoints);
    std::unordered_map<std::string, VectorString> redundancyMapVelocity = generateRedundancyMap(velocityControlledJoints, redundantJoints);
    std::unordered_map<std::string, VectorString> redundancyMapPosition = generateRedundancyMap(positionControlledJoints, redundantJoints);

    if (torqueControlledJoints.size() > 0) {
        ApplyActionFunction applyTorque = [this,
                                           torqueControlledJoints,
                                           redundancyMapTorque,
        startIndex](const VectorFloat & actionVec, const std::string & worldName) {
            VectorString redJoints;
            FloatType jointVel;
            FloatType jointVelLimit;
            FloatType appliedTorque;
            size_t i = 0;
            JointMap jm = worldJointMap_[worldName];
            for (auto & joint : torqueControlledJoints) {
                gazebo::physics::JointPtr jp = jm.at(joint);
                //jp->ResetForcesApplied();
                appliedTorque = actionVec[startIndex + i];
                jointVel = jp->GetVelocity(0);
                jointVelLimit = jp->GetVelocityLimit(0);

                // Set the applied torque to zero if the joint velocity has been reached
                // and the torque is applied in the same direction
                if (jointVel > jointVelLimit - std::numeric_limits<FloatType>::epsilon()) {
                    appliedTorque = appliedTorque > 0 ? 0 : appliedTorque;
                } else if (jointVel < -jointVelLimit + std::numeric_limits<FloatType>::epsilon()) {
                    appliedTorque = appliedTorque < 0 ? 0 : appliedTorque;
                }

                jp->SetForce(0, appliedTorque);
                if (redundancyMapTorque.count(torqueControlledJoints[i]) == 1) {
                    // Need to set the action of the redundant joints as well
                    redJoints = redundancyMapTorque.at(torqueControlledJoints[i]);
                    for (size_t j = 0; j < redJoints.size(); j++) {
                        jm[redJoints[j]]->SetForce(0, appliedTorque);
                    }
                }

                i++;
            }
        };

        applyActionFunctions_.push_back(applyTorque);
        startIndex += torqueControlledJoints.size();
    }

    if (velocityControlledJoints.size() > 0) {
        ApplyActionFunction applyVelocity = [this,
                                             velocityControlledJoints,
                                             redundancyMapVelocity,
        startIndex](const VectorFloat & actionVec, const std::string & worldName) {
            VectorString redJoints;
            std::string redundantJoint;
            size_t i = 0;
            JointMap jm = worldJointMap_[worldName];
            for (auto & joint : velocityControlledJoints) {
                setJointVelocityRecursive(jm[joint], actionVec[startIndex + i]);
                if (redundancyMapVelocity.count(velocityControlledJoints[i]) == 1) {
                    // Need to set the action of the redundant joints as well
                    redJoints = redundancyMapVelocity.at(velocityControlledJoints[i]);
                    for (size_t j = 0; j < redJoints.size(); j++) {
                        redundantJoint = redJoints[j];
                        setJointVelocityRecursive(jm[redundantJoint], actionVec[startIndex + i]);
                    }
                }

                i++;
            }
        };

        applyActionFunctions_.push_back(applyVelocity);
        startIndex += velocityControlledJoints.size();
    }

    if (positionControlledJoints.size() > 0) {
        ApplyActionFunction setPosition = [this,
                                           positionControlledJoints,
                                           redundancyMapPosition,
        startIndex](const VectorFloat & actionVec, const std::string & worldName) {
            VectorString redJoints;
            std::string redundantJoint;
            size_t i = 0;
            JointMap jm = worldJointMap_[worldName];
            for (auto & joint : positionControlledJoints) {
                cout << "SET POSITION: " << jm[joint]->GetName() << endl;
                jm[joint]->SetPosition(0, actionVec[startIndex + i]);
                if (redundancyMapPosition.count(positionControlledJoints[i]) == 1) {
                    redJoints = redundancyMapPosition.at(positionControlledJoints[i]);
                    for (size_t j = 0; j < redJoints.size(); j++) {
                        redundantJoint = redJoints[j];
                        jm[redundantJoint]->SetPosition(0, actionVec[startIndex + i]);
                    }
                }

                i++;
            }
        };

        applyActionFunctions_.push_back(setPosition);
        startIndex += positionControlledJoints.size();
    }
}

void GazeboInterface::setObservationSpaceInformation(const ObservationSpaceInformationPtr& observationSpaceInformation)
{
    observationSpaceInformation_ = observationSpaceInformation;
    VectorString observedJointPositions = observationSpaceInformation_->jointPositions;
    VectorString observedJointVelocities = observationSpaceInformation_->jointVelocities;
    VectorString observedLinkPoses = observationSpaceInformation_->containedLinkPoses;
    VectorString observedLinkVelocitiesLinear = observationSpaceInformation_->containedLinkVelocitiesLinear;
    VectorString observedLinkVelocitiesAngular = observationSpaceInformation_->containedLinkVelocitiesAngular;
    unsigned int startIndex = 0;

    if (observedJointPositions.size() > 0) {
        GetObservationFunction obsFunct = [this,
                                           observedJointPositions,
        startIndex](VectorFloat & observationVec, const std::string & worldName) {
            JointMap jm = worldJointMap_[worldName];
            for (size_t i = 0; i < observedJointPositions.size(); i++) {
                observationVec[startIndex + i] = jm[observedJointPositions[i]]->GetAngle(0).Radian();
            }
        };

        getObservationFunctions_.push_back(obsFunct);
        startIndex += observedJointPositions.size();
    }

    if (observedJointVelocities.size() > 0) {
        GetObservationFunction obsFunct = [this,
                                           observedJointVelocities,
        startIndex](VectorFloat & observationVec, const std::string & worldName) {
            JointMap jm = worldJointMap_[worldName];
            for (size_t i = 0; i < observedJointVelocities.size(); i++) {
                observationVec[startIndex + i] = jm[observedJointVelocities[i]]->GetVelocity(0);
            }

        };

        getObservationFunctions_.push_back(obsFunct);
        startIndex += observedJointVelocities.size();
    }

    if (observedLinkPoses.size() > 0) {
        GetObservationFunction obsFunct = [this,
                                           observedLinkPoses,
        startIndex](VectorFloat & observationVec, const std::string & worldName) {
            unsigned int idx = startIndex;
            gazebo::math::Pose linkPose;
            gazebo::math::Vector3 rotation;
            LinkMap lm = worldLinkMap_[worldName];
            for (auto & link : observedLinkPoses) {
                linkPose = lm[link]->GetWorldPose();
                rotation = linkPose.rot.GetAsEuler();
                observationVec[idx] = linkPose.pos.x;
                observationVec[idx + 1] = linkPose.pos.y;
                observationVec[idx + 2] = linkPose.pos.z;
                observationVec[idx + 3] = rotation.x;
                observationVec[idx + 4] = rotation.y;
                observationVec[idx + 5] = rotation.z;
                idx += 6;
            }
        };

        getObservationFunctions_.push_back(obsFunct);
        startIndex += observedLinkPoses.size() * 6;
    }

    if (observedLinkVelocitiesLinear.size() > 0) {
        GetObservationFunction obsFunct = [this,
                                           observedLinkVelocitiesLinear,
        startIndex](VectorFloat & observationVec, const std::string & worldName) {
            unsigned int idx = startIndex;
            gazebo::math::Vector3 vel;
            LinkMap lm = worldLinkMap_[worldName];
            for (auto & link : observedLinkVelocitiesLinear) {
                vel = lm[link]->GetWorldLinearVel();
                observationVec[idx] = vel.x;
                observationVec[idx + 1] = vel.y;
                observationVec[idx + 2] = vel.z;
                idx += 3;
            }
        };

        getObservationFunctions_.push_back(obsFunct);
        startIndex += observedLinkVelocitiesLinear.size() * 3;
    }

    if (observedLinkVelocitiesAngular.size() > 0) {
        GetObservationFunction obsFunct = [this,
                                           observedLinkVelocitiesAngular,
        startIndex](VectorFloat & observationVec, const std::string & worldName) {
            unsigned int idx = startIndex;
            gazebo::math::Vector3 vel;
            LinkMap lm = worldLinkMap_[worldName];
            for (auto & link : observedLinkVelocitiesAngular) {
                vel = lm[link]->GetWorldAngularVel();
                observationVec[idx] = vel.x;
                observationVec[idx + 1] = vel.y;
                observationVec[idx + 2] = vel.z;
                idx += 3;
            }
        };

        getObservationFunctions_.push_back(obsFunct);
        startIndex += observedLinkVelocitiesAngular.size() * 3;
    }

    GetObservationFunction sensorObservations = [this, startIndex](VectorFloat & observationVec, const std::string & worldName) {
        VectorFloat sensorObservationVec;
        sensorInterface_->getCombinedObservation(sensorObservationVec, worldName);
        for (size_t i = 0; i != sensorObservationVec.size(); ++i) {
            observationVec[i + startIndex] = sensorObservationVec[i];
        }
    };

    getObservationFunctions_.push_back(sensorObservations);
}

void GazeboInterface::setJointVelocityRecursive(gazebo::physics::JointPtr& joint, const FloatType& velocity)
{
    joint->SetVelocity(0, velocity);
    gazebo::physics::LinkPtr childLink = joint->GetChild();
    for (auto & childJoint : childLink->GetChildJoints()) {
        FloatType childJointVelocity = childJoint->GetVelocity(0);
        setJointVelocityRecursive(childJoint, childJointVelocity);
    }

}

void GazeboInterface::setLinkPoseRecursive(LinkPtr& link, gazebo::math::Pose& pose, gazebo::math::Pose& childLinkPose)
{
    auto p = (link->GetWorldPose() - childLinkPose) + pose;
    link->SetWorldPose(p);
    for (auto & childLink : link->GetChildJointsLinks()) {
        setLinkPoseRecursive(childLink, pose, childLinkPose);
    }
}

void GazeboInterface::snapVelocityRecursive(const std::string& worldName, const bool& verbose)
{
    for (auto & jointEntry : rootJoints_) {
        snapVelocityRecursiveHelper(jointEntry.second);
    }
}

void GazeboInterface::snapVelocityRecursiveHelper(gazebo::physics::JointPtr& joint)
{
    FloatType velLimit = joint->GetVelocityLimit(0);
    if (velLimit >= 0) {
        FloatType currentVelocity = joint->GetVelocity(0);
        if (currentVelocity > velLimit - 1e-7) {
            setJointVelocityRecursive(joint, velLimit - 1e-7);
        } else if (currentVelocity < -velLimit + 1e-7) {
            setJointVelocityRecursive(joint, -velLimit + 1e-7);
        }
    }

    gazebo::physics::LinkPtr childLink = joint->GetChild();
    for (auto & childJoint : childLink->GetChildJoints()) {
        snapVelocityRecursiveHelper(childJoint);
    }
}

GazeboObservationResultUniquePtr GazeboInterface::makeObservationReport(const GazeboObservationRequest* const observationRequest)
{
    boost::mutex::scoped_lock lock(odeMtx);
    GazeboWorldStatePtr currentWorldState = observationRequest->currentWorldState;
    std::string worldName = mainWorldName_;
    WorldPtr world_ = worldMap_[worldName];
    if (!world_) {
        ERROR("World '" + worldName + " not found");
    }

    world_->Reset();
    if (!currentWorldState) {
        WARNING("No current world state");
    } else if (*(lastConstructedWorldStates_[mainWorldName_].get()) != * (currentWorldState.get())) {
        setWorldState(currentWorldState);
    }

    setStateManuallyNew(observationRequest->currentStateVec, worldName);
    GazeboObservationResultUniquePtr observationResult(new GazeboObservationResult());
    observationResult->observationVec = std::vector<FloatType> (observationSpaceDimension_, 0);
    for (auto & observationFunction : getObservationFunctions_) {
        observationFunction(observationResult->observationVec, worldName);
    }

    return std::move(observationResult);
}

void GazeboInterface::setCollisionCheckedLinks(const VectorString& collisionCheckedLinks)
{
    collisionCheckedLinks_ = collisionCheckedLinks;
}

std::string GazeboInterface::getUnscopedName(const std::string& name) const
{
    std::string unscopedName = name;
    if (name.find("::") != std::string::npos) {
        VectorString nameElems;
        split(name, "::", nameElems);
        unscopedName = nameElems[nameElems.size() - 1];
    }

    return unscopedName;
}

void GazeboInterface::printWorldState()
{
    auto worldState = lastConstructedWorldStates_[mainWorldName_];
    sdf::ElementPtr sdfElem(new sdf::Element);
    sdf::initFile("world.sdf", sdfElem);
    sdf::ElementPtr stateElem = sdfElem->GetElement("state");
    worldState->getWorldState()->FillSDF(stateElem);
    const std::string pref = "";
    std::string sdfString = stateElem->ToString(pref);
    PRINT(sdfString);

}

void GazeboInterface::printWorldState(GazeboWorldStatePtr& worldState)
{
    sdf::ElementPtr sdfElem(new sdf::Element);
    sdf::initFile("world.sdf", sdfElem);
    sdf::ElementPtr stateElem = sdfElem->GetElement("state");
    worldState->getWorldState()->FillSDF(stateElem);
    const std::string pref = "";
    std::string sdfString = stateElem->ToString(pref);
    PRINT(sdfString);
}

WorldPtr GazeboInterface::getWorld() const
{
    return worldMap_.at(mainWorldName_);
}

void GazeboInterface::setWorldState(const GazeboWorldStatePtr& currentWorldState,
                                    const bool& applyDefPoseChanges)
{
    auto world = worldMap_[mainWorldName_];
    world->SetState(* (currentWorldState->getWorldState()));
    static_cast<gazebo::physics::OpptODEPhysics*>(world->GetPhysicsEngine().get())->setCumulativeAngles(
        currentWorldState->getCumulativeAngles());
    if (applyDefPoseChanges) {
        applyDeferredPoseChanges();
    }
}

void GazeboInterface::setStateVector(const VectorFloat& stateVector)
{
    setStateManuallyNew(stateVector, mainWorldName_);
}

const GazeboWorldStatePtr GazeboInterface::getWorldState(const bool &generateNewWorldState)
{    
    if (generateNewWorldState)
        makeWorldState();
    return lastConstructedWorldStates_.at(mainWorldName_);    
}

GazeboWorldStatePtr GazeboInterface::makeWorldState()
{
    auto world = worldMap_[mainWorldName_];
    GazeboWorldStatePtr worldState = std::make_shared<GazeboWorldState>(world);
    worldState->setCumulativeAngles(
        static_cast<gazebo::physics::OpptODEPhysics*>(world->GetPhysicsEngine().get())->getCumulativeAngles());
    lastConstructedWorldStates_[mainWorldName_] = worldState;
    return worldState;
}

void GazeboInterface::setBeforePhysicsUpdateFn(std::function<void()> beforePhysicsUpdateFn)
{
    beforePhysicsUpdateFn_ = beforePhysicsUpdateFn;
}


void GazeboInterface::setAfterPhysicsUpdateFn(std::function<void()> afterPhysicsUpdateFn)
{
    afterPhysicsUpdateFn_ = afterPhysicsUpdateFn;
}

GazeboPropagationResultUniquePtr GazeboInterface::doPropagation(const GazeboPropagationRequest* const propagationRequest)
{
    GazeboPropagationResultUniquePtr propagationResult(new GazeboPropagationResult());
    std::unique_ptr<ContactInformation> contactInformation(new ContactInformation());
    WorldPtr world_ = worldMap_[mainWorldName_];
    if (!world_) {
        ERROR("World '" + mainWorldName_ + " not found");
    }

    world_->GetPhysicsEngine()->Reset();
    size_t numIterations = propagationRequest->duration / world_->GetPhysicsEngine()->GetMaxStepSize();
    if (!propagationRequest->currentWorldState) {
        WARNING("State has no world state");
    } else {
        setWorldState(propagationRequest->currentWorldState, true);
    }

    setStateVector(propagationRequest->currentStateVec);
    world_->SetSimTime(lastSimTime_);

    auto jointEntries = worldJointMap_[mainWorldName_];
    auto linkEntries = worldRobotLinkMap_[mainWorldName_];

    bool breaking = false;
    gazebo::common::Time startTime = world_->GetSimTime();
    for (size_t i = 0; i != numIterations; ++i) {
        for (auto & actionFunction : applyActionFunctions_) {
            actionFunction(propagationRequest->actionVec, mainWorldName_);
        }

        for (auto & jointEntry : jointEntries) {
            jointEntry.second->ApplyStiffnessDamping();
        }

        if (propagationRequest->enableCollision) {
            if (!propagationResult->collisionReport || !(propagationResult->collisionReport->collides))
                propagationResult->collisionReport = collisionFunction_((void * const)(contactInformation.get()));
            if (propagationResult->collisionReport &&
                    propagationResult->collisionReport->collides &&
                    !propagationRequest->allowCollisions) {
                // Collisions are not allowed, so we can stop forward-propagating here
                breaking = true;
                break;
            }
        }

        if (beforePhysicsUpdateFn_)
            beforePhysicsUpdateFn_();

        world_->GetPhysicsEngine()->UpdatePhysics();
        for (auto & linkEntry : linkEntries) {
            linkEntry.second->SetWorldPose(linkEntry.second->GetDirtyPose(), false);
        }

        snapVelocityRecursive(mainWorldName_, false);
        if (afterPhysicsUpdateFn_)
            afterPhysicsUpdateFn_();

        startTime += world_->GetPhysicsEngine()->GetMaxStepSize();
        world_->SetSimTime(startTime);
        if (saveUserData_ && prefix_ == "exec") {
            auto nextStateVec = getState_(mainWorldName_);
            propagationResult->subStates.push_back(nextStateVec);
        }
    }

    // Need to do one last collision check
    if ((!propagationResult->collisionReport || propagationResult->collisionReport->collides) &&
            !breaking &&
            propagationRequest->enableCollision)
        propagationResult->collisionReport = collisionFunction_((void * const)(contactInformation.get()));

    lastSimTime_ = world_->GetSimTime();
    propagationResult->nextStateVec = getState_(mainWorldName_);
    propagationResult->nextWorldState = makeWorldState();
    lastConstructedWorldStates_[mainWorldName_] = propagationResult->nextWorldState;
    propagationResult->contactInformation = std::move(contactInformation);
    return std::move(propagationResult);
}

void GazeboInterface::setStateManuallyNew(const VectorFloat& currentStateVec, const std::string& worldName)
{
    for (size_t i = 0; i < setStateFunctions_.size(); i++) {
        setStateFunctions_[i](currentStateVec, worldName);
    }
}

void GazeboInterface::applyAction(const VectorFloat& actionVec, const std::string& worldName)
{
    for (size_t i = 0; i < applyActionFunctions_.size(); i++) {
        applyActionFunctions_[i](actionVec, worldName);
    }
}

}
