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
#include "SensorInterface.hpp"
#include "RaySensor.hpp"
#include "GPSSensor.hpp"
#include "DepthSensor.hpp"

namespace oppt
{
SensorInterface::SensorInterface():
    worldSensorMap_()
{

}

void SensorInterface::init(const std::string& worldName)
{
    gazebo::sensors::SensorManager* mgr =
        gazebo::sensors::SensorManager::Instance();
    gazebo::sensors::Sensor_V sensors = mgr->GetSensors();
    VectorSensor scopedSensors;
    VectorSensor validScopedSensors;
    for (auto& sensor : sensors) {
        sensor->SetActive(false);
        if (sensor->WorldName() == worldName) {
            GazeboSensorPtr gazeboSensor;
            if (sensor->Type() == "ray") {
                gazeboSensor = std::make_shared<RaySensor>(sensor);
                //scopedSensors.push_back(gazeboSensor);
            } else if (sensor->Type() == "gps") {
                gazeboSensor = std::make_shared<GPSSensor>(sensor);                
            } else if (sensor->Type() == "depth") {
                gazeboSensor = std::make_shared<DepthSensor>(sensor);
            } else {
                ERROR("Sensor of type '" + sensor->Type() + "' not supported");
            }

            scopedSensors.push_back(gazeboSensor);
        }
    }

    for (auto& sensor : scopedSensors) {
        validScopedSensors.push_back(sensor);
    }

    worldSensorMap_[worldName] = validScopedSensors;
}

void SensorInterface::getObservationLimits(VectorFloat& lowerLimits, VectorFloat& upperLimits) const
{
    auto firstEntry = worldSensorMap_.begin();
    for (auto& sensor : firstEntry->second) {
        VectorFloat lowerLimitsSensor;
        VectorFloat upperLimitsSensor;
        sensor->getObservationLimits(lowerLimitsSensor, upperLimitsSensor);
        lowerLimits.insert(lowerLimits.end(), lowerLimitsSensor.begin(), lowerLimitsSensor.end());
        upperLimits.insert(upperLimits.end(), upperLimitsSensor.begin(), upperLimitsSensor.end());
    }
}

void SensorInterface::getCombinedObservation(VectorFloat& observation, const std::string& worldName)
{
    auto sensors = worldSensorMap_[worldName];
    for (auto& sensor : sensors) {
        VectorFloat observationSensor = sensor->getObservation();
        observation.insert(observation.end(), observationSensor.begin(), observationSensor.end());

    }
}


}

