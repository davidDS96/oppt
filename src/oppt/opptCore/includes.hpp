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
#ifndef __INCLUDES_HPP__
#define __INCLUDES_HPP__
#include <vector>
#include <iostream>
#include <memory>
#include <functional>

#include "fcl/BVH/BVH_model.h"
#include "fcl/BV/BV.h"
#include "fcl/collision_object.h"
#include "fcl/shape/geometric_shapes.h"
#include "fcl/shape/geometric_shapes_utility.h"
#include "fcl/collision.h"
#include "fcl/continuous_collision.h"
#include "fcl/broadphase/broadphase.h"
#include "fcl/collision_data.h"
#include <boost/filesystem.hpp>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry> 
#include <tinyxml.h>

#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/common/CommonIface.hh>
#include <gazebo/sensors/SensorsIface.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/gazebo_client.hh>

#endif
