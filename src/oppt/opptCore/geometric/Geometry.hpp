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
#ifndef __OPPT_GEOMETRY_HPP__
#define __OPPT_GEOMETRY_HPP__
#include "oppt/opptCore/typedefs.hpp"
#include "oppt/opptCore/constants.hpp"

namespace oppt
{

/**
* @namespace oppt::geometric
* @brief Namespace for geometries
*/
namespace geometric
{

/**
 * Base class for geometries
 */
class Geometry
{
public:
    virtual ~Geometry() {}

    /**
     * @brief Get the name of the geometry
     */
    std::string getName() const
    {
        return name_;
    }

    /**
     * @brief Set the name of the geometry
     */
    void setName(const std::string& name)
    {
        name_ = name;
    }

    /**
     * @brief Set the pose of the geometry using a 4x4 transformation matrix
     */
    void setPose(oppt::Matrixdf& pose)
    {
        pose_ = pose;
    }

    /**
     * @brief Copy the geometry
     * @return A shared_ptr to the copied geometry
     */
    virtual std::shared_ptr<geometric::Geometry> copy() const = 0;

    /**
     * @brief Copy the geometry without constructing the collision object
     * @return A shared_ptr to the copied geometry
     */
    virtual std::shared_ptr<geometric::Geometry> shallowCopy() const = 0;

    /**
     * @brief Get the pose of the geometry
     */
    oppt::Matrixdf getPose() const
    {
        return pose_;
    }

    void setColor(const oppt::VectorFloat& color)
    {
        color_ = color;
    }

    oppt::VectorFloat getColor() const
    {
        return color_;
    }

    /**
     * @brief Get the GeometryType of this geometry
     */
    GeometryType getType() const
    {
        return type_;
    }

    /**
     * @brief Get a pointer to the CollisionGeometry for this geometry
     */
    CollisionGeometrySharedPtr getCollisionGeometry() const
    {
        return collisionGeometry_;
    }

    /**
     * @brief Get the SDF description of this geometry
     * @return A std::string containting the SDF description of this geometry
     */
    virtual std::string toSDFString() const = 0;

protected:
    std::string name_;

    oppt::Matrixdf pose_;

    oppt::VectorFloat color_;

    const unsigned int type_;

    CollisionGeometrySharedPtr collisionGeometry_;

    virtual void createCollisionGeometry() = 0;

    Geometry(const GeometryType& type, const std::string& name, oppt::Matrixdf& pose):
        type_(type),
        name_(name),
        pose_(pose),
        color_({0.5, 0.5, 0.5, 1.0}),
           collisionGeometry_(nullptr)
    {

    }

    Geometry(const GeometryType& type):
        type_(type),
        name_(""),
        pose_(),
        color_({0.5, 0.5, 0.5, 1.0})
    {

    }

};

};

typedef std::shared_ptr<geometric::Geometry> GeometrySharedPtr;
typedef std::vector<GeometrySharedPtr> VectorGeometryPtr;

};

#endif
