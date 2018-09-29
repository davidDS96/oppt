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
#include "oppt/opptCore/geometric/Cylinder.hpp"

namespace oppt
{
namespace geometric
{
Cylinder::Cylinder(const std::string& name,
                   const FloatType& radius,
                   const FloatType& length,
                   oppt::Matrixdf& pose):
    Geometry(CYLINDER, name, pose),
    radius_(radius),
    length_(length)
{    
    createCollisionGeometry();
}

Cylinder::Cylinder():
    Geometry(CYLINDER)
{

}

GeometrySharedPtr Cylinder::copy() const
{
    std::string name = name_;
    FloatType radius = radius_;
    FloatType length = length_;    
    Matrixdf pose(pose_);
    GeometrySharedPtr copiedGeometry(new Cylinder(name, radius, length, pose));
    return copiedGeometry;
}

std::shared_ptr<geometric::Geometry> Cylinder::shallowCopy() const
{
    GeometrySharedPtr copiedGeometry(new Cylinder());
    auto cylinder = static_cast<Cylinder*>(copiedGeometry.get());
    std::string name = name_;
    Matrixdf pose(pose_);    
    copiedGeometry->setName(name);
    copiedGeometry->setPose(pose);
    copiedGeometry->setColor(getColor());
    cylinder->radius_ = radius_;
    cylinder->length_ = length_;    
    return copiedGeometry;
}

FloatType Cylinder::getRadius() const
{
    return radius_;
}

FloatType Cylinder::getLength() const
{
    return length_;
}

std::string Cylinder::toSDFString() const
{
    std::string serString = "";
    serString += "<geometry>";
    serString += "<cylinder>";
    serString += "<radius>";
    serString += std::to_string(radius_);
    serString += "</radius>";
    serString += "<length>";
    serString += std::to_string(length_);
    serString += "</length>";
    serString += "</cylinder>";
    serString += "</geometry>";
    return serString;
}

void Cylinder::createCollisionGeometry()
{
    collisionGeometry_ = CollisionGeometrySharedPtr(new fcl::Cylinder(radius_, length_));
}

}
}
