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
#ifndef _OPPT_USER_DATA_
#define _OPPT_USER_DATA_
#include "oppt/opptCore/includes.hpp"

namespace oppt
{

/**
 * Base class for any UserData object
 */
class OpptUserData
{
public:
    /** @brief Default constructor*/
    OpptUserData() = default;

    /** @brief Default virtual destructor */
    virtual ~OpptUserData() {};
};

/** @brief std::shared_ptr to oppt::OpptUserData*/
typedef std::shared_ptr<OpptUserData> OpptUserDataSharedPtr;

}

#endif
// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
