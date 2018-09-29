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
#ifndef _VIEWER_PUBLISHER_HPP_
#define _VIEWER_PUBLISHER_HPP_
#include <ros/ros.h>
#include "oppt/opptCore/core.hpp"
#include "oppt/opptCore/Viewer.hpp"
#include "SDFParser.hpp"
#include <std_msgs/Bool.h>

namespace oppt
{

class ViewerPublisher: public ViewerBase
{
public:
    /**
     * Default constructor
     */
    ViewerPublisher();

    /**
     * Default destructor
     */
    virtual ~ViewerPublisher();
 
    /**
     * Setup the viewer publisher to display the provided model and environment
     */
    bool setupViewer(std::string model_file,
                     std::string environment_file);

    bool drawGeometries(VectorGeometryPtr& geometries,
                        std::vector<std::vector<Matrixdf>>& poses,
			const VectorFloat &particleColor,
                        const bool& keepParticles = false,
                        const bool& deleteExistingParticles = false);

    void updateFromEnvironmentInfo(EnvironmentInfoSharedPtr& environmentInfo, bool forceReset = false);

    void displayText(const std::string& text);
    
    /**
     * Determine if the viewer is running
     */
    virtual bool viewerRunning() const override;

private:
    void removeMarkersWithName(const VectorString& names, SDFMarkersSharedPtr& markers, const bool& publish = false);

    void updateFromEnvironmentInfoImpl(EnvironmentInfoSharedPtr& environmentInfo, const bool& forceReset);

    void publishMarkers();

    void reset();

    void waitForDisplayAddedMessage(const std_msgs::BoolConstPtr& msg);
    
    void removeParticles(const bool &publish);

private:
    bool destroy_ = false;

    SDFMarkersSharedPtr environmentMarkers_;

    SDFMarkersSharedPtr robotMarkers_;

    SDFMarkersSharedPtr frameMarkers_;

    boost::recursive_mutex environmentMtx_;

    std::string baseFrame_;

    SDFParser sdfParser_;    

    bool displayAdded_ = true;

    ros::Publisher updateMarkersPublisher_;

    ros::Publisher setFixedFramePublisher_;

    ros::Publisher resetFramePublisher_;

    ros::Subscriber displayAddedSub_;

    bool foundROSEnvironment_;

};
}

#endif
