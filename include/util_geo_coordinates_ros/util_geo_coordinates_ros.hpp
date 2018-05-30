/*
 * Copyright (c) 2017
 * FZI Forschungszentrum Informatik, Karlsruhe, Germany (www.fzi.de)
 * KIT, Institute of Measurement and Control, Karlsruhe, Germany (www.mrt.kit.edu)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

#include "util_geo_coordinates.hpp"

namespace util_geo_coordinates {

class CoordinateTransformRosTest;

class CoordinateTransformRos {
    friend class CoordinateTransformRosTest;

public:
    CoordinateTransformRos();
    void waitForInit(const std::string& originTopic, double pollRate);

    std::pair<double, double> ll2xy(double lat, double lon) const;
    std::pair<double, double> xy2ll(double x, double y) const;

    std::string getOriginFrameId() const;
    std::pair<double, double> getOriginLatLon() const;

private:
    void initializeCoordinateTransform(const sensor_msgs::NavSatFix::ConstPtr& msg);

    ros::Subscriber originSubscriber_;
    std::shared_ptr<CoordinateTransform> coordinateTransformPtr_;
    bool initialized_{false};
    std::string originFrameId_;
    double originLat_, originLon_;
};

inline CoordinateTransformRos::CoordinateTransformRos() {

}

inline void CoordinateTransformRos::waitForInit(const std::string& originTopic, double pollRate) {
    ros::NodeHandle nh;
    originSubscriber_ = nh.subscribe(originTopic, 5, &CoordinateTransformRos::initializeCoordinateTransform, this);
    ros::Rate rate(pollRate);
    while (!initialized_ && ros::ok()) {
        ros::spinOnce();
        ROS_INFO_THROTTLE(3,
                          "CoordinateTransformRos: Waiting for initialization, listening on topic \"%s\"...",
                          originSubscriber_.getTopic().c_str());
        rate.sleep();
    }
}

inline void CoordinateTransformRos::initializeCoordinateTransform(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    originSubscriber_.shutdown();
    originLat_ = msg->latitude;
    originLon_ = msg->longitude;
    coordinateTransformPtr_ = std::make_shared<CoordinateTransform>(originLat_, originLon_);
    originFrameId_ = msg->header.frame_id;
    initialized_ = true;
}


inline std::pair<double, double> CoordinateTransformRos::ll2xy(double lat, double lon) const {
    if (!initialized_)
        throw std::runtime_error("CoordinateTransform not initialized!");
    return coordinateTransformPtr_->ll2xy(lat, lon);
}

inline std::pair<double, double> CoordinateTransformRos::xy2ll(double x, double y) const {
    if (!initialized_)
        throw std::runtime_error("CoordinateTransform not initialized!");
    return coordinateTransformPtr_->xy2ll(x, y);
}

inline std::string CoordinateTransformRos::getOriginFrameId() const {
    if (!initialized_)
        throw std::runtime_error("CoordinateTransform not initialized!");
    return originFrameId_;
}

inline std::pair<double, double> CoordinateTransformRos::getOriginLatLon() const {
    if (!initialized_)
        throw std::runtime_error("CoordinateTransform not initialized!");
    return std::make_pair(originLat_, originLon_);
}
}
