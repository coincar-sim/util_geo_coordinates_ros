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

#include <ros/ros.h>
#include "gtest/gtest.h"

#include "util_geo_coordinates_ros.hpp"


TEST(UtilGeoCoordinatesRos, TestForwardConversion) {
    util_geo_coordinates::CoordinateTransformRos csRos;
    csRos.waitForInit("/nav_sat_fix_topic", 1);
    double latInit = 49.01439;
    double lonInit = 8.41722;

    double x, y;
    std::tie(x, y) = csRos.ll2xy(latInit, lonInit);
    ASSERT_NEAR(457386.38238563854, x, 0.001);
    ASSERT_NEAR(5429219.051147663, y, 0.001);
}

TEST(UtilGeoCoordinatesRos, TestThrowNoInit) {
    util_geo_coordinates::CoordinateTransformRos csRos;
    // csRos.waitForInit(1);
    double latInit = 49.01439;
    double lonInit = 8.41722;

    ASSERT_THROW(csRos.ll2xy(latInit, lonInit), std::runtime_error);
}

TEST(UtilGeoCoordinatesRos, TestOriginGetters) {
    util_geo_coordinates::CoordinateTransformRos csRos;
    csRos.waitForInit("/nav_sat_fix_topic", 1);

    double latOrigin, lonOrigin;
    std::tie(latOrigin, lonOrigin) = csRos.getOriginLatLon();
    ASSERT_NEAR(49.01439, latOrigin, 0.001); // values from NavSatFix Message
    ASSERT_NEAR(8.41722, lonOrigin, 0.001);  // see .test-file
    ASSERT_STREQ(std::string("origin").c_str(), csRos.getOriginFrameId().c_str());
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "unittest");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
