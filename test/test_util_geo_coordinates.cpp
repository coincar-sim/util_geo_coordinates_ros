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

#include "gtest/gtest.h"

#include "util_geo_coordinates.hpp"

TEST(UtilGeoCoordinates, TestForwardConversion) {
    double latInit = 49.01439;
    double lonInit = 8.41722;
    util_geo_coordinates::CoordinateTransform cs(latInit, lonInit);
    double x, y;
    std::tie(x, y) = cs.ll2xy(latInit, lonInit);
    ASSERT_NEAR(x, 457386.38238563854, 0.001);
    ASSERT_NEAR(y, 5429219.051147663, 0.001);
}

TEST(UtilGeoCoordinates, TestBackwardConversion) {
    double latInit = 49.01439;
    double lonInit = 8.41722;
    util_geo_coordinates::CoordinateTransform cs(latInit, lonInit);

    double x = 457386.38238563854;
    double y = 5429219.051147663;
    double lat, lon;
    std::tie(lat, lon) = cs.xy2ll(x, y);
    ASSERT_NEAR(lat, 49.01439, 0.00001);
    ASSERT_NEAR(lon, 8.41722, 0.00001);
}

TEST(UtilGeoCoordinates, TestOutOfHemisphere) {
    double latInit = 49.01439;  // northern hemisphere
    double lonInit = 8.41722;
    util_geo_coordinates::CoordinateTransform cs(latInit, lonInit);

    double latTest = -latInit; // southern hemisphere
    ASSERT_THROW(cs.ll2xy(latTest, lonInit), std::runtime_error);
}

TEST(UtilGeoCoordinates, TestOutOfZone) {
    double latInit = 49.01439;
    double lonInit = 8.41722; // UTM zone 32
    util_geo_coordinates::CoordinateTransform cs(latInit, lonInit);

    double lonTest = 5.; // UTM zone 31
    ASSERT_THROW(cs.ll2xy(latInit, lonTest), std::runtime_error);
}

TEST(UtilGeoCoordinates, TestCompletelyOutOfZoneReverse) {
    double latInit = 49.01439;
    double lonInit = 8.41722; // UTM zone 32
    util_geo_coordinates::CoordinateTransform cs(latInit, lonInit);

    double x = 0.0;
    double y = 0.0;  // outside UTM zone 32
    ASSERT_THROW(cs.xy2ll(x, y);, std::runtime_error);
}

TEST(UtilGeoCoordinates, TestSlightlyOutOfZoneReverse) {
    double latInit = 49.01439;
    double lonInit = 8.41722; // UTM zone 32
    util_geo_coordinates::CoordinateTransform cs(latInit, lonInit);

    double x = 900000.0; // outside UTM zone 32
    double y = 5000000.0;
    ASSERT_THROW(cs.xy2ll(x, y);, std::runtime_error);
}
