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

#include <exception>
#include <utility>

#include <geodesy/utm.h>
#include <geographic_msgs/GeoPoint.h>


namespace util_geo_coordinates {

class CoordinateTransform {
public:
    CoordinateTransform(double latInit, double lonInit);

    std::pair<double, double> ll2xy(double lat, double lon) const;
    std::pair<double, double> xy2ll(double x, double y) const;

private:
    uint8_t zone_; ///< UTM longitude zone number
    char band_;    ///< MGRS latitude band letter
};

inline CoordinateTransform::CoordinateTransform(double latInit, double lonInit) {

    geographic_msgs::GeoPoint geoOrigin;
    geoOrigin.latitude = latInit;
    geoOrigin.longitude = lonInit;
    geoOrigin.altitude = 0.0;

    geodesy::UTMPoint utmOrigin;
    geodesy::fromMsg(geoOrigin, utmOrigin);

    zone_ = utmOrigin.zone;
    band_ = utmOrigin.band;
}

inline std::pair<double, double> CoordinateTransform::ll2xy(double lat, double lon) const {

    geographic_msgs::GeoPoint geoPoint;
    geoPoint.latitude = lat;
    geoPoint.longitude = lon;
    geoPoint.altitude = 0.0;

    geodesy::UTMPoint utmPoint;
    geodesy::fromMsg(geoPoint, utmPoint);

    if (utmPoint.zone != zone_) {
        throw std::range_error("You have left the UTM zone!");
    }
    if (utmPoint.band != band_) {
        throw std::range_error("You have left the UTM band!");
    }

    double x = utmPoint.easting;
    double y = utmPoint.northing;

    return std::make_pair(x, y);
}

inline std::pair<double, double> CoordinateTransform::xy2ll(double x, double y) const {

    geodesy::UTMPoint utmPoint{x, y, zone_, band_};
    geographic_msgs::GeoPoint geoPoint = geodesy::toMsg(utmPoint);

    double lat = geoPoint.latitude;
    double lon = geoPoint.longitude;

    ll2xy(lat, lon); // zone compliance check

    return std::make_pair(lat, lon);
}
}
