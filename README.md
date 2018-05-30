# util_geo_coordinates_ros

Library providing the conversion between geodetic (latitude/ longitude) and Cartesian (easting x/ northing y) coordinates.

The library uses the [Universal Transverse Mercator (UTM) coordinate system](https://en.wikipedia.org/wiki/Universal_Transverse_Mercator_coordinate_system) for this purpose.

## Usage

### C++
* define the lat/lon origin yourself, using `util_geo_coordinates.hpp`
* use a ROS NavSatFix-message for defining the lat/lon origin across multiple nodes in your projects, using `util_geo_coordinates_ros.hpp`
  * define the topic of the NavSatFix-message
  * the NavSatFix-message must contain the lat/lon origin, and the frame_id that originates in this lat/lon origin

### Python
* define the lat/lon origin yourself, using the module `util_geo_coordinates_ros`, a boost_python interface to `util_geo_coordinates.hpp`

## License
This package is distributed under the 3-Clause BSD License, see [LICENSE](LICENSE).
