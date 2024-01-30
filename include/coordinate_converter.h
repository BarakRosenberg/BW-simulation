/*
 * coordinate_converter.h
 *
 *  Created on: 27 בינו׳ 2024
 *      Author: owner
 */

#ifndef CORD_CONV_H_
#define CORD_CONV_H_

#include "ros/ros.h"
#include <geodesy/utm.h>


class CoordinateConverter {
public:
  CoordinateConverter();
  ~CoordinateConverter();

  void geoToUtm(double lat, double lon, double& easting, double& northing);
  void utmToGeo(double easting, double northing, double& lat, double& lon);
  double utmDist(geometry_msgs::Point p1, geometry_msgs::Point p2);
  double geoDist(double lat1, double lon1, double lat2, double lon2);

private:
  geographic_msgs::GeoPoint geo_pt_;

};


#endif /* CORD_CONV_H_ */




