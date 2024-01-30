/*
 * coordinate_converter.cpp
 *
 *  Created on: 27 בינו׳ 2024
 *      Author: owner
 */

#include "../include/coordinate_converter.h"


CoordinateConverter::CoordinateConverter() {}

CoordinateConverter::~CoordinateConverter() {}

void CoordinateConverter::geoToUtm(double lat, double lon, double& easting, double& northing) {
  // Convert geographic to UTM
	geo_pt_.latitude = lat;
	geo_pt_.longitude = lon;
	geo_pt_.altitude = 0.0;
	geodesy::UTMPoint utm_pt_(geo_pt_);
	easting = utm_pt_.easting;
	northing = utm_pt_.northing;
//	std::cout << "utm: " <<  easting << ", " << northing << std::endl;
}

void CoordinateConverter::utmToGeo(double easting, double northing, double& lat, double& lon) {
  // Convert UTM to geographic (UTM params set to Israel)
  geodesy::UTMPoint utm_pt_(easting, northing, 0.0, 36, 'T');
  geo_pt_ = geodesy::toMsg(utm_pt_);
  lat = geo_pt_.latitude;
  lon = geo_pt_.longitude;
//  	std::cout << "geo: " <<  lat << ", " << lon << std::endl;

}

double CoordinateConverter::utmDist(geometry_msgs::Point p1, geometry_msgs::Point p2){
	// Simple euclidean distance
	double distance;

	double d1 = p2.x - p1.x;
	double d2 = p2.y - p1.y;

	distance = std::sqrt(d1 * d1 + d2 *d2);

	return distance;
}

// Function to calculate the Haversine distance between two points
double CoordinateConverter::geoDist(double lat1, double lon1, double lat2, double lon2) {
    const double R = 6371.0;  // Earth radius in kilometers

    // Convert latitude and longitude from degrees to radians
    lat1 = lat1 * M_PI / 180.0;
    lon1 = lon1 * M_PI / 180.0;
    lat2 = lat2 * M_PI / 180.0;
    lon2 = lon2 * M_PI / 180.0;

    // Differences in coordinates
    double dlat = lat2 - lat1;
    double dlon = lon2 - lon1;

    // Haversine formula
    double a = sin(dlat / 2) * sin(dlat / 2) + cos(lat1) * cos(lat2) * sin(dlon / 2) * sin(dlon / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));

    // Calculate distance
    double distance = R * c * 1000;
    return distance;
}


/*
 * Usage example:
 *
 * CoordinateConverter converter;
 *
 * double lat = 32.072734;
 * double lon = 34.787465;
 * double easting;
 * double northing;
 *
 * Convert GEO to UTM
 * converter.CoordinateConverter::geoToUtm(lat, lon, easting, northing);
 *
 * double new_lat;
 * double new_lon;
 * converter.CoordinateConverter::utmToGeo(easting, northing, new_lat, new_lon);
 *
 * double distance = converter.geoDist(lat1, lon1, lat2, lon2);
 *
 * double distance = converter.utmDist(p1, p2);
 *
 */


