#ifndef SIM_MAIN_NODE_H
#define SIM_MAIN_NODE_H

#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "geographic_msgs/GeoPoint.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include <tf/transform_datatypes.h>
#include <cmath>
#include "../include/coordinate_converter.h"

class SimulationMainNode {
public:
  SimulationMainNode();
  ~SimulationMainNode();

  void targetLocationCallback(const geographic_msgs::GeoPoint::ConstPtr& target_msg);
  void moveRobot(const geographic_msgs::GeoPoint::ConstPtr& target_location);
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  double AngleDifference(double angle1, double angle2);

private:
  CoordinateConverter converter;
  ros::NodeHandle nh_;
  ros::Subscriber target_location_sub_;
  ros::Subscriber odom_sub;
  ros::Publisher current_location_pub_;
  sensor_msgs::NavSatFix current_location_;
  ros::Publisher vel_pub_;
  ros::Rate rate_;
  geometry_msgs::Twist vel_msg;
  geometry_msgs::Point start_pos;
  geometry_msgs::Point cur_pos;
  double yaw_ang;
  geometry_msgs::Point tar_pos;
  bool new_target;
  bool target_reached;
  double max_linear_vel;
  double approach_kp;
  double ang_vel_kp;
  double cur_vel;
  double simScale;
  double brake_dist;
};


#endif  // SIM_MAIN_NODE_H
