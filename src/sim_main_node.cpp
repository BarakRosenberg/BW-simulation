/*
 * Main simulation file
 * This code subscribes to the GUI /target location topic
 * The turtlebot robot is controlled using /odom topic for feedback and /cmd_vel to control the robot speed.
 * The robot current location in GEO coordinates is published to the /robot_location topic
 */

#include "../include/sim_main_node.h"

using namespace std;

SimulationMainNode::SimulationMainNode() : nh_("~"), rate_(10) {
	// Initialize the robot's starting position
	current_location_.latitude = 32.072734;
	current_location_.longitude = 34.787465;

	// Starting position in UTM coordinates
	start_pos.x = 668714.23;
	start_pos.y = 3549895.46;

	// Subscribe to the target location topic
	target_location_sub_ = nh_.subscribe("/target_location", 1, &SimulationMainNode::targetLocationCallback, this);

	// Publish the robot's current location
	current_location_pub_ = nh_.advertise<sensor_msgs::NavSatFix>("/robot_location", 1);

	// Subscribe to the robot odometry
	odom_sub = nh_.subscribe("/odom", 1000, &SimulationMainNode::odomCallback, this);

	// Create a publisher for the velocity commands
	vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

	// Boolean flags init
	new_target = false;
	target_reached = true;

	// Motion parameters initialization
	double map_scale = 4.0;
	max_linear_vel = 3.6 * map_scale;
	approach_kp = 0.5;
	ang_vel_kp = 3.2 * map_scale;
	yaw_ang = 0.0;
	cur_vel = 0.0;

	// 177.6m in world = 22.5m in sim
	simScale = (177.6 / 22.5) * map_scale;

	// distance traveled in 1.0 seconds to ensure braking this is multiplied by 1.2
	brake_dist = max_linear_vel * 1.0 * 1.2;
}

// Destructor
SimulationMainNode::~SimulationMainNode() {
	vel_msg.angular.z = 0.0;
	vel_msg.linear.x = 0.0;
	vel_msg.linear.y = 0.0;
	vel_pub_.publish(vel_msg);
}


// /target_location topic callback
// Every target location published causes the robot to start moving to new target
void SimulationMainNode::targetLocationCallback(const geographic_msgs::GeoPoint::ConstPtr& target_msg) {
  new_target = true;
  target_reached = false;
  ROS_INFO("New target received");
  // Move the robot towards the target location
  moveRobot(target_msg);

  // Publish the updated current location
  current_location_pub_.publish(current_location_);
}


// Main motion function
// This function runs in a loop until target is reached or new target is set
void SimulationMainNode::moveRobot(const geographic_msgs::GeoPoint::ConstPtr& target_location) {
	new_target = false;

	// Convert positions to UTM to work in meters
	converter.CoordinateConverter::geoToUtm(current_location_.latitude, current_location_.longitude, cur_pos.x, cur_pos.y);
	converter.CoordinateConverter::geoToUtm(target_location->latitude, target_location->longitude, tar_pos.x, tar_pos.y);

	ROS_INFO("Current pos in UTM: %f, %f", cur_pos.x, cur_pos.y);
	ROS_INFO("Target pos in UTM: %f, %f", tar_pos.x, tar_pos.y);

	double target_orientation = atan2((tar_pos.y - cur_pos.y), (tar_pos.x - cur_pos.x));
	ROS_INFO("Current orientation: %f", yaw_ang);
	ROS_INFO("Target orientation: %f", target_orientation);

	double distance = converter.utmDist(cur_pos, tar_pos);
	ROS_INFO("Distance to target in meters: %f", distance);

	double angle_err = 0.0;
	while (!new_target && !target_reached) {
		// Init cmd_vel message velocities
		vel_msg.linear.x = 0.0;
		vel_msg.angular.z = 0;

		// Calculate heading error and distance to target
		target_orientation = atan2((tar_pos.y - cur_pos.y), (tar_pos.x - cur_pos.x));
		angle_err = AngleDifference(target_orientation, yaw_ang);
		distance = converter.utmDist(cur_pos, tar_pos);

		// Rotate robot as long as the heading error is larger than 0.05 rad
		if (abs(angle_err) > 0.05){
			if (abs(angle_err) < 0.8){
				vel_msg.angular.z = -angle_err * ang_vel_kp;
			}
			else{ 		// If the heading error is too large rotate the robot fast in its place
				vel_msg.angular.z = -angle_err * ang_vel_kp * 2.0;
			}
		}
		else{
			vel_msg.angular.z = 0;
		}
//		ROS_INFO("angle_err: %f, ang_vel: %f", angle_err, vel_msg.angular.z);

		// Calculate desired linear velocity
		double temp_lin_vel = 0.0;
		if (abs(angle_err) <= 0.6){		// If the heading error is under 0.5 rad begin moving forward
			temp_lin_vel = max_linear_vel;
		}

		// Check linear distance to avoid overshoot
		if (distance > brake_dist){
			vel_msg.linear.x = temp_lin_vel;
		}
		else{	// Target reached
			vel_msg.linear.x = approach_kp * distance;
		}

		if (distance <= 1.0){
			vel_msg.linear.x = 0.0;
			vel_msg.angular.z = 0.0;

			if (abs(cur_vel) <= 0.1){
				target_reached = true;
				ROS_INFO("\n\nTarget reached");
				ROS_INFO("cur_pos: [%f, %f], distance: %f", current_location_.latitude, current_location_.longitude, distance);
			}
		}

//		ROS_INFO("cur_vel: %f, ang_vel: %f, distance: %f", vel_msg.linear.x, vel_msg.angular.z, distance);
		vel_msg.linear.x /= simScale;
		vel_msg.angular.z /= simScale;

		vel_pub_.publish(vel_msg);

		ros::spinOnce();
		rate_.sleep();
	}

	vel_msg.angular.z = 0.0;
	vel_msg.linear.x = 0.0;
	vel_msg.linear.y = 0.0;
	vel_pub_.publish(vel_msg);
}


// Callback function for the odometry data
void SimulationMainNode::odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
	// Get the orientation quaternion
	tf::Quaternion q(
		msg->pose.pose.orientation.x,
		msg->pose.pose.orientation.y,
		msg->pose.pose.orientation.z,
		msg->pose.pose.orientation.w);

	// Convert the quaternion to a rotation matrix
	tf::Matrix3x3 rot_mat(q);

	// Declare variables for roll, pitch and yaw angles
	double roll, pitch;

	// Get the roll, pitch and yaw angles from the rotation matrix
	rot_mat.getRPY(roll, pitch, yaw_ang);
	//	ROS_INFO("Yaw: [%f]", yaw_ang);

	double robot_x = simScale * msg->pose.pose.position.x;
	double robot_y = simScale * msg->pose.pose.position.y;

	cur_pos.x = start_pos.x + robot_x;
	cur_pos.y = start_pos.y + robot_y;

	cur_vel = simScale * msg->twist.twist.linear.x;

	// Convert new current position to GEO to publish
	converter.CoordinateConverter::utmToGeo(cur_pos.x, cur_pos.y, current_location_.latitude, current_location_.longitude);
	current_location_pub_.publish(current_location_);

	// Print the position and velocity
//	ROS_INFO("Position: [%f, %f]", robot_x, robot_y);
//	ROS_INFO("Velocity: [%f, %f, %f]", msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
}


// Calculate shortest distance between two angles
double SimulationMainNode::AngleDifference(double angle1, double angle2)
{
	double diff = fmod((angle2 - angle1 + M_PI), (2 * M_PI)) - M_PI;
	return (diff < -M_PI) ? (diff + (2 * M_PI)) : diff;
}


int main(int argc, char** argv) {
	ROS_INFO("Starting main node:");
	ros::init(argc, argv, "sim_main_node");
	SimulationMainNode sim_node;
	ros::spin();
	return 0;
}


