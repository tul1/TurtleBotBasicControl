#include <iostream>
#include <cmath>
#include <tf/transform_datatypes.h>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "robot_driver.h"

RobotDriver::RobotDriver(ros::NodeHandle &nh){
	nh_ = nh;
	cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity",1);
	odom_sub_ = nh_.subscribe("odom", 1, &RobotDriver::odomCallback, this);
	loop_rate_ = new ros::Rate(PUBLISH_RATE);
}

void RobotDriver::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
	tf::Pose pose;
	tf::poseMsgToTF(msg->pose.pose, pose);
	yaw_angle_ = tf::getYaw(pose.getRotation()) * 180 / MATH_PI;
	position_[0] = msg->pose.pose.position.x;
	position_[1] = msg->pose.pose.position.y;
}

double RobotDriver::getYawAngle(){
	return yaw_angle_;
}

double RobotDriver::calculateDistance(double x1, double y1, double x2, double y2) {
	return std::sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
}

int RobotDriver::moveForward(double distance) {
	geometry_msgs::Twist base_cmd;
	ros::spinOnce();
	loop_rate_->sleep();
	double start_position_x = position_[0];
	double start_position_y = position_[1];

#ifdef DEBUG_MOVE_FORWARD
	std::cout << "----------------------- MOVING FORWARD -----------------------" << std::endl;			
	std::cout << "initial_x: " << start_position_x << "  initial_y: " << start_position_y << std::endl;
#endif

	while( nh_.ok() && calculateDistance(start_position_x, start_position_y, position_[0], position_[1]) <= distance ){
		base_cmd.linear.x = linear_speed_;
		cmd_vel_pub_.publish(base_cmd);

#ifdef DEBUG_MOVE_FORWARD
		std::cout << "initial_x: " << start_position_x << "  initial_y: " << start_position_y << "  actual_x: " << position_[0] << "  actual_y: " << position_[1] << std::endl;
		std::cout << "calculated_distance: " << calculateDistance(start_position_x,start_position_y,position_[0],position_[1]) << "  distance: " << distance << std::endl;
#endif

		ros::spinOnce();
		loop_rate_->sleep();
	}

	base_cmd.linear.x = 0.0;
	cmd_vel_pub_.publish(base_cmd);
	return 0;
}


double RobotDriver::calculateDestinationAngle(double start, double angle){
	if((start+angle)>180)
		return (start+angle-360);
	if((start+angle)<-180)
		return (start+angle+360);
	return (start+angle);
}


int RobotDriver::turnRight(double angle){			
	geometry_msgs::Twist base_cmd;
	ros::spinOnce();
	loop_rate_->sleep();
	double destination_yaw_angle = calculateDestinationAngle(yaw_angle_,angle);

#ifdef DEBUG_TURN_RIGHT
	std::cout << "----------------------- TURNING RIGHT -----------------------" << std::endl;			
	std::cout << "destination_angle: "<< destination_yaw_angle << "  actual_angle: " << yaw_angle_ << std::endl; 
#endif

	while(nh_.ok() && std::abs(yaw_angle_-destination_yaw_angle) > ROTATION_ANGLE_ERROR ){
		base_cmd.angular.z = std::abs(angular_speed_);
		cmd_vel_pub_.publish(base_cmd);
		
#ifdef DEBUG_TURN_RIGHT
		std::cout << "destination_angle: "<< destination_yaw_angle << "  actual_angle: " << yaw_angle_ << std::endl; 
#endif

		ros::spinOnce();
		loop_rate_->sleep();
	}

	base_cmd.angular.z = 0.0;
	cmd_vel_pub_.publish(base_cmd);
	return 0;
}


void RobotDriver::driveOnAPolygonPath(int edges, double edges_size) {
	if(edges<3){
		std::cout<<"A polygone have at least 3 edges. Please enter the correct number of edges."<<std::endl;
		return;
	}
	double polygon_internal_angle = 180 - (edges-2)*180/edges;
	for(int i=0; i<edges; i++){
		moveForward(edges_size);
		turnRight(polygon_internal_angle);
	}
}

void RobotDriver::setSpeed(double linear_speed, double angular_speed){
	linear_speed_ = linear_speed;
	angular_speed_ = angular_speed;
}
