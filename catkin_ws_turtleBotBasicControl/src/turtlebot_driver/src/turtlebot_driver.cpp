#include <iostream>
#include <sstream>
#include <string>
#include <cmath>
#include <tf/transform_datatypes.h>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

//#define DEBUG_MOVE_FORWARD
//#define DEBUG_TURN_RIGHT

class RobotDriver {
	private:
		ros::NodeHandle nh_;
		ros::Publisher cmd_vel_pub_;
		ros::Subscriber odom_sub_;
		ros::Rate* loop_rate_;
		
		double yaw_angle_;
		double position_[2];

		double linear_speed_;
		double angular_speed_;

	public:

		RobotDriver(ros::NodeHandle &nh){
			nh_ = nh;
			cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity",1);
			odom_sub_ = nh_.subscribe("odom", 1, &RobotDriver::odomCallback, this);
			loop_rate_ = new ros::Rate(50);
		}

		void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
			tf::Pose pose;
			tf::poseMsgToTF(msg->pose.pose, pose);
			yaw_angle_ = tf::getYaw(pose.getRotation()) * 180 / 3.141593;
			position_[0] = msg->pose.pose.position.x;
			position_[1] = msg->pose.pose.position.y;
		}

		double getYawAngle(){
			return yaw_angle_;
		}

		double calculateDistance(double x1, double y1, double x2, double y2) {
			return std::sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
		}

		int moveForward(double distance) {
			geometry_msgs::Twist base_cmd;
			ros::spinOnce();
			loop_rate_->sleep();
			double start_position_x = position_[0];
			double start_position_y = position_[1];

#ifdef DEBUG_MOVE_FORWARD
			std::cout << "----------------------- MOVING FORWARD -----------------------" << std::endl;			
			std::cout << "initial_x: " << start_position_x << "  initial_y: " << start_position_y << std::endl;
#endif

			while( nh_.ok() && calculateDistance(start_position_x, start_position_y, position_[0], position_[1]) < distance ){
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

		double calculateDestinationAngle(double start, double angle){
			if((start+angle)>180)
				return (start+angle-360);
			if((start+angle)<-180)
				return (start+angle+360);
			return (start+angle);
		}

		int turnRight(double angle){			
			geometry_msgs::Twist base_cmd;
			ros::spinOnce();
			loop_rate_->sleep();
			double destination_yaw_angle = calculateDestinationAngle(yaw_angle_,angle);

#ifdef DEBUG_TURN_RIGHT
			std::cout << "----------------------- TURNING RIGHT -----------------------" << std::endl;			
			std::cout << "destination_angle: "<< destination_yaw_angle << "  actual_angle: " << yaw_angle_ << std::endl; 
#endif

			while(nh_.ok() && std::abs(yaw_angle_-destination_yaw_angle) > 0.5 ){
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


		void driveOnAPolygonPath(int edges, double edges_size) {
			if(edges<3){
				std::cout<<"A polygone have at least 3 edges. Please enter the correct number of edges."<<std::endl;
				return;
			}
			double polygon_internal_angle = (edges-2)*180/edges;
			for(int i=0; i<edges; i++){
				moveForward(edges_size);
				turnRight(polygon_internal_angle);
			}
		}

		void setSpeed(double linear_speed, double angular_speed){
			linear_speed_ = linear_speed;
			angular_speed_ = angular_speed;
		}

};


int captureSpeedValue(std::string speed_tag){
	int number = 0;
	std::string str;
	while(true){
		std::cout << "Enter a number between 0 and 100 to set the "<< speed_tag <<" speed of the TurtleBot: " << std::endl;
		std::getline(std::cin,str);
		std::stringstream ss(str);
		if(ss >> number){
			if(number<100)
				return number;
		}
		std::cout << "Enter a valid number between 0 and 100!" << std::endl;
	}
}

int captureTurtleSpeed(double &linear, double &angular){
	char cmd[50];
	while(true){
		linear = captureSpeedValue("Linear")*0.3/100;
		angular = captureSpeedValue("Angular")*0.4/100;

		std::cout << "TurtleBot is ready to move this fast:" << std::endl;
		std::cout << "Linear speed: " << linear << std::endl;
		std::cout << "Angular speed: " << angular << std::endl;
		
		std::cout << "Do you agree? Y/N" << angular << std::endl;
		std::cin.getline(cmd,50);
		if(cmd[0]!='Y' && cmd[0]!='N'){
			std::cout << "Unknown command: " << cmd[0] << std::endl;
			std::cout << "Only keys Y or N are valid commands." << std::endl;
		}
		if(cmd[0]!='Y')
			break;
	}

}

int main(int argc, char** argv){
	ros::init(argc,argv,"teleop_control");
	ros::NodeHandle nh;
	RobotDriver driver(nh);

	double angular_speed = 0.3;
	double linear_speed = 0.1;
	int drawing_cycles = 1;

	std::cout << "Welcome to tutlebot_driver" << std::endl;
	std::cout << "     _________     __  " << std::endl;
	std::cout << "    /_|__|__|_\\   / -\\ " << std::endl;
	std::cout << "   /__|__|__|__\\  \\__< " << std::endl;
	std::cout << "o_/___|__|__|___\\_|    " << std::endl;
	std::cout << "   ())      ())        " << std::endl;
	std::cout << "" << std::endl;


	captureTurtleSpeed(linear_speed,angular_speed);
	driver.setSpeed(linear_speed,angular_speed);

	for(int i=0; i<drawing_cycles; i++){
		std::cout << "Square path: " << std::endl;		
		driver.driveOnAPolygonPath(4, 1);
	}

	for(int i=0; i<drawing_cycles; i++){
		std::cout << "Triangle path: " << std::endl;
		driver.driveOnAPolygonPath(3, 1);
	}

	for(int i=0; i<drawing_cycles; i++){
		std::cout << "Pentagon path: " << std::endl;
		driver.driveOnAPolygonPath(5, 1);
	}

	return 0;
}

