#include <iostream>
#include <sstream>
#include <string>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "robot_driver.h"

#define ANGULAR_MAX_SPEED 0.4
#define LINEAR_MAX_SPEED 0.3

int captureIntValue(std::string value_tag, int min_value, int max_value){
	int number = 0;
	std::string str;
	while(true){
		std::cout << "Enter a number between " << min_value << " and " << max_value << " to set the "<< value_tag <<" of the TurtleBot\'s pathway: " << std::endl;
		std::getline(std::cin,str);
		std::stringstream ss(str);
		if(ss >> number){
			if(number<=max_value && number>=min_value)
				return number;
		}
		std::cout << "Enter a valid number between " << min_value << " and " << max_value << "!" << std::endl;
	}
}

double captureFloatValue(std::string value_tag, double min_value, double max_value){
	double number = 0;
	std::string str;
	while(true){
		std::cout << "Enter a number between " << min_value << " and " << max_value << " to set the "<< value_tag <<" of the TurtleBot\'s pathway: " << std::endl;
		std::getline(std::cin,str);
		std::stringstream ss(str);
		if(ss >> number){
			if(number<=max_value && number>=min_value)
				return number;
		}
		std::cout << "Enter a valid number between " << min_value << " and " << max_value << "!" << std::endl;
	}
}


int captureTurtleSettings(double &linear, double &angular,int &edges_number, double &edges_length){
	char cmd[50];
	while(true){
		linear = captureIntValue("Linear speed",0,100);
		angular = captureIntValue("Angular speed",0,100);
		edges_number = captureIntValue("Number of edges of the polygon",3,10);
		edges_length = captureFloatValue("Length of edges of the polygon", 0.1, 5.0);
		
		std::cout << "TurtleBot is ready to go!" << std::endl;
		std::cout << "" << std::endl;
		std::cout << "Linear speed: " << linear << std::endl;
		std::cout << "Angular speed: " << angular << std::endl;
		std::cout << "Polygon's number of edges: " << edges_number << std::endl;
		std::cout << "Polygon's edges' length: " << edges_length << std::endl;
		std::cout << "" << std::endl;
		std::cout << "Are this setting correct? Can we unleash the turtlebotsaurus? Y/N" << std::endl;
		std::cin.getline(cmd,50);
		if(cmd[0]!='Y' && cmd[0]!='N'){
			std::cout << "Unknown command: " << cmd[0] << std::endl;
			std::cout << "Only keys Y or N are valid commands." << std::endl;
			std::cout << "" << std::endl;
		}
		if(cmd[0]!='Y'){
			linear = linear*LINEAR_MAX_SPEED/100;
			angular = angular*ANGULAR_MAX_SPEED/100;		
			std::cout << "Let's roll!" << std::endl;
			break;
		}
	}

}


int main(int argc, char** argv){
	ros::init(argc,argv,"teleop_control");
	ros::NodeHandle nh;
	RobotDriver driver(nh);

	//default settings to draw a square.
	double angular_speed = ANGULAR_MAX_SPEED/2;
	double linear_speed = LINEAR_MAX_SPEED/2;
	int edges_number = 4;
	double edges_length = 1;

	std::cout << "Welcome to tutlebot_driver" << std::endl;


	std::cout << "                        _____________  " << std::endl; 
	std::cout << "                       /             \\ " << std::endl;
	std::cout << "                       | Let\'s roll! | "<< std::endl;
	std::cout << "     _________     __  \\  ___________/ " << std::endl;
	std::cout << "    /_|__|__|_\\   / -\\  \\/             " << std::endl;
	std::cout << "   /__|__|__|__\\  \\__<                " << std::endl;
	std::cout << "o_/___|__|__|___\\_|                    " << std::endl;
	std::cout << "   ())      ())                         " << std::endl;
	std::cout << "" << std::endl;
	std::cout << "" << std::endl;
	std::cout << "This program intends to control Gazebo\'s TurtleBot and drive it in a polygonal shaped pathway." << std::endl;
	std::cout << "" << std::endl;
	

	captureTurtleSettings(linear_speed, angular_speed, edges_number, edges_length);
	driver.setSpeed(linear_speed, angular_speed);
	driver.driveOnAPolygonPath(edges_number, edges_length);

	return 0;
}

