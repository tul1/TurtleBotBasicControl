#include <iostream>
#include <cmath>


#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

#include <tf/transform_datatypes.h>


class RobotDriver {
	private:
		ros::NodeHandle nh_;
		ros::Publisher cmd_vel_pub_;
		ros::Subscriber odom_sub_;
		
		double yaw_angle_;
		double position_[2];
	public:

		RobotDriver(ros::NodeHandle &nh){
			nh_ = nh;
			cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity",1);
			odom_sub_ = nh_.subscribe("odom", 1, &RobotDriver::odomCallback, this);
		}

		void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
			tf::Pose pose;
			tf::poseMsgToTF(msg->pose.pose, pose);
			yaw_angle_ = tf::getYaw(pose.getRotation()) * 180 / 3.141593;
			position_[0] = msg->pose.pose.position.x;
			position_[1] = msg->pose.pose.position.y;
		}

		double* getPositionX(){
			return position_;
		}

		double getYawAngle(){
			return yaw_angle_;
		}

		double calculateDistance(double x1, double y1, double x2, double y2) {
			return std::sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
		}

		void moveForward(double distance, double speed) {
			geometry_msgs::Twist base_cmd;
			ros::spinOnce();
			double start_position_x = position_x_;
			double start_position_y = position_y_;
			
			while( calculateDistance(start_position_x, start_position_y, position_x_,position_y_) < distance ){
				base_cmd.linear.x = speed;
				cmd_vel_pub_.publish(base_cmd);

				std::cout<< "x: " << position_x_ << "y: " << position_y_ <<"\n";
				std::cout<< "calculated distance: " << calculateDistance(start_position_x, start_position_y, position_x_,position_y_) <<"\n";

				ros::spinOnce();
			}
	
			base_cmd.linear.x = 0;
			cmd_vel_pub_.publish(base_cmd);
		}

		double calculateDestinationAngle(double start, double angle){
			if((start+angle)>180)
				return (start+angle-360);
			if((start+angle)<-180)
				return (start+angle+360);
			return (start+angle);
		}

		void turnRight(double angle, double speed){			
			geometry_msgs::Twist base_cmd;
			float destination_yaw_angle = computeDestinationAngle(yaw_angle_,angle);

			ros::spinOnce();
			while( std::abs(yaw_angle_) < angle){

				std::cout << "destination: "<< destination_yaw_angle << "  Start angle: " << yaw_angle_ <<"\n"; 

				base_cmd.angular.z = - std::abs(speed);
				cmd_vel_pub_.publish(base_cmd);
				ros::spinOnce();
			}

			base_cmd.angular.z = 0.0;
			cmd_vel_pub_.publish(base_cmd);
		}


		void driveOnAPolygonPath(int edges, double edges_size, double linear_speed, double angular_speed) {
			if(edges<3){
				std::cout<<"A polygone have at least 3 edges. Please enter the correct number of edges.\n";
				return;
			}
			double polygon_internal_angle = (edges-2)*180/edges;
			int i;	
			for(i=0; i<edges; i++){
				moveForward(edge_size, linear_speed);
				turnRight(polygon_internal_angle, angular_speed);
			}
		}


};


int main(int argc, char** argv){
	ros::init(argc,argv,"teleop_control");
	ros::NodeHandle nh;

	RobotDriver driver(nh);

	double angular_speed = 0.3;
	double linear_speed = 0.2;

	while(nh_.ok())
		driver.driveOnAPolygonPath(4, 2, linear_speed, angular_speed);

}
