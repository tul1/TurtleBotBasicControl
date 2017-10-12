#ifndef __ROBOT_DRIVER__
#define __ROBOT_DRIVER__

#define MATH_PI 3.141593
#define PUBLISH_RATE 100 //Hz
#define ROTATION_ANGLE_ERROR 0.5//degree

//DEBUG
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
		RobotDriver(ros::NodeHandle &nh);
		void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
		double getYawAngle();
		double calculateDistance(double x1, double y1, double x2, double y2);
		int moveForward(double distance);
		double calculateDestinationAngle(double start, double angle);
		int turnRight(double angle);
		void driveOnAPolygonPath(int edges, double edges_size);
		void setSpeed(double linear_speed, double angular_speed);
};

#endif

