#ifndef STDR_LINE_FOLLOWING
#define STDR_LINE_FOLLOWING

#include <iostream>
#include <cstdlib>
#include <cmath>
#include <ctime>

#include <ros/package.h>
#include "ros/ros.h"

#include <stdr_msgs/RobotIndexedVectorMsg.h>

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>

namespace reactive_robot_feup
{
	class WallFollowing
	{
	private:
		sensor_msgs::LaserScan scan;
		ros::Subscriber subscriber;
		ros::NodeHandle n;
		std::string laser_topic;
		std::string speeds_topic;
		ros::Publisher cmd_vel_pub;
		
		float rTd(float radian);
		float dTr(float degree);
		
	public:
		WallFollowing(int argc,char **argv);
		~WallFollowing(void);
		void callback(const sensor_msgs::LaserScan& msg);
  	};
}

#endif
