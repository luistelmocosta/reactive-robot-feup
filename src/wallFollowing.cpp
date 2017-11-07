# include "../include/wallFollowing.h"

using namespace std;

namespace reactive_robot_feup
{
	WallFollowing::WallFollowing(int argc, char **argv)
	{
		if(argc != 3)
		{
			ROS_ERROR(
			"Usage : stdr_wall following <robot_frame_id> <laser_frame_id>");
			exit(0);
		}

		laser_topic = string("/") + string(argv[1]) + string("/") + string(argv[2]);
		speeds_topic = string("/") + string(argv[1]) + string("/cmd_vel");

		subscriber = n.subscribe(laser_topic.c_str(), 1, &WallFollowing::callback, this);

		cmd_vel_pub = n.advertise<geometry_msgs::Twist>(speeds_topic.c_str(), 1);
	}


	WallFollowing::~WallFollowing(void){}

  	void WallFollowing::callback(const sensor_msgs::LaserScan& msg)
	{
	    scan = msg;
    	geometry_msgs::Twist cmd;
    	
    	float min_distance_right_fan = FLT_MAX; //-100 to 0
    	float min_distance_left_fan = FLT_MAX; //0 to 100
    	float min_distance_front = FLT_MAX; // -15 to 15
    	float min_distance_right = FLT_MAX; // -100 to -85
    	float min_distance_left = FLT_MAX; // 85 to 100
    	float min_distance = 0.0;
    	float robot_angle = 0.0;
    	float alpha = 0.0;
    	
    	//read the info from the laser sensor
    	for(int i = 0; i < scan.ranges.size(); i++)
    	{
			float distance = scan.ranges[i];
			float sensor_angle = -100.0 + rTd(scan.angle_increment) * i;
			if(sensor_angle >= -100 && sensor_angle <= 0) //right side
			{
				if(distance < min_distance_right_fan)
				{
					min_distance_right_fan = distance;
					robot_angle = sensor_angle;
					if(sensor_angle >= -100 && sensor_angle <= -85)
					{
						if(distance < min_distance_right)
						{
							min_distance_right = distance;
						}
					}
				}
			}
			else if(sensor_angle > 0 && sensor_angle <= 100) // left side
			{
				if(distance < min_distance_left)
				{
					min_distance_left_fan = distance;
					robot_angle = sensor_angle;
					if(sensor_angle <= 100 && sensor_angle >= 85)
					{
						if(distance < min_distance_left)
						{
							min_distance_left = distance;
						}
					}
				}	
			} 
			
			if(sensor_angle >= -15 && sensor_angle <= 15)//front
			{
				if(distance < min_distance_front)
				{
					min_distance_front = distance;
				}
			}
		}
		
		min_distance = min(min_distance_left_fan, min_distance_right_fan);
		alpha = 90.0 - abs(robot_angle);
		
		if(min_distance <= scan.range_max) // within range of sensors
		{	
			cout << "DISTANCE TO WALL: " << min_distance << endl;
			cout << "ALPHA: " << alpha << endl;
	
			cmd.linear.x = 0.4;
			//https://www.seas.upenn.edu/sunfest/docs/papers/12-bayer.pdf
			cmd.angular.z = (-20 * (sin(dTr(alpha)) - (min_distance - 1.46))) * cmd.linear.x;
		}
		else //initial movement
		{
			cmd.linear.x = 0.3;
			cmd.angular.z = 0.1;
		}
		
		if(min_distance_front <= 1.5 && min_distance_left < scan.range_max && min_distance_right < scan.range_max) // on the pipe
		{
			cout << "STOPPED" << endl;
			cout << "FRONT DISTANCE: " << min_distance_front << endl;
			cout << "LEFT DISTANCE: " << min_distance_left << endl;
			cout << "RIGHT DISTANCE: " << min_distance_right << endl;
			cmd.linear.x = 0.0;
			cmd.angular.z = 0.0;
		}
    	
		cmd_vel_pub.publish(cmd);
	}

	float WallFollowing::rTd(float radian)
	{
		return 	radian * (180 / 3.1415);
	}

	float WallFollowing::dTr(float degree)
	{
		return degree * (3.1415 / 180);
	}
}






