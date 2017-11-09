#include "../include/wallFollowing.h"
#include <iostream>
#include <fstream>
#include <string>
#include <time.h>
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

    	char filename[ ] = "Results.csv";
     	fstream appendFileToWorkWith;

     	appendFileToWorkWith.open(filename, std::fstream::in | std::fstream::out | std::fstream::app);




    	
    	
    	//read the scan from the sensors
    	for(int i = 0; i < scan.ranges.size(); i++)
    	{
			float distance = scan.ranges[i];
			float sensor_angle = -100.0 + rTd(scan.angle_increment) * i;

			/*
			RIGHT SENSOR
			*/
			if(sensor_angle >= -100 && sensor_angle <= 0) 
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

			/*
			LEFT SENSOR
			*/
			else if(sensor_angle > 0 && sensor_angle <= 100) 
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

			/* 
			FRONT SENSOR 
			*/
			if(sensor_angle >= -15 && sensor_angle <= 15)
			{
				if(distance < min_distance_front)
				{
					min_distance_front = distance;
				}
			}
		}


		
		min_distance = min(min_distance_left_fan, min_distance_right_fan);
		alpha = 75.0 - abs(robot_angle);
		
		if(min_distance <= scan.range_max) // within range of sensors
		{

			if (!appendFileToWorkWith ) 
			{
      			cout << "Cannot open file, file does not exist. Creating new file..";

        		appendFileToWorkWith.open(filename,  fstream::in | fstream::out | fstream::trunc);
        		appendFileToWorkWith <<"\n";
        		appendFileToWorkWith.close();
        	}
        	else

      			{    // use existing file
         			cout<<"success "<<filename <<" found. \n";
         			cout<<"\nAppending writing and working with existing file"<<"\n---\n";

         			appendFileToWorkWith << min_distance << "," << alpha << "\n";
         			appendFileToWorkWith.close();
         			cout<<"\n";

    


			
			cout << "DISTANCE TO WALL: " << min_distance << endl;
			cout << "ALPHA: " << alpha << endl;
			//https://www.seas.upenn.edu/sunfest/docs/papers/12-bayer.pdf

			cmd.linear.x = 0.4;
			cmd.angular.z = (-20 * (sin(dTr(alpha)) - (min_distance - 1.46))) * cmd.linear.x;
		}
		}
		else //initial movement
		{
			cmd.linear.x = 0.3;
			cmd.angular.z = 0.1;
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






