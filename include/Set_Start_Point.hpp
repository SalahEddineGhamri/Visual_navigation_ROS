/***********************************************************************/
/*                         Set_Start_Point.hpp                         */
/***********************************************************************/
/* -VERSION: ROS_Ubuntu 14.04                                          */
/* -AUTHOR:  RUBIO Luc                                                 */
/* -LAST_MODIFICATION: 03/2017                                         */
/***********************************************************************/
#ifndef SET_START_POINT_H
#define SET_START_POINT_H

#include "Target.hpp"
#include "ros/ros.h"
#include <nav_msgs/Odometry.h>

#include <vector>
#include <fstream>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

using namespace std;

class Set_Start_Point
{
	private:
		ros::NodeHandle nh_;      // The node handle for Recherche
		ros::Subscriber sub_;     // Subscriber to odom topic
		nav_msgs::Odometry odom_; // The message to save odom informations
		int type_;                // The message to save the type of the Target (a start)
		int id_;                  // The id of the Target
			
	public:

		//Constructor
		Set_Start_Point(ros::NodeHandle nh);
		Set_Start_Point(){};

		//Destructor
		~Set_Start_Point(){};

		//Operator
		
		void CallbackreadOdom(const nav_msgs::Odometry Pos_goal);

		//Getter
		vector<float> Odom_Position();
		vector<float> Odom_Orientation();
		Target Create_target_start(int id);
		
};


#endif //SET_START_POINT_H
