/***********************************************************************/
/*                             Goal.hpp                                */
/***********************************************************************/
/* -VERSION: ROS_Ubuntu 14.04                                          */
/* -AUTHOR:  RUBIO Luc                                                 */
/* -LAST_MODIFICATION: 03/2018                                         */
/***********************************************************************/
#ifndef GOAL_H
#define GOAL_H

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "Target.hpp"

#include <vector>
#include <fstream>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

using namespace std;

class Goal
{
	private:
		ros::NodeHandle nh_;                  // The node handle for Recherche
		ros::Subscriber sub_;                 // Subscriber to move_base_simple/goal
		geometry_msgs::PoseStamped Pose_Goal_;// The message to save Pose_Goal_informations
		int type_;                            // The message to save the type of the Target (a goal)
		int id_;                              // The id of the Target
		vector<float> orientation_;
		bool msg_ok;                          // If the goal has been chosen by the user

	public:

		//Constructor
		Goal(ros::NodeHandle nh);
		Goal(){};

		//Destructor
		~Goal(){};

		//Operator
		
		void CallbackreadGoal(const geometry_msgs::PoseStamped Pos_goal);

		//Getter
		vector<float> Get_Goal_Position();
		vector<float> Get_Goal_Orientation();
		int Get_id();
		int Get_type();
		//Setter
		void set_Id_(int id);
		
		Target Create_target_goal(int id);
		
};


#endif //MAP_TARGET_H
