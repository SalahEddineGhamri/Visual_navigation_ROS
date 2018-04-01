/***********************************************************************/
/*                             Goal.cpp                                */
/***********************************************************************/
/* -VERSION: ROS_Ubuntu 14.04                                          */
/* -AUTHOR:  RUBIO Luc                                                 */
/* -LAST_MODIFICATION: 03/2018                                         */
/***********************************************************************/
#include "Goal.hpp"

// Constructor
Goal::Goal(ros::NodeHandle nh)
{
	this->nh_ = nh;
	this->type_ = 3;
	this->msg_ok = false;
	this->sub_= nh_.subscribe("move_base_simple/goal",100,&Goal::CallbackreadGoal,this);
}


// Callback of the move_base_simple/goal topic
// Sets the goal position
void Goal::CallbackreadGoal(const geometry_msgs::PoseStamped Pos_goal)
{
	    this->Pose_Goal_= Pos_goal;
	    this->msg_ok = true;
}



// Get_Goal_Position
// Returns a vector containing the position of the Goal
vector<float> Goal::Get_Goal_Position()
{
	vector<float> position;
	position.push_back(this->Pose_Goal_.pose.position.x);
	position.push_back(this->Pose_Goal_.pose.position.y);
	position.push_back(this->Pose_Goal_.pose.position.z);
	
	return(position);
}



// Get_Goal_Orientation
// Returns a vector containing the orientation of the Goal
vector<float> Goal::Get_Goal_Orientation()
{
	vector<float> orientation;
	orientation.push_back(this->Pose_Goal_.pose.orientation.x);
	orientation.push_back(this->Pose_Goal_.pose.orientation.y);
	orientation.push_back(this->Pose_Goal_.pose.orientation.z);
	orientation.push_back(this->Pose_Goal_.pose.orientation.w);
	
	return(orientation);
}


// Setter
void Goal::set_Id_(int id){this->id_ = id;}

// Getter
int Goal::Get_id(){return(this->id_);}
int Goal::Get_type(){return(this->type_);}



// Create_target_goal
// Returns the Target representing the Goal point
// id: the id of the Target
Target Goal::Create_target_goal(int id)
{
    while(!msg_ok && ros::ok())
    {
        ros::spinOnce();
        ros::Duration(0.05).sleep();
    }
    this->msg_ok = false;
	Target Tar(id, this->type_, Get_Goal_Position(), Get_Goal_Orientation());
	return(Tar);
}


//... UNITARY TEST ...

/*
int main(int argc, char **argv)
{
	ros::init(argc, argv, "Goal");
	
	ros::NodeHandle nh;

	Goal Test(nh);
	
	while(ros::ok())
	{
		ros::spinOnce();

		ROS_INFO("pos x : %f\n",Test.Get_Goal_Position()[0]);
		ROS_INFO("pos y : %f\n",Test.Get_Goal_Position()[1]);
	}
	

	return(1);
}*/

