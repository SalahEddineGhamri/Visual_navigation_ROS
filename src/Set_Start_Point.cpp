/***********************************************************************/
/*                        Set_Start_Point.cpp                          */
/***********************************************************************/
/* -VERSION: ROS_Ubuntu 14.04                                          */
/* -AUTHOR:  RUBIO Luc                                                 */
/* -LAST_MODIFICATION: 03/2018                                         */
/***********************************************************************/
#include "Set_Start_Point.hpp"


// Constructor
Set_Start_Point::Set_Start_Point(ros::NodeHandle nh)
{
	this->nh_ = nh;
	this->sub_= nh_.subscribe("/odom",100,&Set_Start_Point::CallbackreadOdom,this);
}


// Callback of the odom topic
// Sets the odom informations
void Set_Start_Point::CallbackreadOdom(const nav_msgs::Odometry Odom)
{
	this->odom_= Odom;
}


// Odom_Position
// Returns a vector containing the position of the start
vector<float> Set_Start_Point::Odom_Position()
{
	vector<float> position;
	position.push_back(this->odom_.pose.pose.position.x);
	position.push_back(this->odom_.pose.pose.position.y);
	position.push_back(this->odom_.pose.pose.position.z);
	
	return(position);
}


// Odom_Orientation
// Returns a vector containing the orientation of the start
vector<float> Set_Start_Point::Odom_Orientation()
{
	vector<float> orientation;
	orientation.push_back(this->odom_.pose.pose.orientation.x);
	orientation.push_back(this->odom_.pose.pose.orientation.y);
	orientation.push_back(this->odom_.pose.pose.orientation.z);
	orientation.push_back(this->odom_.pose.pose.orientation.w);
	
	return(orientation);
}


// Create_target_start
// Returns the Target representing the start point
// id: the id of the Target
Target Set_Start_Point::Create_target_start(int id)
{
    ros::spinOnce();
	Target Tar(id,1, Odom_Position(), Odom_Orientation());
	return(Tar);
}




//... UNITARY TEST ...

/*int main(int argc, char **argv)
{
	ros::init(argc, argv, "Goal");
	ros::NodeHandle nh;
	while(ros::ok())
	{
		ros::spinOnce();
	}
	

	return(1);
}*/


