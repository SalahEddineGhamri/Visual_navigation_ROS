/***********************************************************************/
/*                             NewOdom.cpp                             */
/***********************************************************************/
/* -VERSION: ROS_Ubuntu 14.04                                          */
/* -AUTHOR:  GUILLAUME Sylvain                                         */
/* -LAST_MODIFICATION: 03/2018                                         */
/***********************************************************************/

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <iostream>



// Global vars
nav_msgs::Odometry odomCurr;
nav_msgs::Odometry odomRef;
nav_msgs::Odometry odom;
geometry_msgs::PoseWithCovarianceStamped odomCombined;
ros::Publisher newOdom;


void odomComCallback(const geometry_msgs::PoseWithCovarianceStamped& odom_msg) {
      odomRef = odomCurr;
      odomCombined = odom_msg;
}

void odomCallback(const nav_msgs::Odometry& msg) {
      odomCurr = msg;
}

void pub_Odom()
{
     odom = odomCurr;
   
     odom.pose.pose.position.x +=   (odomCombined.pose.pose.position.x - odomRef.pose.pose.position.x);
     odom.pose.pose.position.y +=  (odomCombined.pose.pose.position.y - odomRef.pose.pose.position.y);

	 newOdom.publish(odom);
}




int main(int argc, char** argv){
    // ROS node init
    ros::init(argc, argv, "localisation_broadcaster_node");
    ros::NodeHandle node; 
 
    // Transform broadcaster and listener
    tf::TransformBroadcaster br;
    tf::TransformListener li;

    //  Subscribers and Publishers
    newOdom = node.advertise<nav_msgs::Odometry>("/new_odom",1000);
    ros::Subscriber sub_odom = node.subscribe ("/odom", 1000, odomCallback);
    ros::Subscriber sub_odomCom = node.subscribe("/robot_pose_ekf/odom_combined", 1000, odomComCallback);
    
    
    
    // Classic ROS loop
    while(ros::ok())
    {
	    ros::spinOnce();
	    pub_Odom();
	    ros::Duration(0.02).sleep();
    }

    return 0;
};
