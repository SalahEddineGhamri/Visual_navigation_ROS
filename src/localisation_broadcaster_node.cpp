/*
  localisation_broadcaster.cpp
  Marine Bouchet & Tristan Klempka
  ROS Node which broadcast constantly the robot position through /odom
  Initially /odom and /map are the same
  Odom is updated by listening to the /new_odom topic
  Initial pos can be set by running :
  "rostopic pub -r 10 /new_odom geometry_msgs/Transform '{translation:  {x: 0.1, y: 0.0, z: 0.0}, rotation: {x: 0.0,y: 0.0,z: 0.0, w: 1.0}}'"
  
 */
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <iostream>

// Update freq. ROS node
static const int NODE_FREQ = 40; // Must be > ~50 Hz if not nav stack throws errors

// Global vars
tf::Transform transform_mapOdom;
tf::Transform transform_mapOdomRot;
ros::Publisher pub_resetodom;

// odomCallback : odom modification when a marker is seen
// Fills transform_mapOdom with new data from the localisation node
// Robot odemtry is also reset because we fully trust the new pos. and orientation from the marker seen
void odomCallback(const geometry_msgs::PoseWithCovarianceStamped& odom_msg) {
    transform_mapOdom.setOrigin(tf::Vector3(odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, odom_msg.pose.pose.position.z));
    transform_mapOdom.setRotation(tf::Quaternion(0,0,0, 1));
    transform_mapOdomRot.setOrigin(tf::Vector3(0, 0, 0));
    transform_mapOdomRot.setRotation(tf::Quaternion(odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w));
}

int main(int argc, char** argv){
    // ROS node init
    ros::init(argc, argv, "localisation_broadcaster_node");
    ros::NodeHandle node; 
 
    // Transform broadcaster and listener
    tf::TransformBroadcaster br;
    tf::TransformListener li;

    //  Subscribers and Publishers
    pub_resetodom = node.advertise<std_msgs::Empty>("/mobile_base/commands/reset_odometry", 1000);   
    ros::Subscriber sub = node.subscribe("/robot_pose_ekf/odom_combined", 1000, odomCallback);
    
    // Identity tf
    tf::Transform transform_identity;
    transform_identity.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    transform_identity.setRotation(q);

    // Localisation TF
    transform_mapOdom = transform_identity;

    ros::Rate loop_rate(NODE_FREQ);
    
    // Classic ROS loop
    while(ros::ok())
    {
    //pub_resetodom.publish(std_msgs::Empty());
	//br.sendTransform(tf::StampedTransform(transform_mapOdomRot, ros::Time::now(), "odom","base_footprint"));
	br.sendTransform(tf::StampedTransform(transform_mapOdom, ros::Time::now(), "map","odom"));
	ros::spinOnce();
	loop_rate.sleep();
    }

    return 0;
};
