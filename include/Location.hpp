/***********************************************************************/
/*                         Location.hpp                            */
/***********************************************************************/
/* -VERSION: ROS_Ubuntu 14.04                                          */
/* -AUTHOR:  GUILLAUME Sylvain                                         */
/* -LAST_MODIFICATION: 03/2018                                         */
/***********************************************************************/

# ifndef LOCATION_H
# define LOCATION_H

# include <ros/ros.h>
# include <nav_msgs/Odometry.h>
# include <math.h>




class Location
{
	private:
    ros::Subscriber vo_subscriber_;            // Publisher to /vo
    ros::Publisher  odom_publisher_;            // Publisher to /vo
    nav_msgs::Odometry odom_msg_;             // The message to save odometry informations
    bool msg_ok_;

   

	public:

	// Constructors
    Location(ros::NodeHandle nh);
    Location(){};

	// Destructor
	~Location(){};

    // Setters
    void vo_Callback(nav_msgs::Odometry msg);
    
    // Getters

    // Operators
    void relocation();
};

#endif //LOCATION_H
