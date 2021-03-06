/***********************************************************************/
/*                             Bumper.hpp                              */
/***********************************************************************/
/* -VERSION: ROS_Ubuntu 14.04                                          */
/* -AUTHOR:  BEAUHAIRE Pierre                                          */
/* -LAST_MODIFICATION: 03/2018                                         */
/***********************************************************************/

#ifndef BUMPER_H
#define BUMPER_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <kobuki_msgs/BumperEvent.h>
#include <kobuki_msgs/Sound.h>
#include <std_msgs/Bool.h>

class Bumper
{
	private:
    ros::Subscriber bump_subscriber_;       // Subscriber to /mobile_base/events/bumper
    ros::Publisher vel_publisher_;          // Publisher to /mobile_base/commands/velocity
    ros::Publisher sound_publisher_;        // Publisher to /mobile_base/commands/sound
    ros::Publisher bump_pub_;               // Publisher to /chat_bumper
    geometry_msgs::Twist twist_message_;    // The message to make the turtlebot move
    kobuki_msgs::Sound sound_message_;      // The message to make a sound
    std_msgs::Bool bump_state_message_;

	public:

	// Constructors
	Bumper(){};
    Bumper(ros::NodeHandle nh);
    

	// Destructor
	~Bumper(){};

    // Setters
    void bumpCallback(const kobuki_msgs::BumperEvent &message);

    // Operators
    void stop();
    void makeSound();

};

#endif //BUMPER_H

