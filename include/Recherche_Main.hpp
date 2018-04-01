/***********************************************************************/
/*                           Recherche.hpp                             */
/***********************************************************************/
/* -VERSION: ROS_Ubuntu 14.04                                          */
/* -AUTHOR:  BEAUHAIRE Pierre                                          */
/* -LAST_MODIFICATION: 03/2018                                         */
/***********************************************************************/

#ifndef RECHERCHE_MAIN_H
#define RECHERCHE_MAIN_H

#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include "Bumper.hpp"

#define DEBUT 0                                 // State for the beginning of the research
#define FIN 1                                   // State for the end of the research

class Recherche_Main
{
	private:
    ros::NodeHandle nh_;                        // The node handle for Recherche
    ros::Subscriber recherche_subscriber_;      // Subscriber to /chat_recherche_retour
    int state;                                  // The state of the research

	public:

		// Constructors
    Recherche_Main(ros::NodeHandle nh);
    Recherche_Main(){};

		// Destructor
		~Recherche_Main(){};

		// Getters
    int getState();

    // Setters
    void rechercheCallback(const std_msgs::Int64 &message);

    // Operators
    void start(ros::NodeHandle nh);

};

#endif //RECHERCHE_MAIN_H

