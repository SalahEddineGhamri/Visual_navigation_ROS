/***********************************************************************/
/*                           Recherche.hpp                             */
/***********************************************************************/
/* -VERSION: ROS_Ubuntu 14.04                                          */
/* -AUTHOR:  BEAUHAIRE Pierre                                          */
/* -LAST_MODIFICATION: 03/2018                                         */
/***********************************************************************/

#ifndef RECHERCHE_H
#define RECHERCHE_H

#include <ros/ros.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <nav_msgs/Odometry.h>
#include <kobuki_msgs/Sound.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int64.h>

#define V_LIN_MAX 0.5                // m/s     // Maximal treshold for linear speed
#define V_LIN_MIN 0.3                // m/s     // Minimal treshold for linear speed
#define V_ANG_MAX 0.5                // rad/s   // Maximal turtlebot angular speed
#define V_ANG_MIN 0.5                // rad/s   // Minimal turtlebot angular speed
#define SEUIL_LIN_MAX 4.0            // m       // Treshold used to decrease linear speed
#define SEUIL_LIN_MIN 3.0            // m       // Treshold where robot linear speed must be minimal
#define SEUIL_ANG_MAX 0.1            // m       // Treshold used to decrease angular speed
#define SEUIL_ANG_MIN 0.02           // m       // Treshold where robot angular speed must be minimal
#define SEUIL_CRIT 1.5               // m       // Critical trashold to reach the amer (we must not go closer)
#define BASE_ROBOT 0.075             // m       // Camera - Robot center distance
#define ETAT_RECHERCHE 0                        // State in which the robot is looking for an amer
#define ETAT_ASSERVISSEMENT 1                   // State in which the robot is moving towards an amer
#define ETAT_FIN 2                              // Final state

class Recherche
{
	private:
    ros::NodeHandle nh_;                        // The node handle for Recherche
    ros::Subscriber ar_subscriber_;             // Subscriber to /ar_pose_marker
    ros::Subscriber odom_subscriber_;           // Subscriber to /odom
    ros::Subscriber bump_subscriber_;           // Subscriber to /chat_bumper
    ros::Publisher vel_publisher_;              // Publisher to /mobile_base/commands/velocity
    ros::Publisher sound_publisher_;            // Publisher to /mobile_base/commands/sound
    ros::Publisher recherche_publisher_;        // Publisher to /chat_recherche_retour
    nav_msgs::Odometry odom_message_test;       // The message to save odometry informations
    geometry_msgs::Twist twist_message_;        // The message to make the turtlebot move
    kobuki_msgs::Sound sound_message_;          // The message to make a sound
    std_msgs::Int64 recherche_retour_message_;  // The message to give the state of the research
    double Marker_Position_[2];                 // Marker_Position_[0] = x; Marker_Position_[1] = y; Marker_Position_[2] = z
    int etat_;                                  // The id of the state we are in (Looking for an amer, moving towards an amer, and final state)
    bool recherche_en_cours_;                   // if the turtleBot is looking for or moving towards an amer

	public:

		// Constructors
    Recherche(ros::NodeHandle nh);
    Recherche(){};

		// Destructor
		~Recherche(){};

    // Setters
    void setOdomMessageRecherche(const nav_msgs::Odometry &message);
    void arCallback(ar_track_alvar_msgs::AlvarMarkers req);
    void bumpCallback(const std_msgs::Bool &message);
    void sendTwistMessage(float length, float angle);

		// Getters
    int getEtat();
    double * getMarkerPosition();

    // Operators
    double calculateAngle(float x, float y, float z, float w);
    int signe(double val);
    double calculVitesseAngulaire();
    void rechercherAmer();
    void asservissementAmer();
    void start();

};

#endif //RECHERCHE_H

