/***********************************************************************/
/*                           Recherche.cpp                             */
/***********************************************************************/
/* -VERSION: ROS_Ubuntu 14.04                                          */
/* -AUTHOR:  BEAUHAIRE Pierre                                          */
/* -LAST_MODIFICATION: 03/2018                                         */
/***********************************************************************/

#include "Recherche.hpp"
#include "Bumper.hpp"

// Constructor
Recherche::Recherche(ros::NodeHandle nh) {
  this->nh_ = nh;
  this->ar_subscriber_ = nh_.subscribe("ar_pose_marker", 1, &Recherche::arCallback, this);
  this->odom_subscriber_ = nh_.subscribe("odom", 1, &Recherche::setOdomMessageRecherche, this);
  this->bump_subscriber_ = nh_.subscribe("chat_bumper", 1, &Recherche::bumpCallback, this);
  this->recherche_publisher_ = nh_.advertise<std_msgs::Int64>("/chat_recherche_retour", 1);
  this->vel_publisher_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
  this->sound_publisher_ = nh_.advertise<kobuki_msgs::Sound>("/mobile_base/commands/sound", 1);
  this->odom_message_test.pose.pose.position.x = 0.0;
  this->odom_message_test.pose.pose.position.y = 0.0;
  this->odom_message_test.pose.pose.orientation.x = 0.0;
  this->odom_message_test.pose.pose.orientation.y = 0.0;
  this->odom_message_test.pose.pose.orientation.z = 0.0;
  this->odom_message_test.pose.pose.orientation.w = 0.0;
  this->twist_message_.linear.x = 0.0;
  this->twist_message_.linear.y = 0.0;
  this->twist_message_.linear.z = 0.0;
  this->twist_message_.angular.x = 0.0;
  this->twist_message_.angular.y = 0.0;
  this->twist_message_.angular.z = 0.0;
  this->sound_message_.value = 0.0;
  this->Marker_Position_[0] = -1.0;
  this->Marker_Position_[1] = -1.0;
  this->Marker_Position_[2] = -1.0;         // at the init, we don't see any amer => x = y = z = -1
  this->etat_ = ETAT_RECHERCHE;
  this->recherche_retour_message_.data = 0;
  this->recherche_en_cours_ = false;
}

// Callback of the ar_pose_marker topic
// Sets the position of the amer
void Recherche::arCallback(ar_track_alvar_msgs::AlvarMarkers req) {
  // if an amer is in sight
  if (!req.markers.empty()) {
    this->Marker_Position_[0] = req.markers[0].pose.pose.position.x;
    this->Marker_Position_[1] = req.markers[0].pose.pose.position.y;
    this->Marker_Position_[2] = req.markers[0].pose.pose.position.z;
  }
  else {
    this->Marker_Position_[0] = -1.0;
    this->Marker_Position_[1] = -1.0;
    this->Marker_Position_[2] = -1.0;
  }
}

// Callback of the odometry topic
// Sets the odom message with odometry informations
void Recherche::setOdomMessageRecherche(const nav_msgs::Odometry &message) {
  this->odom_message_test.pose.pose.position.x = message.pose.pose.position.x;
  this->odom_message_test.pose.pose.position.y = message.pose.pose.position.y;
  this->odom_message_test.pose.pose.orientation.x = message.pose.pose.orientation.x;
  this->odom_message_test.pose.pose.orientation.y = message.pose.pose.orientation.y;
  this->odom_message_test.pose.pose.orientation.z = message.pose.pose.orientation.z;
  this->odom_message_test.pose.pose.orientation.w = message.pose.pose.orientation.w;
}

// Callback of the chat_bump topic
// Stops the robot in case of contact
void Recherche::bumpCallback(const std_msgs::Bool &message) {
  if (message.data) {
    printf("... BUMP ...\n");
    sendTwistMessage(0.0, 0.0);   // stopping the robot
    this->sound_message_.value = 1.0;
    this->sound_publisher_.publish(this->sound_message_);
    exit(0);
  }
}

// SendTwistMessage
// Sets twist message to control speed, and sends the message to /mobile_base/commands/velocity topic
// length: length the turtlebot has to cross in 1 second (m)
// angle:	angle the turtlebot has to cross in 1 second (rad)
void Recherche::sendTwistMessage(float length, float angle) {
  this->twist_message_.linear.x = length;
  this->twist_message_.linear.y = 0.0;
  this->twist_message_.linear.z = 0.0;
  this->twist_message_.angular.x = 0.0;
  this->twist_message_.angular.y = 0.0;
  this->twist_message_.angular.z = angle;
  this->vel_publisher_.publish(this->twist_message_);
}

// GetEtat
// Returns the state of the research
int Recherche::getEtat() {
  return this->etat_;
}

// GetMarkerPosition
// Returns the position (x,y,z) of the amer in sight
double* Recherche::getMarkerPosition() {
  return this->Marker_Position_;
}

// CalculateAngle
// Calculates the orientated angle of the robot
// x, y, z, w: Quaternion representation of the angle
double Recherche::calculateAngle(float x, float y, float z, float w) {
  double cosy, siny, yaw;
  cosy = 1.0 - 2.0 * (y * y + z * z);  
  siny = 2.0 * (w * z + x * y);
  yaw = atan2(siny, cosy);          // yaw : rotation around Z axis
  return (yaw + M_PI);              // angle between [0 ; 2Pi]
}

// Signe
// Returns the signe of -val-
// val: the value we want the sign of
int Recherche::signe(double val) {
  if (val > 0) {
    return 1;
  } else if (val < 0) {
    return -1;
  } else {
    return 0;
  }
}

// CalculVitesseAngulaire
// Returns the angular speed of the robot, depending on the amer position on horizontal axis
double Recherche::calculVitesseAngulaire() {
  double pente_ang = (V_ANG_MAX - V_ANG_MIN) / (SEUIL_ANG_MAX - SEUIL_ANG_MIN);
  double vitesse = 0.0;
  if ((this->Marker_Position_[0] > SEUIL_ANG_MAX) || (this->Marker_Position_[0] < -SEUIL_ANG_MAX)) {
    vitesse = -signe(this->Marker_Position_[0]) * V_ANG_MAX;
  } else if ((this->Marker_Position_[0] > SEUIL_ANG_MIN) || (this->Marker_Position_[0] < -SEUIL_ANG_MIN)) {
    vitesse = pente_ang * this->Marker_Position_[0];
  } else {
    vitesse = -signe(this->Marker_Position_[0]) * V_ANG_MIN;
  }
  return vitesse;
}

// RechercherAmer
// Makes the TurtleBot turns on itself until it sees an amer
void Recherche::rechercherAmer() {
  printf("... RECHERCHE AMER ...\n");
  double angle_depart = calculateAngle(this->odom_message_test.pose.pose.orientation.x, this->odom_message_test.pose.pose.orientation.y, this->odom_message_test.pose.pose.orientation.z, this->odom_message_test.pose.pose.orientation.w);
  double angle_courant = angle_depart;
  double seuil = 0.2;
  int cpt = 0;
  bool found = true;
  // while we don't see any amer
  while (((this->Marker_Position_[0] == -1) || (this->Marker_Position_[1] == -1) || (this->Marker_Position_[2] == -1)) && (found) && (ros::ok())) {
    // the robot turns with 2Pi max
    angle_courant = calculateAngle(this->odom_message_test.pose.pose.orientation.x, this->odom_message_test.pose.pose.orientation.y, this->odom_message_test.pose.pose.orientation.z, this->odom_message_test.pose.pose.orientation.w);
    if (((cpt < 100000) && (cpt > 200)) && ((angle_courant < angle_depart) || (angle_courant > (angle_depart + seuil)))) {
      found = false;
    }
    else {
      sendTwistMessage(0.0, 1.0);
    }
    cpt++;
    ros::spinOnce();
    ros::Duration(0.05).sleep();
  }
  if (found) {
    this->sound_message_.value = 0.0;
    this->sound_publisher_.publish(this->sound_message_);
    this->etat_ = ETAT_ASSERVISSEMENT;
  } else {
    this->sound_message_.value = 1.0;
    this->sound_publisher_.publish(this->sound_message_);
    this->etat_ = ETAT_FIN;
    ROS_INFO("AUCUNE AMER TROUVEE, VEUILLEZ REPOSITIONNER LE ROBOT ET RELANCER\n");
    exit(0);
  }
}

// AsservissementAmer
// Makes the turtlebot move to the front of the amer
void Recherche::asservissementAmer() {
  printf("... ASSERVISSEMENT SUR L'AMER ...\n");
  ros::Duration(0.05).sleep();
  double vitesse_lin = V_LIN_MAX;
  double vitesse_ang = V_ANG_MIN;
  double pente_lin = (V_LIN_MAX - V_LIN_MIN) / (SEUIL_LIN_MAX - SEUIL_LIN_MIN);
  // going forward of V_LIN_MAX m/s until distance_amer = SEUIL_LIN_MAX
  while ((this->Marker_Position_[2]-BASE_ROBOT) > SEUIL_LIN_MAX && ros::ok()) {
    vitesse_ang = calculVitesseAngulaire();
    sendTwistMessage(V_LIN_MAX, vitesse_ang);
    ros::spinOnce();
    ros::Duration(0.05).sleep();
  }
  // going forward with a degressive linear speed until (vitesse = V_LIN_MIN || distance_amer <= SEUIL_LIN_MIN)
  while ((vitesse_lin > V_LIN_MIN) && ((this->Marker_Position_[2]-BASE_ROBOT) > SEUIL_LIN_MIN) && ros::ok()) {
    // updating linear speed
    vitesse_lin = pente_lin * (this->Marker_Position_[2]-BASE_ROBOT);
    vitesse_lin = V_LIN_MIN;
    vitesse_ang = calculVitesseAngulaire();
    sendTwistMessage(vitesse_lin, vitesse_ang);
    ros::spinOnce();
    ros::Duration(0.05).sleep();
  }
  // going forward with vitesse_lin = V_LIN_MIN until distance_amer = SEUIL_CRIT
  while ((this->Marker_Position_[2]-BASE_ROBOT) > SEUIL_CRIT && ros::ok()) {
    vitesse_ang = calculVitesseAngulaire();
    sendTwistMessage(V_LIN_MIN, vitesse_ang);
    ros::spinOnce();
    ros::Duration(0.05).sleep();
  }
  // if we lost the sight of the amer while moving towards it
  if ((this->Marker_Position_[0] == -1) && (this->Marker_Position_[1] == -1) && (this->Marker_Position_[2] == -1) && ros::ok()) {
    this->sound_message_.value = 1.0;
    this->sound_publisher_.publish(this->sound_message_);
    this->etat_ = ETAT_RECHERCHE;
  } else {
    this->sound_message_.value = 0.0;
    this->sound_publisher_.publish(this->sound_message_);
    this->etat_ = ETAT_FIN;
  }
}

// Start
// Describes the TurtleBot moves sequence
void Recherche::start() {
  this->etat_ = ETAT_RECHERCHE;
  this->recherche_retour_message_.data = 1;
  while (getEtat() != ETAT_FIN) {
    switch(getEtat()) {
      case ETAT_RECHERCHE:
        rechercherAmer();
        break;
      case ETAT_ASSERVISSEMENT:
        asservissementAmer();
        break;
      case ETAT_FIN:
        break;
      default:
        break;
    }
    ros::Duration(1).sleep();
  }
  // we publish that looking_for_an_amer state is over
  this->recherche_retour_message_.data = 3;
  this->recherche_publisher_.publish(recherche_retour_message_);
  ros::spinOnce();
  ros::Duration(0.05).sleep();
  this->recherche_en_cours_ = false;
}

/*
// main
int main(int argc, char **argv) {
	ros::init(argc,argv,"Recherche");
	ros::NodeHandle nh;
	Bumper bumper(nh);
	Recherche recherche(nh);
  recherche.start();
}
*/


