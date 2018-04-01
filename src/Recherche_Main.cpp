/***********************************************************************/
/*                           Recherche.cpp                             */
/***********************************************************************/
/* -VERSION: ROS_Ubuntu 14.04                                          */
/* -AUTHOR:  BEAUHAIRE Pierre                                          */
/* -LAST_MODIFICATION: 03/2018                                         */
/***********************************************************************/

#include "Recherche_Main.hpp"
#include "Recherche.hpp"

// Constructor
Recherche_Main::Recherche_Main(ros::NodeHandle nh) {
  this->nh_ = nh;
  this->recherche_subscriber_ = nh_.subscribe("chat_recherche_retour", 1, &Recherche_Main::rechercheCallback, this);
  this->state = DEBUT;
}

int Recherche_Main::getState() {
  return this->state;
}

// Callback of the recherche topic
// Sets the state if the research is over
void Recherche_Main::rechercheCallback(const std_msgs::Int64 &message) {
  // if research is over (the turtleBot will be in front of the amer)
  if (message.data == 3) {
    this->state = FIN;
  }
}

// Start
// Launches the recherche of amers process
// nh: The NodeHandle used to create an instance of Recherche
void Recherche_Main::start(ros::NodeHandle nh) {
  this->state = DEBUT;
  Recherche recherche(nh);
  while ((getState() != FIN) && (ros::ok())) {
    switch(getState()) {
      case DEBUT:
        // launching the research
        recherche.start();
        this->state = FIN;
        break;
      case FIN:
        break;
      default:
        break;
    }
    ros::Duration(1).sleep();
  }
}

/*
// main
int main(int argc, char **argv) {
	ros::init(argc, argv, "Main");
	ros::NodeHandle nh;
	Recherche_Main r(nh);
  r.start(nh);
  printf("mon state = %d\n", r.getState());
}
*/

