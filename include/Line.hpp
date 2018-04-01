/***********************************************************************/
/*                               Line.hpp                              */
/***********************************************************************/
/* -VERSION: ROS_Ubuntu 14.04                                          */
/* -AUTHOR:  GUILLAUME Sylvain                                         */
/* -LAST_MODIFICATION: 12/2017                                         */
/***********************************************************************/
#ifndef MAP_LINE_H
#define MAP_LINE_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "visualization_msgs/Marker.h"
#include "Target.hpp"


using namespace std;

class Line
{
	private:
		int id_;				// Object's id
		Target start_,end_;		// Targets: [start] ----- [end]
		vector<float> color_;	// Line's color in RGB ([0.0 .. 1.0])

	public:

		//Constructor
		Line(int id,Target &st,Target &en, vector<float> color);
		Line(){};

		//Destructor
		~Line();

		//Operator
		visualization_msgs::Marker create_MSG_Marker ();

		//Getter
		Target get_Start ();
		Target get_End ();
		vector<float> get_Color ();
		int get_Id ();

		//Setter
		void set_Start (Target target);
		void set_End (Target target);
		void set_Color (vector<float> color);

};


#endif //MAP_LINE_H