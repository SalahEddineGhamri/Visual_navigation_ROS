/***********************************************************************/
/*                            Map_Gui.hpp                              */
/***********************************************************************/
/* -VERSION: ROS_Ubuntu 14.04                                          */
/* -AUTHOR:  GUILLAUME Sylvain                                         */
/* -LAST_MODIFICATION: 12/2017                                         */
/***********************************************************************/
#ifndef MAP_GUI_H
#define MAP_GUI_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "visualization_msgs/Marker.h"
#include "Target.hpp"
#include <angles/angles.h>

# define SCALE_X_LINE 0.01
# define SCALE_Y_LINE 0.01
# define SCALE_Z_LINE 0.0

using namespace std;

class Map_Gui
{
	private:
		ros::Publisher vis_pub ;
		ros::NodeHandle nh_;

	public:

		//Constructor
		Map_Gui(ros::NodeHandle nh);
		Map_Gui(){};

		//Destructor
		~Map_Gui();

		//Operator
		int add_Object (int id,int type,vector<float> position, vector<float> orientation, vector<float> scale, vector<float> color);
		int add_Object (visualization_msgs::Marker marker);
		int add_List_Target (int id, vector<Target> list);
		int add_Line (int id,Target start, Target end, vector<float> color);
		int add_Line (int id,vector<float> start, vector<float> end, vector<float> color);
		int add_Line_List_float (int id, vector<vector<float> > list_pos, vector<float> color_RGBA);
		int add_Line_List (int id, vector<Target> list_Target, vector<float> color_RGBA);
		int add_Elipse_Conf (int id, vector<float> pos, vector<float> orien, vector<float> scale);
		int add_cone_visibility (int id,float theta,float angles, vector<float> pose,float distance);

		//Debug Mod
		int Test_add_Object (int id);
};


#endif //MAP_GUI_H
