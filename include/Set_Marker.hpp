/***********************************************************************/
/*                          Set_Marker.hpp                            	   */
/***********************************************************************/
/* -VERSION: ROS_Ubuntu 14.04                                          */
/* -AUTHOR:  BREFEL Hugo                                      		   */
/* -LAST_MODIFICATION: 03/2018                                         */
/***********************************************************************/

#ifndef SET_MARKER_H
#define SET_MARKER_H

# include <ros/ros.h>
# include "Target.hpp"
# include <tf/transform_listener.h>
# include <tf/tf.h>
# include <sstream>
# include <string>

# define NB_MARKER 8	// the total number of list_markers
# define DIST_OPT_AMER 1.5	// the optimal distance to an amer to minimize errors

class Set_Marker
{
	private:
		ros::NodeHandle nh_;
		tf::TransformListener listener;	// listener /map
		vector<Target> list_markers_;	// list of the markers on the map

	public:

		//Constructor
		Set_Marker(ros::NodeHandle nh);
		Set_Marker(){};

		//Destructor
		~Set_Marker(){};

		//Operator
		vector<Target> init_Markers();
		vector<Target> init_ARcode_pose();

		//Getter

		//Setter
};

#endif //SET_MARKER_H
