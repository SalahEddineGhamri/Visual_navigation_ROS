/***********************************************************************/
/*                             Point_Cloud.hpp                         */
/***********************************************************************/
/* -VERSION: ROS_Ubuntu 14.04                                          */
/* -AUTHOR:  BREFEL Hugo                                         */
/* -LAST_MODIFICATION: 12/2017                                         */
/***********************************************************************/
#ifndef POINT_CLOUD_H
#define POINT_CLOUD_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "visualization_msgs/Marker.h"
#include "Target.hpp"
#include "Map_node.hpp"
#include <fstream>
#include <cstdlib>
#include <ctime>

# define SIZE_FILTER_NEIGHBOR 4	// taille du filtre de voisinage
# define VISIBILITY 20	// the minimal distance between two points of the point cloud
# define NB_ERR_CONS 8000	// number of consecutive error while trying to add a point before we stop the algorithm

using namespace std;

class Point_Cloud
{
	private:
		Map_node map_real_;	// the map on wich we extract informations
		vector<int> map_treatment_; // the map used to apply intermediate treatments
		vector<Target> targets_;	// the vector in wich we will store the targets of the point cloud
		vector<Target> markers_;	// the vector in wich are stored the Targets representing the ar-markers
		vector<float> start_point_;	// a vector containing the position of the start point
		float dist_vis_;	// the minimal distance between two points of the point cloud
		int max_err_cons_;	// the number of consecutive errors before we stop trying to add points to the point cloud
		int width_map_;	// the width of the map
		int height_map_;	// the height of the map
		float resolution_;	// the resolution of the map

	public:

		//Constructor
		Point_Cloud(Map_node &map, vector<Target> &markers, vector<float> &start_point);
		Point_Cloud(){};

		//Destructor
		~Point_Cloud(){};

		//Operator
		vector<Target> create_Point_Cloud();
		void fill_Circle(int x, int y);
		bool is_In_Circle(int x_Center, int y_Center, int x_Point, int y_Point);
		bool is_Target_Redundant(int x_Target, int y_Target);
		bool is_Target_Connected_Pix(int x_Target, int y_Target);
		bool is_Target_Connected_Pos(float x_Target, float y_Target);
		


		//Getter

		//Setter

		//Debug
		void write_BMP (vector<int> &map_data_);

};


#endif //POINT_CLOUD_H
