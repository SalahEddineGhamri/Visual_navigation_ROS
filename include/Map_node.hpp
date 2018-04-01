/***********************************************************************/
/*                            Map_node.hpp                             */
/***********************************************************************/
/* -VERSION: ROS_Ubuntu 14.04                                          */
/* -AUTHOR:  GUILLAUME Sylvain                                         */
/* -LAST_MODIFICATION: 12/2017                                         */
/***********************************************************************/
#ifndef MAP_NODE_H
#define MAP_NODE_H

# include "ros/ros.h"
# include "nav_msgs/OccupancyGrid.h"
# include <fstream>
# include <math.h>
# include "Map_Gui.hpp"
# include "Image_processing.hpp"


class Map_node
{
	private:
		ros::Subscriber sub_OccGrid_;		//subscriber /map
		ros::Publisher pub_OccGrid_;		//publisher  /map
		ros::NodeHandle nh_;				
		float resolution_;					//(m/pixel)
		int width_, height_;				//(nb_pixel)
		std::vector<float> pose_;			//(x,y,z) in m
		std::vector<float> orientation_;	//(x,y,z,w) in rad
		std::vector<int> map_data_;			//(pixel)
		std::string frame_id_;				//frame's id
		bool read_;

	public:

		// Constructor
		Map_node(ros::NodeHandle nh);
		Map_node(){};

		//Destructor
		~Map_node();

		// Map_info
		void CallBack_Map(const nav_msgs::OccupancyGrid::ConstPtr& msg);
		std::vector<int> pose_to_pix(float x_pos,float y_pos);
		std::vector<float> pix_to_pose(int x_pix,int y_pix);
		bool is_intersection (std::vector<float> start, std::vector<float> end);
		bool is_intersection (std::vector<int> start, std::vector<int> end);
		bool is_intersection (std::vector<float> start, std::vector<float> end, float distance);
		bool is_intersection (std::vector<int> start, std::vector<int> end, float distance);
		bool is_Location_Ok(int x, int y, int dim);

		// Map_Modifier
		void update_Map();
		void dilate_Map();
		void open_Map(int size_filter);
		void close_Map(int size_filter);
		void Add_obstacle(vector<vector<float> > list_point);

		// Getter
		int get_Width_Map();
		int get_Height_Map();
		float get_Resolution_Map();
		float get_Position_Origin_Map(int axe);
		float get_Orientation_Origin_Map(int axe);
		int get_Val_Pix_Map(int x,int y);
		int get_Size_Map();

		// DEBUG MOD
		void write_BMP ();
};

#endif //MAP_NODE_H