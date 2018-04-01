/***********************************************************************/
/*                       Image_Processing.hpp                          */
/***********************************************************************/
/* -VERSION: ROS_Ubuntu 14.04                                          */
/* -AUTHOR:  GUILLAUME Sylvain                                         */
/* -LAST_MODIFICATION: 12/2017                                         */
/***********************************************************************/
#ifndef IMAGE_PROCESSING_H
#define IMAGE_PROCESSING_H

#include "ros/ros.h"
#include <math.h>

using namespace std;

vector<int> erode (const vector<int> map, int size_Filter,int width,int height);

vector<int> erode_with_Filter_square (const vector<int> map, const vector<int> filter,int width_map,int height_map,int width_filter);

vector<int> dilate (const vector<int> map, int size_Filter,int width,int height);

vector<int> dilate_with_Filter_square (const vector<int> map, const vector<int> filter,int width_map,int height_map,int width_filter);

vector<int> create_Filter_Turtlebot2(float size_turtlebot2,float resolution);

vector<int> open_Image_processing (const vector<int> map, int size_Filter,int width,int height);

vector<int> close_Image_processing (const vector<int> map, int size_Filter,int width,int height);


#endif //IMAGE_PROCESSING_H