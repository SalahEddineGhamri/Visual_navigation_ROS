/***********************************************************************/
/*                             Color.cpp                               */
/***********************************************************************/
/* -VERSION: ROS_Ubuntu 14.04                                          */
/* -AUTHOR:  GUILLAUME Sylvain                                         */
/* -LAST_MODIFICATION: 12/2017                                         */
/***********************************************************************/
#ifndef COLOR_H
#define COLOR_H

#include "ros/ros.h"


using namespace std;

vector<float> color_red()
{
	vector<float> vec;
	vec.push_back(1.0);
	vec.push_back(0.0);
	vec.push_back(0.0);
	return vec;
};

vector<float> color_magenta()
{
	vector<float> vec;
	vec.push_back(1.0);
	vec.push_back(0.0);
	vec.push_back(1.0);
	return vec;
};

vector<float> color_yellow()
{
	vector<float> vec;
	vec.push_back(1.0);
	vec.push_back(1.0);
	vec.push_back(0.0);
	return vec;
};

vector<float> color_orange()
{
	vector<float> vec;
	vec.push_back(1.0);
	vec.push_back(5.0);
	vec.push_back(0.0);
	return vec;
};

vector<float> color_cyan()
{
	vector<float> vec;
	vec.push_back(0.0);
	vec.push_back(1.0);
	vec.push_back(1.0);
	return vec;
};

vector<float> color_green()
{
	vector<float> vec;
	vec.push_back(0.0);
	vec.push_back(1.0);
	vec.push_back(0.0);
	return vec;
};

vector<float> color_blue()
{
	vector<float> vec;
	vec.push_back(0.0);
	vec.push_back(0.0);
	vec.push_back(1.0);
	return vec;
};

vector<float> color_white()
{
	vector<float> vec;
	vec.push_back(1.0);
	vec.push_back(1.0);
	vec.push_back(1.0);
	return vec;
};

vector<float> color_black()
{
	vector<float> vec;
	vec.push_back(0.0);
	vec.push_back(0.0);
	vec.push_back(0.0);
	return vec;
};


#endif //COLOR_H