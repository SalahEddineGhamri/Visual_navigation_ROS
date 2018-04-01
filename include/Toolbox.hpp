/***********************************************************************/
/*                             Toolbox.hpp                             */
/***********************************************************************/
/* -VERSION: ROS_Ubuntu 14.04                                          */
/* -AUTHOR:  GUILLAUME Sylvain                                         */
/* -LAST_MODIFICATION: 03/2018                                         */
/***********************************************************************/
#ifndef TOOLBOX_H
#define TOOLBOX_H

#include "ros/ros.h"

using namespace std;

class Toolbox
{
	private:

	public:

		//Constructor
		Toolbox(){};

		//Destructor
		~Toolbox(){};

		//Operator

		vector<float> vector_AB(vector<float> a, vector<float> b);
		vector<float> vector_vision(float angle);
		float distance(vector<float> p1, vector<float> p2);
		vector<float> normalized_2D(vector<float> vec);
		float dot_2D(vector<float> vec1, vector<float> vec2);
		float determinant_2D(vector<float> vec1, vector<float> vec2);
		float calculate_angle(vector<float> u, vector<float> v);

};


#endif //TOOLBOX_H
