/***********************************************************************/
/*                             Toolbox.cpp                             */
/***********************************************************************/
/* -VERSION: ROS_Ubuntu 14.04                                          */
/* -AUTHOR:  GUILLAUME Sylvain                                         */
/* -LAST_MODIFICATION: 03/2018                                         */
/***********************************************************************/

# include "Toolbox.hpp"


// vector_AB
// Return the vector between the point a and the point b
// a: position of the first point
// b: position of the second point
vector<float> Toolbox::vector_AB(vector<float> a, vector<float> b)
{
	vector<float> vec;
	vec.push_back(b[0]-a[0]);
	vec.push_back(b[1]-a[1]);
	vec.push_back(b[2]-a[2]);

	return vec;
}

// vector_vision
// Return the vector of vision associated to an angle
// angle: angle of direction
vector<float> Toolbox::vector_vision(float angle)
{
	//vector Xrobot/world = Rz * (1,0,0)t 
	vector<float> vision;

	float x = cos(angle);
	float y = sin(angle);

	vision.push_back(x);
	vision.push_back(y);
	vision.push_back(0.0f);

	return vision;
}

// distance
// Return the distance between two points
// p1: position of the first point
// p2: position of the second point
float Toolbox::distance (vector<float> p1, vector<float> p2)
{
	float diff_x=(p1[0]-p2[0]);
	float diff_y=(p1[1]-p2[1]);
	float diff_z=(p1[2]-p2[2]);
	float dist = sqrt(diff_x*diff_x + diff_y*diff_y + diff_z*diff_z);
	return dist;
}

// normalized_2D
// Return the normalized vector for a 2D environment
// vec: the vector to normalize
vector<float> Toolbox::normalized_2D (vector<float> vec)
{
	float xx=(vec[0]*vec[0]);
	float yy=(vec[1]*vec[1]);
	float norm = sqrt(xx + yy);

	vector<float> res;
	res.push_back(vec[0]/norm);
	res.push_back(vec[1]/norm);

	return res;
}

// dot_2D
// Return the dot from two vector in a 2D environment
// vec1: the first vector
// vec2: the second vector
float Toolbox::dot_2D (vector<float> vec1, vector<float> vec2)
{
	vector<float> normV1 = normalized_2D(vec1);
	vector<float> normV2 = normalized_2D(vec2);
	float norm = normV1[0]*normV2[0] + normV1[1]*normV2[1];
	return norm;
}

// determinant_2D
// Return the determinant between two vector in a 2D environment
// vec1: the first vector
// vec2: the second vector
float Toolbox::determinant_2D (vector<float> vec1, vector<float> vec2)
{
	vector<float> normV1 = normalized_2D(vec1);
	vector<float> normV2 = normalized_2D(vec2);
	float det = normV1[0]*normV2[1] - normV1[1]*normV2[0];
	return det;
}

// calculate_angle
// Return the angle between two vector in a 2D environment
// u: the first vector
// v: the second vector
float Toolbox::calculate_angle (vector<float> u, vector<float> v)
{
	/*float dot = dot_2D(vec1,vec2);
	float alpha = acos(dot);
	float det = determinant_2D(vec1,vec2);
	float sign = det/abs(det);
	return sign * alpha;*/
	 //cos a = (xu * xv + yu * yv + zu * zv) / (racine(xu²+yu²+zu²) * racine(xv²+yv²+zv²))
	float var = (u[0]*v[0]+u[1]*v[1])/(sqrt(u[0]*u[0]+u[1]*u[1])*sqrt(v[0]*v[0]+v[1]*v[1]));
	float alpha = acos(var);
	return alpha;
}