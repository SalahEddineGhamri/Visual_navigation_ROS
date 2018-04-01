/***********************************************************************/
/*                              Line.cpp                               */
/***********************************************************************/
/* -VERSION: ROS_Ubuntu 14.04                                          */
/* -AUTHOR:  GUILLAUME Sylvain                                         */
/* -LAST_MODIFICATION: 12/2017                                         */
/***********************************************************************/
# include "Line.hpp"

#define SCALE_X 0.02
#define SCALE_Y 0.02
#define SCALE_Z 0.0

//Constructor
	Line::Line(int id,Target &st,Target &en, vector<float> c)
	{
		id_ = id;
		start_ = st;
		end_ = en;
		color_ = c;
	}



//Destructor
	Line::~Line()
	{
		//start_.remove_Son(end_);
		//end_.remove_Son(start_);
	}



//Create_MSG_Marker
//this fonction creates a visualization_msg of the Line
	visualization_msgs::Marker Line::create_MSG_Marker ()
	{
		visualization_msgs::Marker marker;
		marker.header.frame_id = "/map";
		marker.header.stamp = ros::Time::now();
		marker.ns = "Line";
		marker.id = id_;
		marker.type = visualization_msgs::Marker::LINE_STRIP;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.orientation.w = 1.0;
		marker.scale.x = SCALE_X;
		marker.scale.y = SCALE_Y;
		marker.scale.z = SCALE_Z;
		marker.color.a = 1.0; // Don't forget to set the alpha!
		marker.color.r = color_[0];
		marker.color.g = color_[1];
		marker.color.b = color_[2];
		marker.lifetime = ros::Duration();

		geometry_msgs::Point s,e;
		vector<float> vec = start_.get_Position();
		s.x = vec[0];
		s.y = vec[1];
		s.z = vec[2]+0.01;//+0.01

		vec = end_.get_Position();
		e.x = vec[0];
		e.y = vec[1];
		e.z = vec[2]+0.01;

		marker.points.push_back(s);
		marker.points.push_back(e);

		return marker;
	}


//Getter
	Target Line::get_Start (){return start_;}
	Target Line::get_End (){return end_;}
	vector<float> Line::get_Color (){return color_;}
	int Line::get_Id (){return id_;}


//Setter
	void Line::set_Start (Target target){start_=target;}
	void Line::set_End (Target target){end_=target;}
	void Line::set_Color (vector<float> color){color_=color;}

