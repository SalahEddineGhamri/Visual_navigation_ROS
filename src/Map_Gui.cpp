/***********************************************************************/
/*                             Map_Gui.cpp                             */
/***********************************************************************/
/* -VERSION: ROS_Ubuntu 14.04                                          */
/* -AUTHOR:  GUILLAUME Sylvain                                         */
/* -LAST_MODIFICATION: 12/2017                                         */
/***********************************************************************/
# include "Map_Gui.hpp"


using namespace std;



//Constructor
Map_Gui::Map_Gui(ros::NodeHandle nh)
{
	nh_=nh;
	//Publisher
	vis_pub = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1);
}




//Destructor
Map_Gui::~Map_Gui(){}




//Add_Object
//add a object of type Marker on map.
//id: 			object's id
//type:			object's type
//position:		object's position in x, y and z (in m)
//orientation:	object's orientation in x, y, z and w (in rad)
//scale:		object's scale in x, y and z (in m)
//color:		object's color (RGB [0.0 .. 1.0])
int Map_Gui::add_Object (int id,int type,vector<float> position, vector<float> orientation, vector<float> scale, vector<float> color)
{
	//Init variables
	int i = 0;
	int sizePos = position.size();
	int sizeOri = orientation.size();
	int sizeSca = scale.size();
	int sizeCol = color.size();

	//Verification
	if(position.size()!=3 )
	{
		ROS_ERROR("[ Add_object ]Error: size Position (%d to 3).",sizePos);
		return -1;
	}
	if(orientation.size()!=4 )
	{
		ROS_ERROR("[ Add_object ]Error: size Orientation (%d to 4).",sizeOri);
		return -2;
	}
	if(scale.size()!=3 )
	{
		ROS_ERROR("[ Add_object ]Error: size scale (%d to 3).",sizeSca);
		return -3;
	}
	if(color.size()!=3 )
	{
		ROS_ERROR("[ Add_object ]Error: size Color (%d to 3).",sizeCol);
		return -4;
	}

	//Init MSG
	visualization_msgs::Marker marker;
	marker.header.frame_id = "/map";
	marker.header.stamp = ros::Time::now();
	marker.ns = "basic_shapes";
	marker.id = id;
	marker.type = type;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = position[0];
	marker.pose.position.y = position[1];
	marker.pose.position.z = position[2];
	marker.pose.orientation.x = orientation[0];
	marker.pose.orientation.y = orientation[1];
	marker.pose.orientation.z = orientation[2];
	marker.pose.orientation.w = orientation[3];
	marker.scale.x = scale[0];
	marker.scale.y = scale[1];
	marker.scale.z = scale[2];
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = color[0];
	marker.color.g = color[1];
	marker.color.b = color[2];
	marker.lifetime = ros::Duration();

	//Send msg
	while(i<5)
	{
		vis_pub.publish( marker );
		ros::Duration(0.1).sleep();
		i++;
	}

	return 0;
}




//Add_Object
//add a object of type Marker on map.
//marker: 	object's message of type visualization_msgs::Marker
int Map_Gui::add_Object (visualization_msgs::Marker marker)
{
	//Init variables
	int i =0;

	//Send msg
	while(i<5)
	{
		vis_pub.publish( marker );
		ros::Duration(0.1).sleep();
		i++;
	}

	return 0;
}

//Add_List_Target
//add a list object of type Target on map.
//marker: 	object's message of type visualization_msgs::Marker
int Map_Gui::add_List_Target (int id, vector<Target> list)
{
//Init MSG
	visualization_msgs::Marker marker;
	marker.header.frame_id = "/map";
	marker.header.stamp = ros::Time::now();
	marker.ns = "target_List";
	marker.id = id;
	marker.type = visualization_msgs::Marker::CUBE_LIST;
	marker.action = visualization_msgs::Marker::ADD;

	marker.pose.position.z = 0.2f;
	marker.pose.orientation.x = 0;
	marker.pose.orientation.y = 0;
	marker.pose.orientation.z = 0;
	marker.pose.orientation.w = 1;

	marker.scale.x = SCALE_X_TARGET;
	marker.scale.y = SCALE_Y_TARGET;
	marker.scale.z = SCALE_Z_TARGET;
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = 1.0;
	marker.color.g = 0.0;
	marker.color.b = 0.0;

	for(unsigned int j=0; j<list.size(); ++j)
	{
		vector<float> pos = list[j].get_Position();
		geometry_msgs::Point p;
		p.x= pos[0];
		p.y= pos[1];
		p.z= pos[2];

		marker.points.push_back(p);

		vector<float> color = list[j].get_Color();
		std_msgs::ColorRGBA c;
		c.r= color[0];
		c.g= color[1];
		c.b= color[2];
		c.a= 1.0;

		marker.colors.push_back(c);
	}

	marker.lifetime = ros::Duration();

	//Send msg
	int i =0;
	while(i<2)
	{
		vis_pub.publish( marker );
		ros::Duration(0.1).sleep();
		i++;
	}

	return 0;
}


//Add_Line
//add a line on map between two traget.
int Map_Gui::add_Line (int id,Target start, Target end, vector<float> color)
{
	visualization_msgs::Marker marker;
	marker.header.frame_id = "/map";
	marker.header.stamp = ros::Time::now();
	marker.ns = "Line";
	marker.id = id;
	marker.type = visualization_msgs::Marker::LINE_STRIP;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = SCALE_X_LINE;
	marker.scale.y = SCALE_Y_LINE;
	marker.scale.z = SCALE_Z_LINE;
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = color[0];
	marker.color.g = color[1];
	marker.color.b = color[2];
	marker.lifetime = ros::Duration();

	geometry_msgs::Point s,e;
	vector<float> vec = start.get_Position();
	s.x = vec[0];
	s.y = vec[1];
	s.z = vec[2]+0.01;//+0.01

	vec = end.get_Position();
	e.x = vec[0];
	e.y = vec[1];
	e.z = vec[2]+0.01;

	marker.points.push_back(s);
	marker.points.push_back(e);

	marker.lifetime = ros::Duration();

	//Send msg
	int i =0;
	while(i<2)
	{
		vis_pub.publish( marker );
		ros::Duration(0.1).sleep();
		i++;
	}
}

//Add_Line
//add a line on map.
int Map_Gui::add_Line (int id,vector<float> start, vector<float> end, vector<float> color)
{
	visualization_msgs::Marker marker;
	marker.header.frame_id = "/map";
	marker.header.stamp = ros::Time::now();
	marker.ns = "Line";
	marker.id = id;
	marker.type = visualization_msgs::Marker::LINE_STRIP;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = SCALE_X_LINE;
	marker.scale.y = SCALE_Y_LINE;
	marker.scale.z = SCALE_Z_LINE;
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = color[0];
	marker.color.g = color[1];
	marker.color.b = color[2];
	marker.lifetime = ros::Duration();

	geometry_msgs::Point s,e;
	s.x = start[0];
	s.y = start[1];
	s.z = 0.01;//+0.01

	e.x = end[0];
	e.y = end[1];
	e.z = 0.01;

	marker.points.push_back(s);
	marker.points.push_back(e);

	marker.lifetime = ros::Duration();

	//Send msg
	int i =0;
	while(i<2)
	{
		vis_pub.publish( marker );
		ros::Duration(0.1).sleep();
		i++;
	}
}

//Add_Line_List_float
//add a list object of type line on map.
int Map_Gui::add_Line_List_float (int id, vector<vector<float> > list_pos, vector<float> color_RGBA)
{
	//Init MSG
	visualization_msgs::Marker marker;
	marker.header.frame_id = "/map";
	marker.header.stamp = ros::Time::now();
	marker.ns = "Smooth_Path";
	marker.id = id;
	marker.type = visualization_msgs::Marker::LINE_LIST;
	marker.action = visualization_msgs::Marker::ADD;
	marker.scale.x = SCALE_X_LINE;
	marker.scale.y = SCALE_Y_LINE;
	marker.scale.z = SCALE_Z_LINE;
	marker.pose.orientation.x = 0.0f;
	marker.pose.orientation.y = 0.0f;
	marker.pose.orientation.z = 0.0f;
	marker.pose.orientation.w = 1.0f;

	marker.scale.x = 0.1;
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = 1.0;
	marker.color.g = 0.0;
	marker.color.b = 0.0;

	std_msgs::ColorRGBA c;
	c.r= color_RGBA[0];
	c.g= color_RGBA[1];
	c.b= color_RGBA[2];
	c.a= 1.0;


	for(unsigned int j=1; j<list_pos.size(); ++j)
	{
		vector<float> posprev = list_pos[j-1];

		geometry_msgs::Point pprev;
		pprev.x= posprev[0];
		pprev.y= posprev[1];
		pprev.z= 0.01;

		marker.points.push_back(pprev);

		marker.colors.push_back(c);

		vector<float> pos = list_pos[j];
		geometry_msgs::Point p;
		p.x= pos[0];
		p.y= pos[1];
		p.z= 0.01;

		marker.points.push_back(p);

		marker.colors.push_back(c);
	}

	marker.lifetime = ros::Duration();

	//Send msg
	int i =0;
	while(i<2)
	{
		vis_pub.publish( marker );
		ros::Duration(0.1).sleep();
		i++;
	}
	return 0;
}


//Add_Line_List
//add a list object of type line on map.
int Map_Gui::add_Line_List (int id, vector<Target> list_Target, vector<float> color_RGBA)
{
	//Init MSG
	visualization_msgs::Marker marker;
	marker.header.frame_id = "/map";
	marker.header.stamp = ros::Time::now();
	marker.ns = "Path";
	marker.id = id;
	marker.type = visualization_msgs::Marker::LINE_LIST;
	marker.action = visualization_msgs::Marker::ADD;
	marker.scale.x = SCALE_X_LINE;
	marker.scale.y = SCALE_Y_LINE;
	marker.scale.z = SCALE_Z_LINE;
	marker.pose.orientation.x = 0.0f;
	marker.pose.orientation.y = 0.0f;
	marker.pose.orientation.z = 0.0f;
	marker.pose.orientation.w = 1.0f;

	marker.scale.x = 0.1;
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = 1.0;
	marker.color.g = 0.0;
	marker.color.b = 0.0;

	std_msgs::ColorRGBA c;
	c.r= color_RGBA[0];
	c.g= color_RGBA[1];
	c.b= color_RGBA[2];
	c.a= 1.0;


	for(unsigned int j=1; j<list_Target.size(); ++j)
	{
		vector<float> posprev = list_Target[j-1].get_Position();

		geometry_msgs::Point pprev;
		pprev.x= posprev[0];
		pprev.y= posprev[1];
		pprev.z= 0.01;

		marker.points.push_back(pprev);

		marker.colors.push_back(c);

		vector<float> pos = list_Target[j].get_Position();

		geometry_msgs::Point p;
		p.x= pos[0];
		p.y= pos[1];
		p.z= 0.01;

		marker.points.push_back(p);

		marker.colors.push_back(c);
	}

	marker.lifetime = ros::Duration();

	//Send msg
	int i =0;
	while(i<2)
	{
		vis_pub.publish( marker );
		ros::Duration(0.1).sleep();
		i++;
	}

	return 0;
}


//Add_Elipse_Conf
//add a elipse object on map.
int Map_Gui::add_Elipse_Conf (int id, vector<float> pos, vector<float> orien, vector<float> scale)
{
	//Init MSG
	visualization_msgs::Marker marker;
	marker.header.frame_id = "/map";
	marker.header.stamp = ros::Time::now();
	marker.ns = "Elipse";
	marker.id = id;
	marker.type = visualization_msgs::Marker::CYLINDER;
	marker.action = visualization_msgs::Marker::ADD;

	marker.scale.x = scale[0];
	marker.scale.y = scale[1];
	marker.scale.z = 0.01;

	marker.pose.orientation.x = orien[0];
	marker.pose.orientation.y = orien[1];
	marker.pose.orientation.z = orien[2];
	marker.pose.orientation.w = orien[3];

	marker.color.a = 0.6; // Don't forget to set the alpha!
	marker.color.r = 1.0;
	marker.color.g = 1.0;
	marker.color.b = 0.20;

	marker.pose.position.x = pos[0];
	marker.pose.position.y = pos[1];
	marker.pose.position.z = 0.15;

	marker.lifetime = ros::Duration();

	//Send msg
	int i =0;
	while(i<2)
	{
		vis_pub.publish( marker );
		ros::Duration(0.05).sleep();
		i++;
	}
	return 0;
}


//Add_cone_visibility 
//add a cone object on map.
int Map_Gui::add_cone_visibility (int id,float theta,float angles,vector<float> pose,float distance)
{
	//Init MSG
	visualization_msgs::Marker marker;
	marker.header.frame_id = "/map";
	marker.header.stamp = ros::Time::now();
	marker.ns = "Visibility_Cone";
	marker.id = id;
	marker.type = visualization_msgs::Marker::LINE_LIST;
	marker.action = visualization_msgs::Marker::ADD;
	marker.scale.x = 0.1;
	marker.scale.y = SCALE_Y_LINE;
	marker.scale.z = SCALE_Z_LINE;
	marker.pose.orientation.x = 0.0f;
	marker.pose.orientation.y = 0.0f;
	marker.pose.orientation.z = 0.0f;
	marker.pose.orientation.w = 1.0f;

	marker.color.a = 0.2; // Don't forget to set the alpha!
	marker.color.r = 0.8;
	marker.color.g = 0.8;
	marker.color.b = 0.3;

	std_msgs::ColorRGBA c;
	c.r= 1.0;
	c.g= 1.0;
	c.b= 0.3;
	c.a= 0.2;

	int nb_lines = (angles*2)/0.05;
	float alpha=angles::normalize_angle(theta-angles);

	for(int j=0;j<nb_lines;j++)
	{

		geometry_msgs::Point pprev;
		pprev.x= pose[0];
		pprev.y= pose[1];
		pprev.z= 0.02;

		marker.points.push_back(pprev);

		marker.colors.push_back(c);

		geometry_msgs::Point p;
		p.x= cos(alpha)*distance+pose[0];
		p.y= sin(alpha)*distance+pose[1];
		p.z= 0.02;

		marker.points.push_back(p);

		marker.colors.push_back(c);
		alpha=angles::normalize_angle(alpha +0.05);
	}

	marker.lifetime = ros::Duration();

	//Send msg
	int i =0;
	while(i<2)
	{
		vis_pub.publish( marker );
		ros::Duration(0.1).sleep();
		i++;
	}

	return 0;
}


//Debug Mod
//Test_add_Object
//add a blue cube Marker at the position (0,0,0) without rotation.
//id:	object's id
int Map_Gui::Test_add_Object (int id)
{
	vector<float> position;
	position.push_back(0.0f);
	position.push_back(0.0f);
	position.push_back(0.0f);

	vector<float> orientation ;
	orientation.push_back(0.0f);
	orientation.push_back(0.0f);
	orientation.push_back(0.0f);
	orientation.push_back(1.0f);

	vector<float> scale ;
	scale.push_back(0.1);
	scale.push_back(0.1);
	scale.push_back(0.01);

	vector<float> color;
	color.push_back(0.0);
	color.push_back(0.0);
	color.push_back(1.0);

	add_Object (visualization_msgs::Marker::CUBE,id,position,orientation,scale,color);
}
