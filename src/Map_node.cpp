/***********************************************************************/
/*                            MAP_Node.cpp                             */
/***********************************************************************/
/* -VERSION: ROS_Ubuntu 14.04                                          */
/* -AUTHOR:  GUILLAUME Sylvain                                         */
/* -LAST_MODIFICATION: 12/2017                                         */
/***********************************************************************/
# include "Map_node.hpp"

# define SIZE_TURTLE 0.38  //m

//Constructor
Map_node::Map_node(ros::NodeHandle nh)
{
	nh_= nh;
	resolution_ = -1.0;
	sub_OccGrid_ = nh_.subscribe("map",1000,&Map_node::CallBack_Map,this);
	pub_OccGrid_ = nh_.advertise<nav_msgs::OccupancyGrid>("map",10);
	pose_.resize(3);
	orientation_.resize(4);
	read_=false;
}

// Destructor
Map_node::~Map_node(){}




// CallBack the subscriber /map
void Map_node::CallBack_Map(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	if(!read_)
	{
		//Get Info
		frame_id_ = msg->header.frame_id;
		resolution_ = msg->info.resolution;
		width_ = msg->info.width;
		height_ = msg->info.height;

		//Get position
		pose_[0]=(msg->info.origin.position.x);
		pose_[1]=(msg->info.origin.position.y);
		pose_[2]=(msg->info.origin.position.z);

		//Get orientation
		orientation_[0]=(msg->info.origin.orientation.x);
		orientation_[1]=(msg->info.origin.orientation.y);
		orientation_[2]=(msg->info.origin.orientation.z);
		orientation_[3]=(msg->info.origin.orientation.w);

		//Init Data's vector
		map_data_.resize(height_*width_);

		//Get Data
		for(unsigned int i=0;i<height_;++i)
			for(unsigned int j =0;j<width_;++j)
				map_data_[((i*width_)+j)] = msg->data[(height_-i-1)*width_+j];

		if(resolution_!=-1.0f)
			read_ = true;
	}
}




// Pose_To_Pix
// Transform a position to a pixel 
// x_pos:	position of x (m)
// y_pos:	position of y (m)
std::vector<int> Map_node::pose_to_pix(float x_pos,float y_pos)
{
	//init
	int x,y,x_tmp,y_tmp,xPose_tmp,yPose_tmp;

	//Calculating
	x_tmp = x_pos/resolution_;
	y_tmp = y_pos/resolution_;
	xPose_tmp = pose_[0]/resolution_;
	yPose_tmp = (pose_[1]/resolution_);
	x = x_tmp - xPose_tmp;
	y = -y_tmp + yPose_tmp +height_ ;

	//Create the vector
	std::vector<int> vec;
	vec.push_back(x);
	vec.push_back(y);
	vec.push_back(0);

	return vec;
}




// Pix_To_Pose
// Transform a pixel to a position 
// x_pix:	position of x (pixel)
// y_pix:	position of y (pixel)
std::vector<float> Map_node::pix_to_pose(int x_pix,int y_pix)
{
	//init
	float x,y,x_tmp,y_tmp;

	//Calculating
	x_tmp = x_pix + (pose_[0]/resolution_);
	x = x_tmp *resolution_;
	y=-((y_pix - height_) * resolution_ - pose_[1]);

	//Create the vector
	std::vector<float> vec;
	vec.push_back(x);
	vec.push_back(y);
	vec.push_back(0.0f);
	return vec;
}





// Is_Intersection
// Check if there isn't obstacle between start and end
// start:	Position of start (x,y,z) in m
// end:		Position of end (x,y,z) in m
bool Map_node::is_intersection (std::vector<float> start, std::vector<float> end)
{
	std::vector<int> pix;
	//Find the direction's vector
	float diff_x=(end[0]-start[0]);
	float diff_y=(end[1]-start[1]);
	float dist = sqrt(diff_x*diff_x + diff_y*diff_y);
	float dir_x = diff_x/dist;
	float dir_y = diff_y/dist;

	//Resize the direction's vector
	dir_x *= (resolution_/1.5f);
	dir_y *= (resolution_/1.5f);

	//Verifacation 
	float pos_x=start[0];
	float pos_y=start[1];
	while((pos_x<end[0]-resolution_ || pos_x>end[0]+resolution_) ||
		  (pos_y<end[1]-resolution_ || pos_y>end[1]+resolution_))
	{
		pix=pose_to_pix(pos_x,pos_y);
		if(100 == get_Val_Pix_Map(pix[0],pix[1]) || -1 == get_Val_Pix_Map(pix[0],pix[1]))
			return true;
		pos_x+=dir_x;
		pos_y+=dir_y;
	}

	return false;
}


// Is_Intersection
// Check if there isn't obstacle between start and end
// start:	Position of pixel (x,y) in int
// end:		Position of pixel (x,y) in int
bool Map_node::is_intersection (std::vector<int> start, std::vector<int> end)
{
	std::vector<int> pix;
	//Find the direction's vector
	float diff_x=((float)end[0]-(float)start[0]);
	float diff_y=((float)end[1]-(float)start[1]);
	float dist = sqrt(diff_x*diff_x + diff_y*diff_y);
	float dir_x = diff_x/dist;
	float dir_y = diff_y/dist;

	//Resize the direction's vector
	dir_x *= (resolution_/1.5f);
	dir_y *= (resolution_/1.5f);

	//Verifacation 
	int pos_x=start[0];
	int pos_y=start[1];
	while((pos_x<end[0]-resolution_ || pos_x>end[0]+resolution_) ||
		  (pos_y<end[1]-resolution_ || pos_y>end[1]+resolution_))
	{
		int pix =  get_Val_Pix_Map(pos_x,pos_y);
		if(pix == 100 || pix == (-1))
			return true;
		pos_x+=dir_x;
		pos_y+=dir_y;
	}

	return false;
}


// Is_Intersection
// Check if there isn't obstacle between start and end
// start:		Position of pixel (x,y) in int
// end:			Position of pixel (x,y) in int
// distance:	Distance threshold in meter
bool Map_node::is_intersection (std::vector<int> start, std::vector<int> end, float distance)
{
	std::vector<int> pix;
	//Find the direction's vector
	float diff_x=((float)end[0]-(float)start[0]);
	float diff_y=((float)end[1]-(float)start[1]);
	float dist = sqrt(diff_x*diff_x + diff_y*diff_y);
	float dir_x = diff_x/dist;
	float dir_y = diff_y/dist;

	//Resize the direction's vector
	dir_x *= (resolution_/1.5f);
	dir_y *= (resolution_/1.5f);

	//Verifacation 
	int pos_x=start[0];
	int pos_y=start[1];

	if(dist>distance/resolution_)
		return true;

	while((pos_x<end[0]-resolution_ || pos_x>end[0]+resolution_) ||
		  (pos_y<end[1]-resolution_ || pos_y>end[1]+resolution_))
	{
		int pix =  get_Val_Pix_Map(pos_x,pos_y);
		if(pix == 100 || pix == (-1))
			return true;
		pos_x+=dir_x;
		pos_y+=dir_y;
	}

	return false;
}

// Is_Intersection
// Check if there isn't obstacle between start and end
// start:	Position of start (x,y,z) in m
// end:		Position of end (x,y,z) in m
// distance:	Distance threshold in meter
bool Map_node::is_intersection (std::vector<float> start, std::vector<float> end, float distance)
{
	std::vector<int> pix;
	//Find the direction's vector
	float diff_x=(end[0]-start[0]);
	float diff_y=(end[1]-start[1]);
	float dist = sqrt(diff_x*diff_x + diff_y*diff_y);
	float dir_x = diff_x/dist;
	float dir_y = diff_y/dist;

	//Resize the direction's vector
	dir_x *= (resolution_/1.5f);
	dir_y *= (resolution_/1.5f);

	//Verifacation 
	float pos_x=start[0];
	float pos_y=start[1];

	if(dist>distance)
		return true;

	while((pos_x<end[0]-resolution_ || pos_x>end[0]+resolution_) ||
		  (pos_y<end[1]-resolution_ || pos_y>end[1]+resolution_))
	{
		pix=pose_to_pix(pos_x,pos_y);
		if(100 == get_Val_Pix_Map(pix[0],pix[1]) || -1 == get_Val_Pix_Map(pix[0],pix[1]))
			return true;
		pos_x+=dir_x;
		pos_y+=dir_y;
	}

	return false;
}





// Is_Location_Ok
// Check if pixels around the input pixel are ok
// x: position in x of the input pixel
// y: position in y of the input pixel
// dim: radius of the filter centered on the input pixel
bool Map_node::is_Location_Ok(int x, int y, int dim){
	for(int i=x-dim; i<x+dim; ++i)
		for(int j=y-dim; j<y+dim; ++j)
			if(x>=0 && y>=0 && x<width_ && y<height_)
				if(map_data_[j*width_+i] == 100 || map_data_[j*width_+i] == -1)
					return false;
	return true;
}





// Update_Map
// Update the map in RVIZ
void Map_node::update_Map()
{
	//Creating the msg for topic /map
	nav_msgs::OccupancyGrid msg ;
	msg.header.frame_id = frame_id_;
	msg.header.stamp = ros::Time::now();
	msg.info.resolution = resolution_;
	msg.info.width = width_;
	msg.info.height= height_;
	msg.info.origin.position.x = pose_[0];
	msg.info.origin.position.y = pose_[1];
	msg.info.origin.position.z = pose_[2];

	// Data of map
	for(unsigned int i=0;i<height_;++i)
		for(unsigned int j =0;j<width_;++j)
			msg.data.push_back( map_data_[(height_-i-1)*width_+j]);

	//Publish
	pub_OccGrid_.publish(msg);
	ros::spinOnce();
	ros::Duration(0.2).sleep();
}




// Dilate_Map
// Dilate the map of the turtle's size
void Map_node::dilate_Map()
{
	// init
	vector<int> data_res;
	// Creating the turtle's filter
	vector<int> filter = create_Filter_Turtlebot2(SIZE_TURTLE,resolution_);
	// Dilate the map
	data_res=dilate_with_Filter_square (map_data_,filter,width_,height_,(sqrt(filter.size())));
	map_data_ = data_res;
}




// Open_Map
// Open the map (Image processing)
// size_filter:		Square filter [size_filter x size_filter]
void Map_node::open_Map(int size_filter)
{
	vector<int> data_res;
	data_res= open_Image_processing(map_data_,size_filter,width_,height_);
	map_data_ = data_res;
}




// Close_Map
// Close the map (Image processing)
// size_filter:		Square filter [size_filter x size_filter]
void Map_node::close_Map(int size_filter)
{

	vector<int> data_res;
	data_res= close_Image_processing (map_data_,size_filter,width_,height_);
	map_data_ = data_res;
}

// Add_obstacle
// add obstacle in map
// list_point:	point_cloud of obstacle (in meter)
void Map_node::Add_obstacle(vector<vector<float> > list_point)
{
	std::vector<int> map;
	map.resize(map_data_.size());

	// Creating a empty map
	for(unsigned int i = 0;i<map.size();++i)
		map[i]=0;

	// Add obstacle in  empty map
	for(unsigned int i =0; i<list_point.size();i=i+10)
	{
		std::vector<int> pixel= pose_to_pix(list_point[i][0],list_point[i][1]);
		map[(pixel[1]*width_)+pixel[0]]=100;
	}

	// Dilation of obstacle
	vector<int> filter = create_Filter_Turtlebot2(SIZE_TURTLE,resolution_);
	map=dilate_with_Filter_square (map,filter,width_,height_,(sqrt(filter.size())));

	// Fusion of the maps
	for(unsigned int i = 0;i<map.size();++i)
	{
		if(map[i]+map_data_[i]>0)
			map_data_[i]=100;
	}

	//update map_data
	update_Map();
}



//Getter and Setter
int Map_node::get_Width_Map()
{
	return width_;
}

int Map_node::get_Height_Map()
{
	return height_;
}

float Map_node::get_Resolution_Map()
{
	return resolution_;
}

float Map_node::get_Position_Origin_Map(int axe)
{
	return pose_[axe];
}

float Map_node::get_Orientation_Origin_Map(int axe)
{
	return orientation_[axe];
}

int Map_node::get_Val_Pix_Map(int x,int y){
	int id= (y*width_)+x;
	return map_data_[id];
}

int Map_node::get_Size_Map(){
	map_data_.size();
}


// DEBUG MOD
void Map_node::write_BMP ()
{
	unsigned char *image = (unsigned char *) new unsigned char [height_*width_];

	std::ofstream f("map.pgm",std::ios_base::out);

	int maxColorValue = 255;
	f<<"P5\n"<<width_<<" "<<height_<<"\n"<<maxColorValue<<"\n";

	for(int i =0;i<height_;++i)
		for(int j =0 ; j<width_;++j)
		{
			if(map_data_[i*width_+j] == -1)
				image[i*width_+j] = (unsigned char)100;
			else if(map_data_[i*width_+j] == 0)
				image[i*width_+j] = (unsigned char)255;
			else if(map_data_[i*width_+j] == 100)
				image[i*width_+j] = (unsigned char)0;
		}

	f.write(reinterpret_cast<char *>(image),(height_*width_)*sizeof(unsigned char));

	exit(1);
}





/* TEST
int main(int argc, char **argv)
{
	ros::init(argc,argv,"Map_Node");
	ros::NodeHandle nh;
	Map_node node(nh);
	Map_Gui mapgui(nh);
	ros::Rate loop_rate(2);


	int xx,yy;
	xx = 185;
	yy=227;
	float xp,yp;
	xp = -2.92f;
	yp = 6.81f;
	std::vector<float> start;
	start.push_back(-1.60f);
	start.push_back(3.35f);
	std::vector<float> end;
	end.push_back(-0.36f);
	end.push_back(3.33f);
	
	while(ros::ok())
	{
		ros::spinOnce();
		if(node.get_Resolution_Map()>0.0)
		{
			std::vector<int> v = node.pose_to_pix(xp,yp);
			std::vector<float> vf = node.pix_to_pose(xx,yy);

		//Debug MOD
		ROS_INFO("Size of Map : %d x %d [%f]",node.get_Width_Map(),node.get_Height_Map(),node.get_Resolution_Map());
		ROS_INFO("Position of Map : x:%f y:%f z:%f",node.get_Position_Origin_Map(0),node.get_Position_Origin_Map(1),node.get_Position_Origin_Map(2));
		ROS_INFO("Orientation of Map : x:%f y:%f z:%f",node.get_Orientation_Origin_Map(0),node.get_Orientation_Origin_Map(1),node.get_Orientation_Origin_Map(2));
		//ROS_INFO("Val pix of Map :[%d,%d] = %d",0,0,node.get_Val_Pix_Map(0,0));
		ROS_INFO("Pos pix of Map :[%d,%d] = [x: %f , y: %f]",xx,yy,vf[0],vf[1]);
		ROS_INFO("Pix pose of Map :[%f,%f] = [x: %d , y: %d]",xp,yp,v[0],v[1]);

		if(node.is_intersection(start,end))
			ROS_INFO("intersection = true");
		else
			ROS_INFO("intersection = false");
		//node.write_BMP();

		
		return(0);

		}
		mapgui.Test_add_Object (0);
		tf::TransformBroadcaster tf;
		ros::spinOnce();
		return(0);

		loop_rate.sleep();

	}
	return(0);
}*/
