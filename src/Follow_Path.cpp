/***********************************************************************/
/*                           Follow_Path.cpp                           */
/***********************************************************************/
/* -VERSION: ROS_Ubuntu 14.04                                          */
/* -AUTHOR:  RUBIO Luc                                                 */
/* -LAST_MODIFICATION: 03/2018                                         */
/***********************************************************************/

# include "Follow_Path.hpp"
# define DIST_MIN 0.50
# define DIST_FINAL 0.10
# define RATIO_ANGULAR 0.6
# define RATIO_LINEAR 0.1
# define SPEED_LINEAR 0.15
# define OBSTACLE_MUR 120000
# define PI 3.1415
# define DETECTION_OBSTACLE false 


//Constructor
Follow_Path::Follow_Path(ros::NodeHandle nh)
{
	nh_=nh;
	map_gui_ = Map_Gui(nh);
	image_transport::ImageTransport it_(nh);
	// Publisher
	pub_Velocity_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 100);
	image_sub_ = it_.subscribe("/camera/depth_registered/image", 1000, &Follow_Path::Callback_obstacle, this, image_transport::TransportHints("compressedDepth"));
	// Subscriber
	sub_Odom_ = nh_.subscribe("/odom",1000,&Follow_Path::CallBack_Odom,this);
	Robot_pose_.resize(3);			//(x,y,z) in m in world
	Robot_orientation_.resize(4);	//(x,y,z,w) in rad in world
	Robot_variance_.resize(3);
	Robot_pose_[0]= 0.0f;
	Robot_pose_[1]= 0.0f;
	Robot_pose_[2]= 0.0f;
	Robot_orientation_[0]= 0.0f;
	Robot_orientation_[1]= 0.0f;
	Robot_orientation_[2]= 0.0f;
	Robot_orientation_[3]= 0.0f;
	is_Obstacle = false;
}

// Callback of the /odom topic
// Gets the odometry informations and calculates the Euler angle from Quaternion informations
void Follow_Path::CallBack_Odom(const nav_msgs::Odometry::ConstPtr& msg)
{
	Robot_pose_[0]= msg->pose.pose.position.x;
	Robot_pose_[1]= msg->pose.pose.position.y;
	Robot_pose_[2]= msg->pose.pose.position.z;
	tf::Quaternion q(msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z,msg->pose.pose.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll,pitch,yaw);
	Robot_orientation_[0]= roll;
	Robot_orientation_[1]= pitch;
	Robot_orientation_[2]= yaw;
	gamma = angles::normalize_angle(yaw);
	gamma90 = angles::normalize_angle(gamma+1.57079f);
	//Robot_variance_[0] = msg->pose.covariance[0];
	//Robot_variance_[1] = msg->pose.covariance[7];
}


// Callback of the /camera/depth_registered/image topic
// Calculates the point cloud position of the obstacle
void Follow_Path::Callback_obstacle(const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImagePtr img = cv_bridge::toCvCopy(msg);
  float nb_point=0;
  bool obstacle = false;
  int floor_max = img->image.rows;
  for(int m  = 0; m<img->image.cols ; ++m)
  {
    for(int n  = 0; n<img->image.rows ; ++n)
    {
        if((img->image.at<float>(n, m) < CAM_TO_FRONT + SECURITY_DISTANCE) && not(isnan(img->image.at<float>(n, m))))
        	if(n > floor_max)
        		floor_max = n;
            nb_point++;
    }
    //printf("bottom image : %d floor_max : %d\n", img->image.rows, floor_max);
  }
    
  if(nb_point>1000 && nb_point<OBSTACLE_MUR)  
  {
      is_Obstacle = DETECTION_OBSTACLE;   
      for (int i = LEFT_WINDOW_OBSTACLE; i < RIGHT_WINDOW_OBSTACLE; ++i) {
        for (int j = BOTTOM_WINDOW_OBSTACLE; j < UP_WINDOW_OBSTACLE; ++j) {
          if ((img->image.at<float>(j, i) < CAM_TO_FRONT + SECURITY_DISTANCE) && not(isnan(img->image.at<float>(j, i)))) {
            float x = (i - img->image.rows/2)*img->image.at<float>(j, i) / FOCALE ;
            float y = (j - img->image.cols/2)*img->image.at<float>(j, i) / FOCALE;
            float z = img->image.at<float>(j, i) - CAM_TO_CENTER;
            vector<float> vecTemp;
            vecTemp.push_back(z*cos(gamma) - x*(-sin(gamma)) +Robot_pose_[0]);
            vecTemp.push_back(z*sin(gamma) - x*cos(gamma) + Robot_pose_[1]);
            vecTemp.push_back(-Robot_pose_[2]);
            this->pointCloud.push_back(vecTemp);
          }
        }
      }
  }
}



// GetPointCloud
// Returns the point cloud we built
vector<vector<float> > Follow_Path::getPointCloud() {
  is_Obstacle = false;  
  return this->pointCloud;
}



// Send_MSG_Velocity
// Publish the linear and angular speeds to make the TurtleBot move
// linear: the linear speed
// angular: the angular speed
void Follow_Path::send_MSG_Velocity (float linear, float angular)
{
	geometry_msgs::Twist msg;
	msg.linear.x = linear;
	msg.angular.z = angular;
	pub_Velocity_.publish(msg);
}



// Vector_AB
// Returns the vector from point A to point B
// a: vector containing the position (x,y,z) of point A
// b: vector containing the position (x,y,z) of point B
vector<float> Follow_Path::vector_AB (vector<float> a, vector<float> b)
{
	vector<float> vec;
	vec.push_back(b[0]-a[0]);
	vec.push_back(b[1]-a[1]);
	vec.push_back(b[2]-a[2]);
	return vec;
}


// Vector_vision
// Returns the vector containing the information of robot orientation
vector<float> Follow_Path::vector_vision()
{
	//vector Xrobot/world = Rz * (1,0,0)t 
	vector<float> vision;
	float alpha = Robot_orientation_[2];

	float x = cos(alpha);
	float y = sin(alpha);

	vision.push_back(x);
	vision.push_back(y);
	vision.push_back(0.0f);
	return vision;
}


// Distance
// Returns the euclidian distance between two vectors
// p1: the first vector
// p2: the second vector
float Follow_Path::distance (vector<float> p1, vector<float> p2)
{
	float diff_x=(p1[0]-p2[0]);
	float diff_y=(p1[1]-p2[1]);
	float diff_z=(p1[2]-p2[2]);
	float dist = sqrt(diff_x*diff_x + diff_y*diff_y + diff_z*diff_z);
	return dist;
}


// Normalized_2D
// Normalizes a 2D vector
// vec: the vector to normalize
vector<float> Follow_Path::normalized_2D (vector<float> vec)
{
	float xx=(vec[0]*vec[0]);
	float yy=(vec[1]*vec[1]);
	float norm = sqrt(xx + yy);

	vector<float> res;
	res.push_back(vec[0]/norm);
	res.push_back(vec[1]/norm);

	return res;
}


// Dot_2D
// Return the dot from two vector in a 2D environment
// vec1: the first vector
// vec2: the second vector
float Follow_Path::dot_2D (vector<float> vec1, vector<float> vec2)
{
	vector<float> normV1 = normalized_2D(vec1);
	vector<float> normV2 = normalized_2D(vec2);
	float norm = normV1[0]*normV2[0] + normV1[1]*normV2[1];
	return norm;
}


// Determinant_2D
// Return the determinant between two vector in a 2D environment
// vec1: the first vector
// vec2: the second vector
float Follow_Path::determinant_2D (vector<float> vec1, vector<float> vec2)
{
	vector<float> normV1 = normalized_2D(vec1);
	vector<float> normV2 = normalized_2D(vec2);
	float det = normV1[0]*normV2[1] - normV1[1]*normV2[0];
	return det;
}


// Calculate_angle
// Return the angle between two vector in a 2D environment
// u: the first vector
// v: the second vector
float Follow_Path::calculate_angle (vector<float> u1, vector<float> v1)
{
	float det = determinant_2D(u1,v1);
	float sign = det/abs(det);
	vector<float> u = normalized_2D(u1);
	vector<float> v = normalized_2D(v1);
	float var = (u[0]*v[0]+u[1]*v[1])/(sqrt(u[0]*u[0]+u[1]*u[1])*sqrt(v[0]*v[0]+v[1]*v[1]));
	float alpha = acos(var);
	alpha = alpha *(sign);
	return alpha;
}




bool Follow_Path::goToPoint(vector<float> goal)
{
	float dist = distance(Robot_pose_,goal);
	float theta,alpha;
	float linear,angular;
	vector<float> vecGoal ;
	vector<float> vecRobot;
	vector<float> vecX;
	vecX.push_back(1.0f);
	vecX.push_back(0.0f);
	vecX.push_back(0.0f);

	//printf("... NEW GOAL ...\n");
	while(dist>=DIST_MIN && ros::ok() && !is_Obstacle)
	{
		ros::spinOnce();

		vecGoal.clear();
		vecGoal = vector_AB(Robot_pose_,goal);
		vecRobot = vector_vision();

		
		
		theta = atan2(((vecGoal[1]*10)-vecRobot[1]),((vecGoal[0]*10)-vecRobot[0]));
		alpha = angles::shortest_angular_distance(Robot_orientation_[2],theta);

		if(alpha<0.005 && alpha>-0.005)
			alpha =0.0f;


		angular = alpha*RATIO_ANGULAR;

		//printf("alpha:%f theta:%f gama:%f\n",alpha,theta,Robot_orientation_[2]);
		//printf("x:%f y:%f\n",vecGoal[0],vecGoal[1]);
		linear = SPEED_LINEAR + dist * RATIO_LINEAR * (1.20f - abs(angular));
		//printf("linear:%f angular:%f\n",linear,alpha*RATIO_ANGULAR);
		send_MSG_Velocity(linear,angular);
		dist = distance(Robot_pose_,goal);
		ros::Duration(0.05).sleep();
		//map_gui_.add_Elipse_Conf (0, Robot_pose_, Robot_orientation_,Robot_variance_);
	}
	
	send_MSG_Velocity(0.0,0.0);
	return(is_Obstacle);
}


bool Follow_Path::correction_Angles(vector<float> goal)
{
	float dist = distance(Robot_pose_,goal);
	float theta,alpha;
	float linear,angular;
	vector<float> vecGoal ;
	vector<float> vecRobot;
	vector<float> vecX;
	vecX.push_back(1.0f);
	vecX.push_back(0.0f);
	vecX.push_back(0.0f);

	//printf("... NEW GOAL ...\n");
	while(true && ros::ok() && !is_Obstacle)
	{
	
		ros::spinOnce();
		vecGoal.clear();
		vecGoal = vector_AB(Robot_pose_,goal);
		vecRobot = vector_vision();	
		
		theta = atan2(((vecGoal[1]*10)-vecRobot[1]),((vecGoal[0]*10)-vecRobot[0]));
		alpha = angles::shortest_angular_distance(Robot_orientation_[2],theta);

		if(alpha<0.006 && alpha>-0.006)
			return(is_Obstacle);

		angular = alpha*RATIO_ANGULAR+0.2*(alpha/abs(alpha));

		linear = 0.0f;
		send_MSG_Velocity(linear,angular);
		ros::Duration(0.05).sleep();
		//map_gui_.add_Elipse_Conf (0, Robot_pose_, Robot_orientation_,Robot_variance_);
	}
	
	send_MSG_Velocity(0.0,0.0);
	return(is_Obstacle);
}

bool Follow_Path::correction_Dist(vector<float> goal)
{
	float dist = distance(Robot_pose_,goal);
	float theta,alpha;
	float linear,angular;
	vector<float> vecGoal ;
	vector<float> vecRobot;
	vector<float> vecX;
	vecX.push_back(1.0f);
	vecX.push_back(0.0f);
	vecX.push_back(0.0f);

	while(dist>=DIST_FINAL && ros::ok() && !is_Obstacle)
	{
		ros::spinOnce();

		angular = 0.0f;


		linear = SPEED_LINEAR + dist * RATIO_LINEAR * (1.20f - abs(angular));
		send_MSG_Velocity(linear,angular);
		dist = distance(Robot_pose_,goal);
		ros::Duration(0.05).sleep();
		//map_gui_.add_Elipse_Conf (0, Robot_pose_, Robot_orientation_,Robot_variance_);
	}
	send_MSG_Velocity(0.0,0.0);
	return(is_Obstacle);
}

void Follow_Path::turn_Over(){
  ROS_INFO("... ROBOT TURNING OVER ...\n");
  double angle_depart = Robot_orientation_[2];
  double angle_courant;
  double seuil = 0.2;
  bool done = false;
  // while we don't see any amer
  while (!done && (ros::ok())) {
    // the robot turns for PI
    angle_courant = Robot_orientation_[2];
    if (abs(angles::shortest_angular_distance(angle_depart,angle_courant)) > PI - seuil) {
      done = true;
    }
    else {
      send_MSG_Velocity(0.0, 1.0);
    }
    ros::spinOnce();
    ros::Duration(0.05).sleep();
  }
  ROS_INFO("... ROBOT READY TO GO ON ...\n");
}
