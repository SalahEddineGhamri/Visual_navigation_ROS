/***********************************************************************/
/*                          Set_Marker.cpp                             */
/***********************************************************************/
/* -VERSION: ROS_Ubuntu 14.04                                          */
/* -AUTHOR:  BREFEL Hugo                                      		   */
/* -LAST_MODIFICATION: 03/2018                                         */
/***********************************************************************/

# include "Set_Marker.hpp"

//Constructor
Set_Marker::Set_Marker(ros::NodeHandle nh){
	nh_ = nh;
}

// Set_Marker
// return a vector containing all the markers set in the .launch file
vector<Target> Set_Marker::init_Markers(){
	vector<Target> list_markers;
	tf::StampedTransform transform;

	for(int i=0; i<NB_MARKER; ++i){
		std::stringstream sstf;
		sstf << "/marker_" << i;
		std::string id;
		id = sstf.str();

		try{
			listener.waitForTransform("/map", id, ros::Time(0), ros::Duration(0.5));
			listener.lookupTransform("/map", id, ros::Time(0), transform);
		}
		catch (tf::TransformException ex){
			ROS_ERROR("%s", ex.what());
			ros::Duration(1.0).sleep();
		}
		

		vector<float> position;
		vector<float> orientation;

		tf::Quaternion q(transform.getRotation().getX(), transform.getRotation().getY(), transform.getRotation().getZ(), transform.getRotation().getW());
		tf::Matrix3x3 m(q);
		double roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);

		orientation.push_back(roll);
		orientation.push_back(pitch);
		orientation.push_back(yaw);
		orientation.push_back(1.0);


		position.push_back(transform.getOrigin().getX() + cos(orientation[2])*DIST_OPT_AMER);
		position.push_back(transform.getOrigin().getY() + sin(orientation[2])*DIST_OPT_AMER);
		position.push_back(transform.getOrigin().getZ());

		Target marker(i, 2, position, orientation);

		list_markers.push_back(marker);
	}
	return list_markers;
}


vector<Target> Set_Marker::init_ARcode_pose(){
	vector<Target> list_markers;
	tf::StampedTransform transform;

	for(int i=0; i<NB_MARKER; ++i){
		std::stringstream sstf;
		sstf << "/marker_" << i;
		std::string id;
		id = sstf.str();

		try{
			listener.waitForTransform("/map", id, ros::Time(0), ros::Duration(0.5));
			listener.lookupTransform("/map", id, ros::Time(0), transform);
		}
		catch (tf::TransformException ex){
			ROS_ERROR("%s", ex.what());
			ros::Duration(1.0).sleep();
		}
		

		vector<float> position;
		vector<float> orientation;

		tf::Quaternion q(transform.getRotation().getX(), transform.getRotation().getY(), transform.getRotation().getZ(), transform.getRotation().getW());
		tf::Matrix3x3 m(q);
		double roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);

		orientation.push_back(roll);
		orientation.push_back(pitch);
		orientation.push_back(yaw);
		orientation.push_back(1.0);


		position.push_back(transform.getOrigin().getX());
		position.push_back(transform.getOrigin().getY());
		position.push_back(transform.getOrigin().getZ());

		Target marker(i, 2, position, orientation);

		list_markers.push_back(marker);
	}
	return list_markers;
}