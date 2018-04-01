/***********************************************************************/
/*                          Point_Cloud.cpp                            */
/***********************************************************************/
/* -VERSION: ROS_Ubuntu 14.04                                          */
/* -AUTHOR:  BREFEL Hugo                                      		   */
/* -LAST_MODIFICATION: 01/2018                                         */
/***********************************************************************/

# include "Point_Cloud.hpp"

//Constructor
	Point_Cloud::Point_Cloud(Map_node &map, vector<Target> &markers, vector<float> &start_point)
	{
		map_real_ = map;
		markers_ = markers;
		start_point_ = start_point;
		dist_vis_ = VISIBILITY;
		max_err_cons_ = NB_ERR_CONS;
		width_map_ = map_real_.get_Width_Map();
		height_map_ = map_real_.get_Height_Map();
		resolution_ = map.get_Resolution_Map();
	}

	// create_Point_Cloud
	// Return the created point's cloud
	vector<Target> Point_Cloud::create_Point_Cloud()
	{
		int cpt_err_cons = 0;
		int nb_point = markers_.size();
		vector<float> pos;
		vector<int> pos_Pix;

		//copy the map in a temp map for creation of point cloud
		map_treatment_.resize(map_real_.get_Size_Map());
		for(int x=0; x<width_map_; ++x)
		{
			for(int y=0; y<height_map_; ++y)
			{
				map_treatment_[y*width_map_+x] = map_real_.get_Val_Pix_Map(x, y);
			}
		}

		srand(time(NULL));

		//creation of point cloud
		vector<float> orien;
		orien.push_back(0.0f);
		orien.push_back(0.0f);
		orien.push_back(0.0f);
		orien.push_back(1.0f);

		Target t;
		while(cpt_err_cons < max_err_cons_)
		{
    		int x_rand=rand()%width_map_;
    		int y_rand=rand()%height_map_;
			int val_pix = map_treatment_[y_rand*width_map_+x_rand];
 
			// We want to place a new target in an empty area
    		if(val_pix==0 && map_real_.is_Location_Ok(x_rand, y_rand, SIZE_FILTER_NEIGHBOR))
    		{
    			if(is_Target_Connected_Pix(x_rand, y_rand)){
	    			cpt_err_cons = 0;
	    			Target t = Target(nb_point++, 0, map_real_.pix_to_pose(x_rand, y_rand), orien);
	    			targets_.push_back(t);
	    			fill_Circle(x_rand, y_rand);
	    		} else{
	    			++cpt_err_cons;
	    		}

	    	// We want to place a new target in an area of visibility
    		} else if (val_pix==50 && map_real_.is_Location_Ok(x_rand, y_rand, SIZE_FILTER_NEIGHBOR))
    		{
    			if(is_Target_Redundant(x_rand, y_rand) || (!is_Target_Connected_Pix(x_rand, y_rand))){
    				++cpt_err_cons;
    			}
    			else{
	    			t = Target(nb_point++, 0, map_real_.pix_to_pose(x_rand, y_rand), orien);
	    			targets_.push_back(t);
	    			fill_Circle(x_rand, y_rand);	
    			}

    		// We want to place a new target in an obstacle area
    		} else if(val_pix==100)
    		{
    			++cpt_err_cons;
    		}
		}

		//add markers to the vector of targets
		for(int id_marker=0; id_marker<markers_.size(); ++id_marker){
			pos = markers_[id_marker].get_Position();
			if(is_Target_Connected_Pos(pos[0], pos[1])){
				targets_.push_back(markers_[id_marker]);
			}
		}

		write_BMP(map_treatment_);
		return targets_;
	}


	// fill_Circle
	// Fill a circle around the input target
	// x_Point: position in x of the input pixel
	// y_Point: position in y of the input pixelv	
	void Point_Cloud::fill_Circle(int x_Point, int y_Point)
	{
		int radius = ceil(dist_vis_);
		for(int x=x_Point-radius; x<x_Point+radius; ++x)
			if(x>0 && x<width_map_)
				for(int y=y_Point-radius; y<y_Point+radius; ++y)
					if(y>0 && y<height_map_)
						if(is_In_Circle(x, y, x_Point, y_Point))
							if(map_real_.get_Val_Pix_Map(x, y) == 0)
								map_treatment_[y*width_map_+x]=50;
	}


	// is_In_Circle
	// Return true if the input point is inside the input circle
	// x_Center: position in x of the center of the circle
	// y_Center: position in y of the center of the circle
	// x_Point: position in x of the point
	// y_Point: position in y of the point
	bool Point_Cloud::is_In_Circle(int x_Center, int y_Center, int x_Point, int y_Point){
		int radius = ceil(dist_vis_);
		return ((x_Center-x_Point)*(x_Center-x_Point))+((y_Center-y_Point)*(y_Center-y_Point))<=(radius*radius);
	}


	// is_Target_Redundant
	// Return false if the input target is usefull knowing the other targets position, true otherwise
	// x_Target: position in x of the target
	// y_Target: position in y of the target
	bool Point_Cloud::is_Target_Redundant(int x_Target, int y_Target){
		int radius = ceil(dist_vis_);
		vector<int> end_Pix;
		vector<float> start = map_real_.pix_to_pose(x_Target, y_Target);
		vector<float> end;
		for(int id_Target=0; id_Target<targets_.size(); ++id_Target){
			end = targets_[id_Target].get_Position();
			end_Pix = map_real_.pose_to_pix(end[0], end[1]);
			if(is_In_Circle(x_Target, y_Target, end_Pix[0], end_Pix[1])){
				if(!map_real_.is_intersection(start, end, ((float)(VISIBILITY+10) * resolution_))){
					return true;
				}
			}
		}
		return false;
	}


	// is_Target_Connected
	// Return true if the input target is connected to the others targets, false otherwise
	// x_Target: position in x of the target
	// y_Target: position in y of the target
	bool Point_Cloud::is_Target_Connected_Pix(int x_Target, int y_Target){
		vector<float> start = map_real_.pix_to_pose(x_Target, y_Target);
		vector<float> end = start_point_;
		if(!map_real_.is_intersection(start, end, ((float)(VISIBILITY+10) * resolution_))){
			return true;
		}
		for(int id_Target=0; id_Target<targets_.size(); ++id_Target){
			end = targets_[id_Target].get_Position();
			if(!map_real_.is_intersection(start, end, ((float)(VISIBILITY+10) * resolution_))){
				return true;
			}
		}
		return false;
	}


	// is_Target_Connected
	// Return true if the input target is connected to the others targets, false otherwise
	// x_Target: position in x of the target
	// y_Target: position in y of the target
	bool Point_Cloud::is_Target_Connected_Pos(float x_Target, float y_Target){
		vector<float> start;
		start.push_back(x_Target);
		start.push_back(y_Target); 
		vector<float> end = start_point_;
		if(!map_real_.is_intersection(start, end, ((float)(VISIBILITY+10) * resolution_))){
			return true;
		}
		for(int id_Target=0; id_Target<targets_.size(); ++id_Target){
			end = targets_[id_Target].get_Position();
			if(!map_real_.is_intersection(start, end, ((float)(VISIBILITY+10) * resolution_))){
				return true;
			}
		}
		return false;
	}



	// DEBUG MOD
void Point_Cloud::write_BMP (vector<int> &map_data_)
{
	unsigned char *image = (unsigned char *) new unsigned char [height_map_*width_map_];

	ofstream f("map.pgm",std::ios_base::out);

	int maxColorValue = 255;
	f<<"P5\n"<<width_map_<<" "<<height_map_<<"\n"<<maxColorValue<<"\n";

	for(int i =0;i<height_map_;++i)
		for(int j =0 ; j<width_map_;++j)
		{
			if(map_data_[i*width_map_+j] == -1)
				image[i*width_map_+j] = (unsigned char)100;
			else if(map_data_[i*width_map_+j] == 0)
				image[i*width_map_+j] = (unsigned char)255;
			else if(map_data_[i*width_map_+j] == 100)
				image[i*width_map_+j] = (unsigned char)0;
			else
				image[i*width_map_+j] = (unsigned char)150;
		}
	ROS_INFO("   [DEBUG_MODE] \"Generation of visibility map\"");
	f.write(reinterpret_cast<char *>(image),(height_map_*width_map_)*sizeof(unsigned char));
}