/***********************************************************************/
/*                            Map_Main.cpp                             */
/***********************************************************************/
/* -VERSION: ROS_Ubuntu 14.04                                          */
/* -AUTHOR:  TURTLE TEAM                                               */
/* -LAST_MODIFICATION: 03/2017                                         */
/***********************************************************************/


# include "Target.hpp"
# include "Map_Gui.hpp"
# include "Map_node.hpp"
# include "Line.hpp"
# include "Image_processing.hpp"
# include "Point_Cloud.hpp"
# include "Graph.hpp"
# include "Set_Marker.hpp"
# include "Recherche_Main.hpp"
# include "Follow_Path.hpp"
# include "Bumper.hpp"
# include "Goal.hpp"
# include "Set_Start_Point.hpp"
# include "Location.hpp"

# include "Smooth_Path.cpp"

# define INIT_STATE 0
# define ENSLAVEMENT_STATE 1
# define PATH_GENERATION_STATE 2
# define MOVEMENT_STATE 3
# define OBSTACLE_POSITIONNING_STATE 4
# define WAIT_STATE 5
# define RESOLUTION_BEZIER 10


static int current_State = INIT_STATE;





void enslavement(ros::NodeHandle nh, Recherche_Main recherche, Location location){
	ROS_INFO("... RESHEARCH ...");
	recherche.start(nh);
	
	current_State = PATH_GENERATION_STATE;
}







vector<vector<float> > path_Generation(Map_node node, Map_Gui mapgui, vector<Target> list_Markers, Target start_target, Target end_target){

	vector<Target> list_Target;

	vector<float> orientation ;
	orientation.push_back(0.0f);
	orientation.push_back(0.0f);
	orientation.push_back(-1.57f);
	orientation.push_back(1.0f);

	vector<float> color2;
	color2.push_back(0.0);
	color2.push_back(0.0);
	color2.push_back(0.0);

	vector<float> green;
	green.push_back(0.0);
	green.push_back(1.0);
	green.push_back(0.0);


    vector<float> start_point = start_target.get_Position();


	vector<float> scale;
	scale.push_back(0.5f);
	scale.push_back(1.0f);
	scale.push_back(0.0f);

	ROS_INFO("... GENERATION PRM ...");
	Point_Cloud p_C = Point_Cloud(node, list_Markers, start_point);
	list_Target = p_C.create_Point_Cloud();

	end_target.set_Id(list_Target.size()+1+list_Markers.size());
	start_target.set_Id(list_Target.size()+list_Markers.size());
	
	list_Target.push_back(start_target);
	list_Target.push_back(end_target);

	ROS_INFO("... DISPLAY TARGETS ...");
	mapgui.add_List_Target(0,list_Target);
	
	
	ROS_INFO("... GENERATION LINKS ...");
	vector<Target>::iterator it1, it2;
	for(int i=0; i<list_Target.size()-1;++i)
	{
		for (int j=i+1;j<list_Target.size();++j)
		{
			Target a = list_Target[i];
			Target b = list_Target[j];
			if(!node.is_intersection(a.get_Position(),b.get_Position(),2.5))
			{
				list_Target[i].add_Son(j);
				list_Target[j].add_Son(i);
				/* ROS_INFO("... DISPLAY GRAPH ...")
				mapgui.add_Line(i*list_Target.size()+j,list_Target[i],list_Target[j],color2);*/
			}
		}
	}

	ROS_INFO("... A* ...");
	Graph graph = Graph(start_target, end_target, list_Target);
	vector<Target> path = graph.a_Star();

	if(path.size() == 0){
		ROS_INFO(" [ROBOT] \"Impossible de trouver un chemin pour aller du start_point au end_point...\"\n");
		exit(0);
	} else {
		ROS_INFO("... DISPLAY PATH ...");
		mapgui.add_Line_List (0,path, color2);
		ROS_INFO("... GENERATION SMOOTH PATH ...");
		vector<vector<float > > smooth_path = Generator_smooth_Path(path,RESOLUTION_BEZIER);
		ROS_INFO("... DISPLAY SMOOTH PATH ...");
		mapgui.add_Line_List_float (1, smooth_path, green);
		current_State = MOVEMENT_STATE;
		return smooth_path;
	}
	
}



int main(int argc, char **argv)
{
	ros::init(argc,argv,"Map_Main");
	ros::NodeHandle nh;
	
	/*NODE CLASS*/
	Bumper bumper(nh);
	Goal goal(nh);
	Map_node node(nh);
	Map_Gui mapgui(nh);
	Set_Marker sm(nh);
	Recherche_Main recherche(nh);
	Follow_Path followPath(nh);
	Set_Start_Point init_start(nh);
	Location location(nh);
	
	bool is_obstacle = false;
	int ind = 0;
	
	vector<Target> list_Markers;
	vector<Target> list_Arcode;
	vector<vector<float> > path;
	list_Markers = sm.init_Markers();
	list_Arcode= sm.init_ARcode_pose();


	// Add cone of visibility
	for (int j = 0; j < list_Markers.size(); j++)
	{
		mapgui.add_cone_visibility (j+10,list_Arcode[j].get_Orientation()[2],0.5,list_Arcode[j].get_Position(),2.0f);
	}

	
	Target end_target,start_target;

	ros::Rate loop_rate(2);


	vector<vector<float> > obstacle;

	

	while(ros::ok())
	{
		ros::spinOnce();
		switch(current_State){
			case INIT_STATE:
				ROS_INFO("... MAP PROCESSING ...");
				node.close_Map(2);
				//node.open_Map(2);
				node.dilate_Map();
				node.update_Map();
				current_State = WAIT_STATE;
				break;
				
			case ENSLAVEMENT_STATE:
				enslavement(nh,recherche,location);
				break;
				
			case PATH_GENERATION_STATE:
			    start_target = init_start.Create_target_start(0);
				path = path_Generation(node, mapgui, list_Markers,start_target,end_target);
				break;
				
			case MOVEMENT_STATE:
				ROS_INFO("... MOVING ...");
				ind = 0;
				while(ind<path.size() && !is_obstacle)
				{
					is_obstacle = followPath.goToPoint(path[ind]);
					ind++;
				}
				if(!is_obstacle)
				{
				    is_obstacle = followPath.correction_Angles(path[path.size()-1]);
				}
				if(!is_obstacle)
				{
                 is_obstacle = followPath.correction_Dist(path[path.size()-1]);
                }
                if(is_obstacle)
                    current_State = OBSTACLE_POSITIONNING_STATE; 
                else
			        current_State = WAIT_STATE; 
				break;
				
			case OBSTACLE_POSITIONNING_STATE:
			    ROS_INFO("... ADD OBSTACLE ...");
				obstacle = followPath.getPointCloud();
				node.Add_obstacle(obstacle);
				current_State = PATH_GENERATION_STATE;
				break;
				
			case WAIT_STATE:
			    ROS_INFO("... WAIT_STATE ...");
			    end_target = goal.Create_target_goal(0);
			    
			    current_State = ENSLAVEMENT_STATE;
				break;
				
			default:
				ROS_INFO("ERROR UNKNOWN STATE");
				return 0;
				break;
		}		
		loop_rate.sleep();
	}
	return 0;
}
