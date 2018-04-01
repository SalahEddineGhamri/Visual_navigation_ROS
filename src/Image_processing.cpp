/***********************************************************************/
/*                       Image_Processing.cpp                          */
/***********************************************************************/
/* -VERSION: ROS_Ubuntu 14.04                                          */
/* -AUTHOR:  GUILLAUME Sylvain                                         */
/* -LAST_MODIFICATION: 12/2017                                         */
/***********************************************************************/

#include "Image_processing.hpp"

vector<int> erode (const vector<int> map, int size_Filter,int width,int height)
{
	vector<int> data_res;
	data_res.resize(map.size());
	for(unsigned int i=0;i<height;++i)
	{
		for(unsigned int j =0;j<width;++j)
		{
			data_res[((i*width)+j)] = map[((i*width)+j)];
			for (unsigned int m = i-size_Filter; m<i+size_Filter ; ++m)
			{
				for (unsigned int n = j-size_Filter;n<j+size_Filter; ++n)
				{
					if(m>0 && m<height && n>0 && n<width)
					{
						if(data_res[((i*width)+j)] == 100 && map[((m*width)+n)] == 0)
							data_res[((i*width)+j)] = 0;
					}
				}
			}
		}
	}
	return data_res;
}




vector<int> erode_with_Filter_square (const vector<int> map, const vector<int> filter,int width_map,int height_map,int width_filter)
{
	vector<int> data_res;
	data_res.resize(map.size());

	int mid_filter,m_map,n_map;
	mid_filter = floor(width_filter/2);

	for(unsigned int i=0;i<height_map;++i)
	{
		for(unsigned int j =0;j<width_map;++j)
		{
			data_res[((i*width_map)+j)] = map[((i*width_map)+j)];
			if(data_res[((i*width_map)+j)] == 100)
			{
				for (unsigned int m = 0; m<width_filter ; ++m)
				{

					for (unsigned int n = 0;n<width_filter; ++n)
					{
						m_map = i+(m - mid_filter);
						n_map = j+(n - mid_filter); 
						if(m_map>0 && m_map<height_map && n_map>0 && n_map<width_map)
						{
							if( filter[(m*width_filter)+n]==100 && map[((m_map*width_map)+n_map)] == 0 )
								data_res[((i*width_map)+j)] = 0;
						}
					}
				}
			}
		}
	}
	return data_res;
}




vector<int> dilate (const vector<int> map, int size_Filter,int width,int height)
{
	vector<int> data_res;
	data_res.resize(map.size());
	for(unsigned int i=0;i<height;++i)
	{
		for(unsigned int j =0;j<width;++j)
		{
			data_res[((i*width)+j)] = map[((i*width)+j)];
			for (unsigned int m = i-size_Filter; m<i+size_Filter ; ++m)
			{
				for (unsigned int n = j-size_Filter;n<j+size_Filter; ++n)
				{
					if(m>0 && m<height && n>0 && n<width)
					{
						if(data_res[((i*width)+j)] == 0 && map[((m*width)+n)] == 100)
							data_res[((i*width)+j)] = 100;
					}
				}
			}
		}
	}
	return data_res;
}



vector<int> dilate_with_Filter_square (const vector<int> map, const vector<int> filter,int width_map,int height_map,int width_filter)
{
	vector<int> data_res;
	data_res.resize(map.size());

	int mid_filter,m_map,n_map;
	mid_filter = floor(width_filter/2);

	for(unsigned int i=0;i<height_map;++i)
	{
		for(unsigned int j =0;j<width_map;++j)
		{
			data_res[((i*width_map)+j)] = map[((i*width_map)+j)];
			if(data_res[((i*width_map)+j)] == 0)
			{
				for (unsigned int m = 0; m<width_filter ; ++m)
				{

					for (unsigned int n = 0;n<width_filter; ++n)
					{
						m_map = i+(m - mid_filter);
						n_map = j+(n - mid_filter); 
						if(m_map>0 && m_map<height_map && n_map>0 && n_map<width_map)
						{
							if( filter[(m*width_filter)+n]==100 && map[((m_map*width_map)+n_map)] == 100 )
								data_res[((i*width_map)+j)] = 100;
						}
					}
				}
			}
		}
	}
	return data_res;
}




vector<int> create_Filter_Turtlebot2(float size_turtlebot2,float resolution)
{
	int size_filter = ceil(size_turtlebot2/resolution);
	int mid_filter,x,y,radius;

	//filter can not be pair
	if(size_filter % 2 == 0)
		size_filter++;

	mid_filter = floor(size_filter/2);
	radius = (size_filter/2)*(size_filter/2);

	vector<int> filter;
	filter.resize(size_filter*size_filter);

	// Drawing filled circles
	for(unsigned int i =0; i<size_filter;++i)
	{
		y = i- mid_filter;
		for(unsigned int j =0; j<size_filter;++j)
		{
			x = j- mid_filter;
			if((x*x+y*y) <= (mid_filter*mid_filter))
				filter[((i*size_filter)+j)] =100;
			else
				filter[((i*size_filter)+j)] =0;
		}
	}

	//Debug Mod
	// Display the filter
	/*std::cout<<"Filter_turtlebot\n";
	for(unsigned int i =0; i<size_filter;++i)
	{
		std::cout<<"\n";
		for(unsigned int j =0; j<size_filter;++j)
		{
				std::cout<<filter[((i*size_filter)+j)]<<" ";

		}
	}*/

	return filter;
}



vector<int> open_Image_processing (const vector<int> map, int size_Filter,int width,int height)
{
	vector<int> res = erode(map,size_Filter,width,height);
	res = dilate(res,size_Filter,width,height);
	return res;
}



vector<int> close_Image_processing (const vector<int> map, int size_Filter,int width,int height)
{
	vector<int> res = dilate(map,size_Filter,width,height);
	res = erode(res,size_Filter,width,height);
	return res;
}
