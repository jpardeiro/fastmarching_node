#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <cmath>
#include <array>
#include <string>
#include <algorithm>

#include "../include/fastmarching/fmdata/fmcell.h"
#include "../include/fastmarching/ndgridmap/ndgridmap.hpp"
#include "../include/fastmarching/console/console.h"
#include "../include/fastmarching/fm2star/fm2star.hpp"
#include "../include/fastmarching/fm2/fm2.hpp"
#include "../include/fastmarching/fmdata/fmdaryheap.hpp"
#include "../include/fastmarching/io/gridplotter.hpp"

#include <sstream>
#include "fastmarching/map.h"
#include "fastmarching/pathFM.h"
#include "fastmarching/dims.h"
#include "fastmarching/InitAndGoal.h"

fastmarching::map occ;

int row;
int col;
int ndims;
float resolution = -1;

bool enable_points = false;
int init = 0;
int goal = 0;

void chatterCallback_occupancy(const fastmarching::map::ConstPtr &msg)
{
    occ.occupancyGrid = msg -> occupancyGrid;
    occ.gridSize = msg -> gridSize;
    resolution = msg -> resolution;
    ndims = msg -> ndims;
}

void chatterCallback_points(const fastmarching::InitAndGoal::ConstPtr &msg)
{
    enable_points = msg -> enable;
    init = msg -> init;
    goal = msg -> goal;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "FM2_Star_Path_Planner");

  bool enable_fm = true;

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("map_FM", 1000, chatterCallback_occupancy);
  ros::Subscriber sub_points = n.subscribe("Init_and_Goal_Points", 1000, chatterCallback_points);

  ros::Publisher path_pub = n.advertise<fastmarching::pathFM>("path_FM", 1000);

  ros::Duration seconds_sleep(1);

  while (ros::ok())
  {

    if (resolution != -1 && enable_fm && enable_points)
    {
        fastmarching::pathFM pathFM;
        constexpr int ndims_ = 2;
        nDGridMap<FMCell, ndims_> grid;

        std::vector<int> fm2_sources;

        std::array<int, ndims_> dimsize;

        for (int i = 0; i < ndims; i++)
            dimsize[i] = occ.gridSize[i];

        grid.resize(dimsize);

        for (int i = 0; i < occ.occupancyGrid.size(); i++)
        {
            grid.getCell(i).setOccupancy(occ.occupancyGrid[i]);

            if (!occ.occupancyGrid[i])
                fm2_sources.push_back(i);
        }

        grid.setLeafSize(resolution);

        std::vector<int> init_point;
        init_point.push_back(init);

        typedef typename std::vector< std::array<double, ndims_> > Path; // A bit of short-hand.
        Path path;

        std::vector <double> path_velocity; // Velocity of the path

        FastMarching2Star< nDGridMap<FMCell, ndims_> > fm2star;

        fm2star.setEnvironment(&grid);
        fm2star.setInitialAndGoalPoints(init_point, fm2_sources, goal);
        fm2star.computeFM2Star();

        fm2star.computePath(&path, &path_velocity);

        for (int i = (int)path_velocity.size()-1; i > 0; i--)
        {
            fastmarching::dims dims;

            for (int j = 0; j < (int)path[i].size(); j++)
                dims.dims.push_back(path[i][j]*resolution);

            pathFM.positions.push_back(dims);
            pathFM.vel_rate.push_back(path_velocity[i]);
        }

        pathFM.enable = true;

        path_pub.publish(pathFM);

        ROS_INFO("Path published");

        enable_fm = false;

        GridPlotter::plotMapPath(grid,path);

    }
    ros::spinOnce();
    seconds_sleep.sleep();
  }
  
  return 0;
}
