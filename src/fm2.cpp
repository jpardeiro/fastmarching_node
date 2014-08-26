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
#include "../include/fastmarching/fmm/fastmarching.hpp"
#include "../include/fastmarching/fm2/fm2.hpp"
#include "../include/fastmarching/fmdata/fmfibheap.hpp"
#include "../include/fastmarching/fmdata/fmpriorityqueue.hpp"
#include "../include/fastmarching/fmdata/fmdaryheap.hpp"
#include "../include/fastmarching/fmdata/fmdaryheap.hpp"
#include "../include/fastmarching/io/maploader.hpp"
#include "../include/fastmarching/io/gridplotter.hpp"
#include "../include/fastmarching/io/gridwriter.hpp"
#include "../include/fastmarching/io/gridpoints.hpp"

#include <sstream>
#include "fastmarching/map.h"
#include "fastmarching/pathFM.h"
#include "fastmarching/dims.h"

fastmarching::map occ;

int row;
int col;
int ndims;
float resolution = -1;

void chatterCallback_occupancy(const fastmarching::map::ConstPtr &msg)
{
    occ.occupancyGrid = msg -> occupancyGrid;
    occ.gridSize = msg -> gridSize;
    resolution = msg -> resolution;
    ndims = msg -> ndims;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "FM2_Path_Planner");

  bool enable_fm = true;

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("map_FM", 1000, chatterCallback_occupancy);

  ros::Publisher path_pub = n.advertise<fastmarching::pathFM>("path_FM", 1000);

  ros::Duration seconds_sleep(1);

  while (ros::ok())
  {

    if (resolution != -1 && enable_fm)
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

        std::array<int, ndims_> coords_init, coords_goal;
        GridPoints::selectMapPoints(grid, coords_init, coords_goal);

        int idx, goal;
        grid.coord2idx(coords_init, idx);
        init_point.push_back(idx);
        grid.coord2idx(coords_goal, goal);

        typedef typename std::vector< std::array<double, ndims_> > Path; // A bit of short-hand.
        Path path;

        std::vector <double> path_velocity; // Velocity of the path

        FastMarching2< nDGridMap<FMCell, ndims_>, Path > fm2;

        fm2.setEnvironment(&grid);
        fm2.setInitialAndGoalPoints(init_point, fm2_sources, goal);
        fm2.computeFM2();

        fm2.computePath(&path, &path_velocity);

        GridPlotter::plotMapPath(grid,path);

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

    }
    ros::spinOnce();
    seconds_sleep.sleep();
  }
  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */


  return 0;
}
