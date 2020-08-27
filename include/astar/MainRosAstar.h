//
// Created by unicorn on 2019/12/28.
//

#ifndef MOTION_PLAN_OCC_MAINROSASTAR_H
#define MOTION_PLAN_OCC_MAINROSASTAR_H
#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Core>
#include <array>
#include <random>
#include <math.h>
#include "map/OccMap.h"
#include "astar/Astar.h"
#include "tools/Viewer.h"
using  namespace std;
using  namespace Eigen;

class MainRosAstar{
public:
    MainRosAstar(int argc,char ** argv);

private:
    ros::Publisher coverageAreaPub,pathPub,costmapPub,testmapPub, initial_marker_pub, goal_marker_pub;
    ros::Subscriber mapSub,move_base_goalSub, initialSub;
    ros::NodeHandle *nh;
    OccMap * occmap;
    Astar *astar;

public:
    void mapCallBack(nav_msgs::OccupancyGridConstPtr map);
    void initialCallBack(const geometry_msgs::PoseWithCovarianceStampedConstPtr& initial);
    void goalCallBack( geometry_msgs::PoseStampedConstPtr ps);

    void run();

};
#endif //MOTION_PLAN_OCC_MAINROSASTAR_H

