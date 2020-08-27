#include "astar/MainRosAstar.h"

Eigen::Vector2f initialPose(0,0);


MainRosAstar::MainRosAstar(int argc,char ** argv)
{
    astar=new Astar(20,ASTAR_M);
    occmap=new OccMap(0.05,0.2);

    ros::init(argc,argv,"astar");
    nh = new ros::NodeHandle("~");
    coverageAreaPub= nh->advertise<visualization_msgs::Marker>("/coverageArea",1);
    initial_marker_pub = nh->advertise<visualization_msgs::Marker>("/initial_marker", 1);
    goal_marker_pub = nh->advertise<visualization_msgs::Marker>("/goal_marker", 1);
    pathPub= nh->advertise<visualization_msgs::Marker>("/path",1);
    costmapPub=nh->advertise<visualization_msgs::Marker>("/costmap",1);
    testmapPub=nh->advertise<visualization_msgs::Marker>("/testmap",1);
    initialSub = nh->subscribe("/initialpose",10, &MainRosAstar::initialCallBack, this);
    mapSub=nh->subscribe("/map",10,&MainRosAstar::mapCallBack,this);
    move_base_goalSub=nh->subscribe("/move_base_simple/goal",10,&MainRosAstar::goalCallBack,this);
}

//无论是选择起点还是终点，都是先运行mapCallBack,所以不能在这里调用InitStart
void MainRosAstar::mapCallBack(nav_msgs::OccupancyGridConstPtr map){
    occmap->setMap(map);
    astar->InitMap(occmap,testmapPub);
    vis_costmap(occmap,costmapPub);
}


void MainRosAstar::initialCallBack(const geometry_msgs::PoseWithCovarianceStampedConstPtr& initial)
{
    initialPose << initial->pose.pose.position.x, initial->pose.pose.position.y;
    cout << endl<< "inital : "<< initialPose.transpose()<<endl;
    astar->setStart(initialPose);
    astar->InitStart();

    visualization_msgs::Marker marker;
    uint32_t shape = visualization_msgs::Marker::ARROW;

    marker.action = visualization_msgs::Marker::ADD;
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "initial";
    marker.id = 1;

    marker.type = shape;
    marker.scale.x = 0.8;
    marker.scale.y = 0.25;
    marker.scale.z = 0.1;

    // Set the color -- set alpha to something non-zero!
    marker.color.r = 0.4f;
    marker.color.g = 0.0f;
    marker.color.b = 0.5f;
    marker.color.a = 0.7;
    marker.pose.position.x = initial->pose.pose.position.x;
    marker.pose.position.y = initial->pose.pose.position.y;
    marker.pose.position.z = initial->pose.pose.position.z;
    marker.pose.orientation.x = initial->pose.pose.orientation.x;
    marker.pose.orientation.y = initial->pose.pose.orientation.y;
    marker.pose.orientation.z = initial->pose.pose.orientation.z;
    marker.pose.orientation.w = initial->pose.pose.orientation.w;
    marker.lifetime = ros::Duration();
    initial_marker_pub.publish(marker);
}

void MainRosAstar::goalCallBack(geometry_msgs::PoseStampedConstPtr goal)
{
    cout<<"goal : "<<Vector2f(goal->pose.position.x,goal->pose.position.y).transpose()<<endl;
    astar->vistedArea.clear();
    astar->setGoal(Vector2f(goal->pose.position.x,goal->pose.position.y));
    if(astar->goalNode->isObstacle){
        cout<<"target is in Obstacle!"<<endl;
        return ;
    }
    bool succ=astar->solve(coverageAreaPub);
    if (succ) {
        vector<Vector2f> path;
        path.push_back(Vector2f(astar->goalNode->pw[0] , astar->goalNode->pw[1]));
        auto temp=astar->goalNode;
        while (temp->Parent != nullptr) {
            temp= temp->Parent;
            path.push_back(Vector2f(temp->pw[0] , temp->pw[1]));
        }
        vis_path(path , pathPub);
    }
    astar->resetMap();
    visualization_msgs::Marker marker;
    uint32_t shape = visualization_msgs::Marker::ARROW;

    marker.action = visualization_msgs::Marker::ADD;

    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "goal";
    marker.id = 1;

    marker.type = shape;
    marker.scale.x = 0.8;
    marker.scale.y = 0.25;
    marker.scale.z = 0.1;

    marker.color.r = 0.8f;
    marker.color.g = 0.2f;
    marker.color.b = 0.1f;
    marker.color.a = 0.7;
    marker.pose.position.x = goal->pose.position.x;
    marker.pose.position.y = goal->pose.position.y;
    marker.pose.position.z = goal->pose.position.z;
    marker.pose.orientation.x = goal->pose.orientation.x;
    marker.pose.orientation.y = goal->pose.orientation.y;
    marker.pose.orientation.z = goal->pose.orientation.z;
    marker.pose.orientation.w = goal->pose.orientation.w;

    marker.lifetime = ros::Duration();
    goal_marker_pub.publish(marker);
}

void MainRosAstar::run()
{
    ros::Rate rate(100);
    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
}
