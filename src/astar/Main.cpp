#include "map/OccMap.h"
#include "astar/MainRosAstar.h"
int main(int argc,char **argv)
{
    MainRosAstar AstarNode(argc,argv);
    AstarNode.run();
}
