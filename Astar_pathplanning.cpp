#include<ros/ros.h>
#include<iostream>
#include<math.h>
#include<vector>
#include<nav_msgs/OccupancyGrid.h>
#include <unistd.h>
double pi = 3.14;
bool Astar_generate(nav_msgs::OccupancyGrid &map);


struct  nodes{
    bool bObstacle = false;
    bool bVisited = false;
    bool bopen = false;
    float fGlobalGoal;
    float fLocalGoal;  
    int x;              // x,y coordinate
    int y;
    std::vector<nodes*> vecNeighbours;	// Connections to neighbours
    nodes* parent;
};

struct cell{
    float x;
    float y;
};
nav_msgs::OccupancyGrid map;
cell robot_pose;
nodes *nodeStart = nullptr;
nodes *nodeEnd = nullptr;
nodes *node = nullptr;

// void get_map(const nav_msgs::OccupancyGrid::ConstPtr& data){
//     map.data = data->data;
//     map.header = data->header;
//     map.info = data->info;
// }

int main(int argc, char** argv){
    // clock_t start, end;  
    // double time_use;      
    ros::init(argc, argv, "Astar");
    ros::NodeHandle n;
    // ros::Subscriber map_sub = n.subscribe("map", 1000, get_map);
    ros::Publisher map_data = n.advertise<nav_msgs::OccupancyGrid>("map", 50);
    map.info.resolution = 1;
    map.info.width = int(50/map.info.resolution);
    map.info.height = int(50/map.info.resolution);
    int map_size = map.info.width * map.info.height;
    map.data.resize(map.info.width*map.info.height);
    int a = 1;
    // robot_pose.x = 10/map.info.resolution;
    // robot_pose.y = 10/map.info.resolution;

    ros::Rate r(2);
    while(n.ok()){
        ros::spinOnce();
        if(a == 1){
            Astar_generate(map);
            a++;
        }
        map_data.publish(map);
        r.sleep();
    }
}

float distance(nodes* a, nodes* b){
    return sqrtf((a->x - b->x)*(a->x - b->x) + (a->y - b->y)*(a->y - b->y));
}

float heuristic(nodes* a, nodes* b){
    distance(a,b);
}

bool compareByLength(const nodes* a, const nodes* b)
{
    return a->fGlobalGoal < b->fGlobalGoal;
}

bool Astar_generate(nav_msgs::OccupancyGrid &map){
    node = new nodes[map.info.width * map.info.height];                         // tao 1 mang co kieu du lieu la nodes gom x*y phan tu
    // nodeStart = &node[(map.info.height / 2) * map.info.width + 1];              // Phai tao mang truoc khi tro den 
	// nodeEnd = &node[(map.info.height / 2) * map.info.width + map.info.width-2];
    for (int x = 0; x < map.info.width; x++)
		for (int y = 0; y < map.info.height; y++)
		{   
            node[y * map.info.width + x].x = x; 
			node[y * map.info.width + x].y = y;
		    node[y*map.info.width + x].fGlobalGoal = INFINITY;
			node[y*map.info.width + x].fLocalGoal = INFINITY;
		}

    for (int x = 0; x < map.info.width; x++)
			for (int y = 0; y < map.info.height; y++)
			{
				if(y>0)
					node[y*map.info.width + x].vecNeighbours.push_back(&node[(y - 1) * map.info.width + (x + 0)]);
				if(y<map.info.height-1)
					node[y*map.info.width + x].vecNeighbours.push_back(&node[(y + 1) * map.info.width + (x + 0)]);
                if(y > 0 && x > 0)
                    node[y*map.info.width + x].vecNeighbours.push_back(&node[(y - 1) * map.info.width + (x - 1)]);
                if(y > 0 && x < (map.info.width-1))
                    node[y*map.info.width + x].vecNeighbours.push_back(&node[(y - 1) * map.info.width + (x + 1)]);
				if(x>0)
					node[y*map.info.width + x].vecNeighbours.push_back(&node[(y + 0) * map.info.width + (x - 1)]);
				if(x<map.info.width-1)
					node[y*map.info.width + x].vecNeighbours.push_back(&node[(y + 0) * map.info.width + (x + 1)]);
                if(x < (map.info.width-1) && y < (map.info.height-1))
                    node[y*map.info.width + x].vecNeighbours.push_back(&node[(y + 1) * map.info.width + (x + 1)]);
                if(y < (map.info.height-1) && x >0)
                    node[y*map.info.width + x].vecNeighbours.push_back(&node[(y + 1) * map.info.width + (x - 1)]);  
			}
    nodeStart = &node[210];
	nodeEnd = &node[2483];
    nodes *nodeCurrent = nodeStart;
    nodeStart->fGlobalGoal = heuristic(nodeCurrent, nodeEnd);
    nodeStart->fLocalGoal = 0.0f;
    std::vector< nodes*> listNotTestedNodes;   // tao 1 vector con tro co kieu du lieu struct
    listNotTestedNodes.push_back(nodeCurrent);
    int a = 0;
    bool first_time = true;

    while(!listNotTestedNodes.empty() && nodeCurrent != nodeEnd){
        for(auto nodeNeighbour : nodeCurrent->vecNeighbours){       //nodeCurrent is pointing to itself not nodeNeighbour
            if (!nodeNeighbour->bVisited && nodeNeighbour->bObstacle == false && nodeNeighbour->bopen == false){
				listNotTestedNodes.push_back(nodeNeighbour);
                nodeNeighbour->bopen = true;
            }

            float fPossiblyLowerGoal = nodeCurrent->fLocalGoal + distance(nodeCurrent, nodeNeighbour);
            if (fPossiblyLowerGoal < nodeNeighbour->fLocalGoal){
                nodeNeighbour->fLocalGoal = fPossiblyLowerGoal;
                nodeNeighbour->fGlobalGoal = heuristic(nodeNeighbour,nodeEnd) + nodeNeighbour->fLocalGoal;
                nodeNeighbour->parent = nodeCurrent;
            }
        }
        
        sort(listNotTestedNodes.begin(),listNotTestedNodes.end(),compareByLength);          // sap xep lai mang theo fGlobalGoal
        nodeCurrent = listNotTestedNodes.front();
        if(first_time){
            nodeStart->parent = nodeCurrent;
            first_time = false;
        }
        nodeCurrent->bVisited = true;
        listNotTestedNodes.erase(listNotTestedNodes.begin());
    }
    nodeCurrent = nodeEnd;
    map.data[nodeStart->x + nodeStart->y * map.info.width] = 100;
    while(nodeCurrent != nodeStart){
        map.data[nodeCurrent->x + nodeCurrent->y * map.info.width] = 100;
        nodeCurrent = nodeCurrent->parent;
        // ROS_INFO("%d, %d",nodeCurrent->x, nodeCurrent->y);
    }
    return true;
 }
