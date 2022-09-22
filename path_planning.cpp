#include<ros/ros.h>
#include<iostream>
#include<math.h>
#include<vector>
#include<nav_msgs/OccupancyGrid.h>
#include <unistd.h>
#include<geometry_msgs/PoseWithCovarianceStamped.h>
#include<geometry_msgs/PoseStamped.h>
#include<nav_msgs/Path.h>
#include <tf/transform_broadcaster.h>

double pi = 3.14;

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

struct least{
    float a;
    float b;
    float c;
};

struct seed_seg{
    u_int16_t start;
    u_int16_t end;
    least temp;
};



nav_msgs::OccupancyGrid map;
nav_msgs::OccupancyGrid map_test;
std::vector<cell> line;
std::vector<cell> path;
nav_msgs::Path global_path;
cell initial_pose;
cell goal_pose;
int break_point;
bool start = false;
bool stop = false;
nodes *nodeStart = nullptr;
nodes *nodeEnd = nullptr;
nodes *node = nullptr;
bool flag1 = false;
bool flag2 = false;
bool flag3 = false;



void find_nearst_point(nav_msgs::OccupancyGrid &map);
bool init_map(nav_msgs::OccupancyGrid &map);
bool Astar_generate(nav_msgs::OccupancyGrid &map);
float max(nodes* a, nodes* b);
void seed_segment_detection(std::vector<cell> line);
void path_smooth(std::vector<cell> point);
bool check_collision(cell a, cell b, nav_msgs::OccupancyGrid &map,std::vector<cell> &line1);
void breseham(cell a, cell b,nav_msgs::OccupancyGrid &map);



void get_map(const nav_msgs::OccupancyGrid::ConstPtr& data){
    map.info.width = data->info.width;
    map.info.height = data->info.height;
    map.data = data->data;
    map.info.resolution = data->info.resolution;
    // clear it when using Duc's SLAM map
    /////////////////////////////////////////////////////////////////
    map.info.origin.position.x = data->info.origin.position.x + 5;
    map.info.origin.position.y = data->info.origin.position.y + 5;
    map.info.origin.position.z = data->info.origin.position.z;
    /////////////////////////////////////////////////////////////////
    map_test.data = data->data;
    map_test.info = data->info;
    flag1 = true;
}

void get_initial_pose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& data){
    initial_pose.x = data->pose.pose.position.x;
    initial_pose.y = data->pose.pose.position.y;
    nodeStart = &node[int(initial_pose.y/map.info.resolution)*map.info.width + int(initial_pose.x/map.info.resolution)];
    flag2 = true;
    if(nodeStart->bObstacle == true){
        ROS_INFO("make another start position");
        flag2 = false;
    }
                 
}

void get_goal(const geometry_msgs::PoseStamped::ConstPtr& data){
    goal_pose.x = data->pose.position.x;
    goal_pose.y = data->pose.position.y;
    nodeEnd = &node[int(goal_pose.y/map.info.resolution)*map.info.width + int(goal_pose.x/map.info.resolution)];
    flag3 = true;
    if(nodeEnd->bObstacle ==true){
        ROS_INFO("make another goal position");
        flag3 = false;
    }
    
}

int main(int argc, char** argv){
    // clock_t start, end;  
    // double time_use;      
    ros::init(argc, argv, "Astar");
    ros::NodeHandle n;
    ros::Subscriber map_sub = n.subscribe("map", 1000, get_map);
    ros::Subscriber pose = n.subscribe("/initialpose", 1000, get_initial_pose);
    ros::Subscriber goal = n.subscribe("/move_base_simple/goal", 1000, get_goal);   
    ros::Publisher map_data = n.advertise<nav_msgs::OccupancyGrid>("path", 50);
    ros::Publisher global_trajectory = n.advertise<nav_msgs::Path>("global_path", 50);
    map.header.frame_id = "/map";
    global_path.header = map.header;
    ros::Rate r(2);
    while(n.ok()){
        ros::spinOnce();
        break_point = 0;
        if(flag1 == true){
                    int map_size = map.info.width * map.info.height;
                    map.data.resize(map_size);
                    // clear it if using Duc's SLAM map
                    /////////////////////////////////////////////////////////////////////////
                    // for (int x = 0; x < map.info.width; x++){
                    //     for (int y = 0; y < map.info.height; y++){
                    //         if(uint8_t(map.data[y * map.info.width + x]) == 120){
                    //             map.data[y * map.info.width + x] = 0;
                    //         }
                    //         if(uint8_t(map.data[y * map.info.width + x]) == 150){
                    //             map.data[y * map.info.width + x] = 0;
                    //         }
                    //         if(uint8_t(map.data[y * map.info.width + x]) <= 55){
                    //             map.data[y * map.info.width + x] = 0;
                    //         }
                    //         if(uint8_t(map.data[y * map.info.width + x]) > 55){
                    //             map.data[y * map.info.width + x] = 100;
                    //         }
                    //     }
                    // }
                    /////////////////////////////////////////////////////////////////////////
                    find_nearst_point(map);
                    init_map(map);
                    ROS_INFO("Ready for path planning");
                    map_sub.shutdown();
                    flag1 = false;
        }

        if(flag2 == true && flag3 == true){
            Astar_generate(map);
            seed_segment_detection(line);
            flag2 = false;
        }
        map_data.publish(map);
        // global_trajectory.publish(global_path);
        r.sleep();
    }
}

float distance(nodes* a, nodes* b){
    return sqrtf((a->x - b->x)*(a->x - b->x) + (a->y - b->y)*(a->y - b->y));
}

float heuristic(nodes* a, nodes* b){
    return (max(a,b) + 2*distance(a,b));
}

bool compareByLength(const nodes* a, const nodes* b)
{
    return a->fGlobalGoal < b->fGlobalGoal;
}

bool Astar_generate(nav_msgs::OccupancyGrid &map){
    ROS_INFO("processing");
    line.clear();
    nodes *nodeCurrent = nodeStart;
    nodeStart->fGlobalGoal = heuristic(nodeCurrent, nodeEnd);
    nodeStart->fLocalGoal = 0.0f;
    std::vector< nodes*> listNotTestedNodes;   // tao 1 vector con tro co kieu du lieu struct
    listNotTestedNodes.push_back(nodeCurrent);
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
    cell list_of_path;
    nodeCurrent = nodeEnd;
    list_of_path.x = nodeCurrent->x;
    list_of_path.y = nodeCurrent->y;
    // path.pose.position.z = 0;
    // geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(path.pose.position.z);
    // path.pose.orientation = odom_quat;
    // global_path.poses.push_back(path);
    line.push_back(list_of_path);
    map.data[nodeStart->x + nodeStart->y * map.info.width] = 20;
    while(nodeCurrent != nodeStart){
        map.data[nodeCurrent->x + nodeCurrent->y * map.info.width] = 20;
        // ROS_INFO("%d, %d", nodeCurrent->x, nodeCurrent->y);
        list_of_path.x = nodeCurrent->x;
        list_of_path.y = nodeCurrent->y;
        // path.pose.orientation = odom_quat;
        line.push_back(list_of_path);
        nodeCurrent = nodeCurrent->parent;
    }
    list_of_path.x = nodeStart->x;
    list_of_path.y = nodeStart->y;
    line.push_back(list_of_path);
    // ROS_INFO("%d %d", nodeStart->x, nodeStart->y);
    // ROS_INFO("%f %f",line[line.size()-1].x, line[line.size()-1].y);
    ROS_INFO("done");
    return true;
 }

bool init_map(nav_msgs::OccupancyGrid &map){
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

            //broaden obstacles
            if(map.data[y * map.info.width + x] == 100){
                for(int i = -(31/2); i <= (31/2); i++){
                    for(int j = -(31/2); j <= (31/2); j++){
                        int a = x + i;
                        int b = y + j;
                        if((a >= 0) && (a < map.info.width-1) && (b >= 0) && (b < map.info.height-1)){
                            node[b * map.info.width + a].bObstacle = true; 
                        }
                    }
		        }
            }
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

                // if(node[y*map.info.width + x].bObstacle == true){
                //     map.data[y * map.info.width + x] = 50;
                // }
			}
    return true;
 }

float max(nodes* a, nodes* b){
    float delta_x = abs(a->x - b->x);
    float delta_y = abs(a->y - b->y);
    if(delta_x > delta_y){
        return delta_x;
    }
    else{
        return delta_y;
    }
}

void find_nearst_point(nav_msgs::OccupancyGrid &map){
    // ROS_INFO("hello");
    bool x;
    int count = 0;
    for (int x = 0; x < map.info.width; x++){
			for (int y = 0; y < map.info.height; y++){
                if(map.data[y * map.info.width + x] == 100){
                    for(int i = -1; i <= 1; i++){
                        for(int j = -1; j <= 1; j++){
                            int a = x + i;
                            int b = y + j;
                            if((map.data[b * map.info.width + a] == 100) && (a >= 0) && (a < map.info.width-1) && (b >= 0) && (b < map.info.height-1)){
                               count++;
                            }   
                        }
                    }
                    if(count < 3){
                        map.data[y * map.info.width + x] = 0;
                    }
                    count = 0;
                    }
                }
            }
    }

least fitting_tagent_line(int start, int end){
    least temp;
    double w1 = 0, w2 = 0, w3 = 0;
    int n = end - start + 1;
    double mid1 = 0;
    double mid2 = 0;
    double mid3 = 0;
    double mid4 = 0;
    double mid5 = 0;


    for(int i= start ; i <= end; i++){
        mid1 += line[i].x;
        mid2 += line[i].y;
        mid3 += pow(line[i].x,2);
        mid4 += pow(line[i].y,2);
        mid5 += (line[i].x * line[i].y);
    }
    w1 = n*mid5-mid1*mid2;
	w2 = mid2*mid2-n*mid4-mid1*mid1+n*mid3;
	w3 = mid1*mid2-n*mid5;
    //ROS_INFO("%f, %f, %f",w1,w2,w3);
	//ax+by+c = 0 等价于 y = kx + b;kx-y + b = 0 //a = k,c = b,b=-1
	if(w1==0 && (line[start].x == line[end].x))
	{
		temp.a = -1;
		temp.b = 0;
		temp.c = mid1/n;
	}
	if(w1==0 && (line[start].y == line[end].y))
	{
		temp.a = 0;
		temp.b = -1;
		temp.c = mid2/n;
	}
	else
	{
		temp.a = (-w2+sqrt(w2*w2-4*w1*w3))/2.0/w1;
		temp.b = -1;
		temp.c = (mid2-temp.a*mid1)/n;
	}
    // ROS_INFO("%f,%f,%f",temp.a, temp.b, temp.c);
    return temp;
}

float dist_point_to_line(least temp, cell point){
    float ts = abs(temp.a * point.x + temp.b* point.y + temp.c);
    float ms = sqrt(pow(temp.a,2)+pow(temp.b,2));
    return ts/ms;
}

void seed_segment_detection(std::vector<cell> line){
    std::vector<cell> point;
    least temp;
    int j,k;
    double d2;
    point.push_back(line[break_point]);
    for(int i = break_point; i < line.size()-2; i++){
        bool flag = true;
        j = 1 + i;
        for(k = j; k < line.size(); k++){ 
            temp = fitting_tagent_line(i,k);
            d2 = dist_point_to_line(temp, line[k]);
            if(d2 >= 0.5){
                flag = false;
                // breseham(line[break_point],line[k],map);
                // map.data[line[k].x + line[k].y * map.info.width] = 100;
                // point.push_back(line[break_point]);
                point.push_back(line[k]);
                break_point = k;
                ROS_INFO("out of condition");
                // ROS_INFO("%d, %d",i,k);
                i = break_point;
                break;
            }

            if(k >= (line.size()-1)){
                // ROS_INFO("hello");
                // map.data[line[k].x + line[k].y * map.info.width] = 100;
                // point.push_back(line[break_point]);
                point.push_back(line[k]);
                // ROS_INFO("%d, %d",i,k);
                break_point = k + 1;
                i = break_point;
                break;
            }
        }
    }
    // for(int i =0; i<point.size();i++){
    //     map.data[point[i].x + point[i].y * map.info.width] = 100;
    //     ROS_INFO("%f %f", point[i].x , point[i].y);
    // }
    path_smooth(point);
}

bool check_collision(cell a, cell b, nav_msgs::OccupancyGrid &map,std::vector<cell> &line1){
    // ROS_INFO("hello");
    nodes *node_present = nullptr;
    std::vector<cell> test;
	cell cell;
	if (b.y>a.y){
		cell = a;
		a = b;
		b = cell;
	}
	if ((b.y != a.y)&&(a.x>b.x)) for(int i = b.y;i < a.y;i++){

 		for (int j = (a.x + ((b.x-a.x)*(a.y-i)/(a.y-b.y)));j <= (a.x + ((b.x-a.x)*(a.y-(i+1))/(a.y-b.y)));j++){
 			cell.y = i;
 			cell.x = j;
            // node_present = &node[int(cell.y) * map.info.width + int(cell.x)];
            // if(node_present->bObstacle) {return true;}
            ROS_INFO("%d",map.data[cell.x + cell.y * map.info.width]);
            if(map.data[cell.x + cell.y * map.info.width] == 100) {return true;}
 			test.push_back(cell);
 		}
 	}
 	else if ((b.y != a.y)&&(a.x<=b.x)) for(int i=b.y;i<a.y;i++){
 		for (int j = (a.x + ((b.x-a.x)*(a.y-i-1)/(a.y-b.y)));j <= (a.x + ((b.x-a.x)*(a.y-i)/(a.y-b.y)));j++){
 			cell.y = i;
 			cell.x = j;
            node_present = &node[int(cell.y) * map.info.width + int(cell.x)];
            ROS_INFO("%d",map.data[cell.x + cell.y * map.info.width]);
            if(map.data[cell.x + cell.y * map.info.width] == 100) {return true;}
 			test.push_back(cell);
 		}

 	}
 	else {
 		if(a.x<b.x){
 			for(int i =a.x;i<=b.x;i++){
 				cell.y = a.y;
 				cell.x = i;
                node_present = &node[int(cell.y) * map.info.width + int(cell.x)];
                ROS_INFO("%d",map.data[cell.x + cell.y * map.info.width]);
                if(map.data[cell.x + cell.y * map.info.width] == 100) {return true;}
 				test.push_back(cell);
 			}
 		}
 			else if(a.x>=b.x){
 				for(int i = b.x;i<=a.x;i++){
 					cell.y = a.y;
 					cell.x = i;
                    node_present = &node[int(cell.y) * map.info.width + int(cell.x)];
                    ROS_INFO("%d",map.data[cell.x + cell.y * map.info.width]);
                    if(map.data[cell.x + cell.y * map.info.width] == 100) {return true;}
 					test.push_back(cell);
 				}

 			}
 	}
    line1 = test;
    return false;
 }

void path_smooth(std::vector<cell> point){
    std::vector<cell> line1;
    path.clear();
    int i=0;
    bool flag4 = false;
    path.push_back(point[0]);
    while(i<point.size()-1){
        flag4 = false;
        for(int j=i+1;j<point.size();j++){
            ROS_INFO("%d",j);
            if(check_collision(point[i],point[j],map_test,line1)== false){
                ROS_INFO("hello");
                i = j;
                flag4=true;
            }
        }
        if(flag4 == false){
            i++;
        }
        ROS_INFO("%d",i);
        path.push_back(point[i]);
        breseham(point[0], point[i],map); 
        break;
    }
    // ROS_INFO("%ld",path.size());
    // for(int c=0; c < path.size()-1; c++){
    //     // map.data[path[c].x + path[c].y * map.info.width] = 100;
    //     breseham(point[c], point[c+1],map);  
    // }
}

void breseham(cell a, cell b,nav_msgs::OccupancyGrid &map){
    std::vector<cell> line;
	cell cell;
	if (b.y>a.y){
		cell = a;
		a = b;
		b = cell;
	}
	if ((b.y != a.y)&&(a.x>b.x)) for(int i = b.y;i < a.y;i++){

 		for (int j = (a.x + ((b.x-a.x)*(a.y-i)/(a.y-b.y)));j <= (a.x + ((b.x-a.x)*(a.y-(i+1))/(a.y-b.y)));j++){
 			cell.y = i; 
 			cell.x = j;
 			line.push_back(cell);
 		}
 	}
 	else if ((b.y != a.y)&&(a.x<=b.x)) for(int i=b.y;i<a.y;i++){
 		for (int j = (a.x + ((b.x-a.x)*(a.y-i-1)/(a.y-b.y)));j <= (a.x + ((b.x-a.x)*(a.y-i)/(a.y-b.y)));j++){
 			cell.y = i;
 			cell.x = j;
 			line.push_back(cell);
 		}

 	}
 	else {
 		if(a.x<b.x){
 			for(int i =a.x;i<=b.x;i++){
 				cell.y = a.y;
 				cell.x = i;
 				line.push_back(cell);
 			}
 		}
 			else if(a.x>=b.x){
 				for(int i = b.x;i<=a.x;i++){
 					cell.y = a.y;
 					cell.x = i;
 					line.push_back(cell);
 				}

 			}

 	}
    // ROS_INFO("%ld",line.size());
    for(int i = 0; i < line.size(); i++){
        map.data[line[i].x + line[i].y * map.info.width] = 100;
        // ROS_INFO("ff");
    }
 }