#include "ros/ros.h"
#include<unordered_map>
#include "car_driver/AstarPlanner.h"
#include <time.h>
#include "geometry_msgs/PoseArray.h"
#include "nav_msgs/GridCells.h"

int main(int argc, char **argv)
{
        ros::init(argc, argv, "AstarPlannerClient");
        //Initializing map inputs with default values
        /////////////////////////////////////////////
        //EDIT HERE for custom map settings/////////
        ////////////////////////////////////////////
        int row = 20;
        int col = 20;
        int startX = 0;
        int startY= 0;
        int startD = 2; //Key North = 0; West = 1; East = 2; South = 3;
        int goalX = row-2;
        int goalY = col-2;
        int goalD = 2; //Key North = 0; West = 1; East = 2; South = 3;
        int obstacleCount = row*col/25;
        int roughRoadCount = row*col/25;

        //If user inputs args, use them instead
        ROS_INFO("Recieved %d args",argc);
        if (argc == 11)
        {
                ROS_INFO("Creating map with custom parameters!");
                row=atoi(argv[1]);
                col=atoi(argv[2]);
                startX=atoi(argv[3]);
                startY=atoi(argv[4]);
                startD=atoi(argv[5]);
                goalX=atoi(argv[6]);
                goalY=atoi(argv[7]);
                goalD=atoi(argv[8]);
                obstacleCount = atoi(argv[9]);
                roughRoadCount = atoi(argv[10]);
                ROS_INFO("row = %d",row);
                ROS_INFO("col = %d",col);
                ROS_INFO("start = (%d,%d,%d)",startX,startY,startD);
                ROS_INFO("goal = (%d,%d,%d)",goalX,goalY,goalD);
                ROS_INFO("obstaclecount = %d",obstacleCount);
                ROS_INFO("roughRoadcount = %d",roughRoadCount);
        }
        else{
                ROS_INFO("Creating map with default parameters!");
        }

        //Service client and service request message initialization
        ros::NodeHandle n;
        ros::ServiceClient client = n.serviceClient<car_driver::AstarPlanner>("AstarPlanner");
        car_driver::AstarPlanner srv;
        srv.request.mapWidth = row;
        srv.request.mapHeight = col;
        srv.request.start.x = startX;
        srv.request.start.y = startY;
        srv.request.start.d = startD;
        srv.request.goal.x = goalX;
        srv.request.goal.y = goalY;
        srv.request.goal.d = goalD;

        //Create random obstacles and rough road patches
        srand (time(NULL));
        car_driver::Pair p;
        for(int i=0; i<obstacleCount; i++) {
                p.x = (rand()%(row-1));
                p.y = (rand()%(col-1));
                srv.request.obstacles.vect.push_back(p);
        }
        for(int i=0; i<roughRoadCount; i++) {
                p.x = (rand()%(row-1));
                p.y = (rand()%(col-1));
                srv.request.roughRoads.vect.push_back(p);
        }

        //////////////////////////////////////////////////////////////
        //EDIT HERE for custom obstacles and roughRoads///////////////
        //Repeat code block for multiple obstacles and rough Roads////
        /////////////////////////////////////////////////////////////
/*
        srv.request.obstacles.vect.clear();
        p.x = USER_INPUT; //enter x coordinate of obstacle cell
        p.y = USER_INPUT; //enter y coordinate of obstacle cell
        srv.request.obstacles.vect.push_back(p);

        srv.request.roughRoads.vect.clear();
        p.x = USER_INPUT; //enter x coordinate of obstacle cell
        p.y = USER_INPUT; //enter y coordinate of obstacle cell
        srv.request.roughRoads.vect.push_back(p);

 */

        //Creating publishers for rviz display
        ros::Publisher poseArrayPublisher;
        ros::Publisher obstaclesPublisher;
        ros::Publisher roughRoadsPublisher;
        ros::Rate loop_rate(10);
        geometry_msgs::PoseArray message;
        message.header.seq = 0;
        message.header.stamp = ros::Time::now();
        message.header.frame_id = "Car_driver";

        nav_msgs::GridCells obstacleArray;
        obstacleArray.header.seq = 0;
        obstacleArray.header.stamp = ros::Time::now();
        obstacleArray.header.frame_id = "Car_driver";
        obstacleArray.cell_width = 1.0;
        obstacleArray.cell_height = 1.0;
        geometry_msgs::Point point;
        for(auto obs: srv.request.obstacles.vect) {
                point.x = obs.x+0.5;
                point.y = -(obs.y+0.5);
                point.z = 0;
                obstacleArray.cells.push_back(point);
        }

        nav_msgs::GridCells roughRoadsArray;
        roughRoadsArray.header.seq = 0;
        roughRoadsArray.header.stamp = ros::Time::now();
        roughRoadsArray.header.frame_id = "Car_driver";
        roughRoadsArray.cell_width = 1.0;
        roughRoadsArray.cell_height = 1.0;
        for(auto rrs: srv.request.roughRoads.vect) {
                point.x = rrs.x+0.5;
                point.y = -(rrs.y+0.5);
                point.z = 0;
                roughRoadsArray.cells.push_back(point);
        }

        geometry_msgs::Pose pose;
        poseArrayPublisher = n.advertise<geometry_msgs::PoseArray>("/car_driver/poseArray", 100);
        obstaclesPublisher = n.advertise<nav_msgs::GridCells>("/car_driver/obstacleArray",100);
        roughRoadsPublisher = n.advertise<nav_msgs::GridCells>("/car_driver/roughRoadsArray",100);

        std::unordered_map<int,char> intToDirection({{0,'N'},{1,'W'},{2,'E'},{3,'S'}});
        std::unordered_map<int,std::string> intToControlOptions({{0,"FWD      "},{1,"FWD_RIGHT"},{2,"FWD_LEFT "},{3,"REV      "},{4,"REV_RIGHT"},{5,"REV_LEFT "}});


        //Call server and process response
        if (client.call(srv))
        {
                ROS_INFO("Response came back!");
                if(srv.response.moves.size()==0) {
                        ROS_ERROR("Could not find a path!");
                        //Publish obstacles and rough roads alone when path isnt found
                        while(ros::ok()) {
                                obstaclesPublisher.publish(obstacleArray);
                                roughRoadsPublisher.publish(roughRoadsArray);
                                message.header.seq++;
                                ros::spinOnce();

                                loop_rate.sleep();
                        }
                        //ROS_ERROR("")
                        return 0;
                }
                else{
                        //Parse response and publish path, obstacles and roughRoads.
                        for(auto p: srv.response.path) {
                                pose.position.x = p.x;
                                pose.position.y = -p.y;
                                pose.position.z = 0;
                                //ROS_INFO("(%d,%d)",p.x,p.y);
                                //ROS_INFO("%d",p.d);
                                if(p.d == 2) {
                                        pose.orientation.x =0;
                                        pose.orientation.y =0;
                                        pose.orientation.z =0;
                                        pose.orientation.w =1;
                                }
                                else if(p.d == 1) {
                                        pose.orientation.x =0;
                                        pose.orientation.y =0;
                                        pose.orientation.z =1;
                                        pose.orientation.w =0;
                                }
                                else if(p.d == 0) {
                                        pose.orientation.x =0;
                                        pose.orientation.y =0;
                                        pose.orientation.z =sqrt(0.5);
                                        pose.orientation.w =sqrt(0.5);
                                }
                                else{
                                        pose.orientation.x =0;
                                        pose.orientation.y =0;
                                        pose.orientation.z =-sqrt(0.5);
                                        pose.orientation.w =sqrt(0.5);
                                }

                                message.poses.push_back(pose);
                        }

                        //Printing waypoints and moves list on command line
                        ROS_INFO("Way point list:");
                        for(int i=0;i<srv.response.path.size();i++){
                          ROS_INFO("Go %s -> (%d,%d,%c) ", intToControlOptions[srv.response.moves[i]].c_str(), srv.response.path[i].x,srv.response.path[i].y,intToDirection[srv.response.path[i].d]);
                        }

                        while(ros::ok()) {//publish results for rviz
                                poseArrayPublisher.publish(message);
                                obstaclesPublisher.publish(obstacleArray);
                                roughRoadsPublisher.publish(roughRoadsArray);
                                message.header.seq++;
                                ros::spinOnce();

                                loop_rate.sleep();
                        }
                }
        }
        else
        {
                ROS_ERROR("Failed to call service AstarPlanner");
                //return 1;
        }


        return 0;
}
