#include "ros/ros.h"
#include "car_driver/AstarPlanner.h"
#include "car_driver/Astar_planner.h"

//calling Astar_planner class
bool callAstarSearch(car_driver::AstarPlanner::Request  &req,
                     car_driver::AstarPlanner::Response &res)
{
        ROS_INFO("AstarPlanner Service is called!");
        using std::vector;
        using std::pair;
        using std::make_pair;
        using std::list;
        int mapHeight = req.mapHeight;
        int mapWidth = req.mapWidth;

        vector<pair<int,int> > obstacles;
        for(int i=0; i<req.obstacles.vect.size(); i++) {
                obstacles.push_back(make_pair(req.obstacles.vect[i].x,req.obstacles.vect[i].y));
        }

        vector<pair<int,int> > roughRoads;
        for(auto it = req.roughRoads.vect.begin(); it!=req.roughRoads.vect.end(); it++) {
                roughRoads.push_back(make_pair(it->x,it->y));
        }

        Pose start = {req.start.x,req.start.y,static_cast<Direction>(req.start.d)};
        Pose goal = {req.goal.x,req.goal.y,static_cast<Direction>(req.goal.d)};

        list<ControlOptions> moves;
        Astar_planner obj(false);//Initalize with verbose = false to limit print log on command line
        list<Pose> result = obj.search(mapWidth, mapHeight, obstacles, roughRoads, start, goal, moves);
        car_driver::Pose2D p;
        for(auto r: result) {
                p.x = r.x;
                p.y = r.y;
                p.d = static_cast<int>(r.orientation);
                res.path.push_back(p);
        }
        for(auto m:moves) {

                res.moves.push_back(static_cast<int>(m));
        }


        ROS_INFO("AstarPlanner is done");
        return true;
}

int main(int argc, char **argv)
{
        ros::init(argc, argv, "AstarPlanner_server");
        ros::NodeHandle n;
        ROS_INFO("AstarPlannerServer is now online!");
        ros::ServiceServer service = n.advertiseService("AstarPlanner", callAstarSearch);
        ros::spin();

        return 0;
}
