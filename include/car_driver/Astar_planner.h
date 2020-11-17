
#ifndef ASTAR_PLANNER_H
#define ASTAR_PLANNER_H

//Header files for boost and standard C++ libraries
#include <iostream>
#include <queue>
#include <set>
#include <unordered_map>
#include <vector>
#include <math.h>
#include <unordered_set>
#include <list>


enum Direction {N,W,E,S};//For tracking orientation of vehicle at each pose
enum ControlOptions {FWD,FWD_RIGHT,FWD_LEFT,REV,REV_RIGHT,REV_LEFT}; //Discrete control options of the vehicle


//Structure to represent (x,y) locations
struct Pose {
        int x;
        int y;
        Direction orientation; // Direction of the vechile as the 3rd dimension of the map


        //Comparison operators for the unordered_maps and sets
        bool operator==(const Pose& a) const
        {
                return (x == a.x && y == a.y && orientation == a.orientation);
        }

        bool operator!=(const Pose& a) const
        {
                return (x != a.x || y != a.y || orientation != a.orientation);
        }

        bool operator<(const Pose& a) const
        {
                if(x<a.x) return true;
                else if(x>a.x) return false;
                else if(y<a.y) return true;
                else if(y>a.y) return false;
                else if(orientation<a.orientation) return true;
                else return false;
        }

};

//Custom hash function for using std::pair as key in unordered_maps
struct HashPair {
        template <class T1, class T2>
        std::size_t operator()(const std::pair<T1, T2>&p) const
        {
                auto hash1 = std::hash<T1>{} (p.first);
                auto hash2 = std::hash<T2>{} (p.second);
                return hash1 ^ hash2;
        }
};

//Custom hash function for using Pose as key in unordered_maps and sets
struct HashTriplet {
        std::size_t operator()(const Pose &p) const {
                using std::hash;
                using std::string;

                return ((hash<int>{} (p.x)
                         ^ (hash<int>{} (p.y) << 1)) >> 1)
                       ^ (hash<int>{} (p.orientation) << 1);
        }
};

class Astar_planner {
private:

//Astar_planner state variables
        bool verbose; //Verbose statements during algorithm runtime
        bool reached_goal;
        int mapWidth;
        int mapHeight;
        Pose goal;
        Pose start;

//Structure to store processed Cell properties
        struct Cell {
                Pose previous; //index of parent Cell with shortest distance from start position
                ControlOptions move;
                float f;
                float h;
                float g;

        };
        std::unordered_map<Pose, Cell, HashTriplet> cellInfo;


//Priority queue to store pairs of <f(Cell),Cell_index> in openlist sorted in ascending order of F(Cell)
        std::priority_queue<std::pair<float,Pose>,std::vector<std::pair<float,Pose>>,std::greater <std::pair<float,Pose>> > openlist;

//obstacle corners (x,y) stored in unordered_set for fast lookup.
        std::unordered_set<std::pair<int,int>, HashPair> obstacles;

//roughRoads stored as unordered_map where every (x,y) coordinate involved in a roughroad is mapped to all possible near coordinates that are reached via a rough road
        std::unordered_map<std::pair<int,int>,std::set<std::pair<int,int>>, HashPair> roughRoads;

//Function to evaluate H(Cell)
        float Hvalue(const Pose &a);

//Caclucate the cost of traversing from startPose to a neighbor endPose, including curved turns and rough road patches
        float StepCost(const Pose &startPose, const Pose &endPose, const ControlOptions &c);

//List all possible moves from a given coordinate and the resulting coordinates
        std::vector<Pose> NextStates(const Pose &pose, std::vector<ControlOptions> &move);

//Function to check if the current move hits an obstacle or if it hits an obstacle
        bool CheckWall(const Pose &p);

//Function to trace path via Cellinfo member 'previous'
        std::list<Pose> TracePath(const std::unordered_map<Pose,Cell,HashTriplet> &cellInfo,std::list<ControlOptions> &moves);


public:

//Constructor to set verbose option, default = false
        Astar_planner(bool verbose_flag=false) : verbose(verbose_flag) {
        }

//Destructor to delete dynamic vectors
        ~Astar_planner(){
        }

//Search function implementation
        std::list<Pose> search(int mapWidth, int mapHeight,const std::vector<std::pair<int,int>> &obstacles, const std::vector<std::pair<int,int>> &roughroad,const Pose &robotPose,const Pose &goalPose, std::list<ControlOptions> &move);

};

#endif
