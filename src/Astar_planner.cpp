
#ifndef PATH_PLANNER_ASTAR_CPP
#define PATH_PLANNER_ASTAR_CPP

#include "car_driver/Astar_planner.h"

//Function to evaluate H(Cell)
float Astar_planner::Hvalue(const Pose &a){

        float diagonalPart = std::min(abs(goal.x-a.x),abs(goal.y-a.y));
        float linearPart = abs(abs(goal.x-a.x)-abs(goal.y-a.y));
        return 1.5707*diagonalPart+linearPart;

}

//Function to calculate step cost for moving from startPose to endPose
float Astar_planner::StepCost(const Pose &startPose, const Pose &endPose, const ControlOptions &c){
        float cost;
        if(c==FWD || c==REV) {
                cost =  1.0;
        }
        else cost=1.5707;
        std::pair<int,int> startCell(startPose.x,startPose.y);
        std::pair<int,int> endCell(endPose.x,endPose.y);
        if(roughRoads[startCell].find(endCell) != roughRoads[startCell].end()) {
                cost = cost*2;
        }
        return cost;
}

//Function to check if the current move hits a wall or if it hits an obstacle
bool Astar_planner::CheckWall(const Pose &p){

        if(p.x<0 || p.y<0 || (p.x>=mapWidth) || (p.y>=mapHeight)) return true;

        else if(obstacles.count({p.x,p.y})!=0) return true;
        else return false;
}

//Lists the set of possible moves and poses from a given pose
std::vector<Pose> Astar_planner::NextStates(const Pose &pose, std::vector<ControlOptions> &move){
        std::vector<Pose> result;
        move.clear();
        if(pose.orientation == N) {
                result.push_back((Pose){pose.x, pose.y-1, N});
                move.push_back(FWD);
                result.push_back((Pose{pose.x-1,pose.y-1,W}));
                move.push_back(FWD_LEFT);
                result.push_back((Pose){pose.x+1,pose.y-1,E});
                move.push_back(FWD_RIGHT);
                result.push_back((Pose){pose.x,pose.y+1,N});
                move.push_back(REV);
                result.push_back((Pose){pose.x-1,pose.y+1,E});
                move.push_back(REV_LEFT);
                result.push_back((Pose){pose.x+1,pose.y+1,W});
                move.push_back(REV_RIGHT);
        }
        else if(pose.orientation == S) {
                result.push_back((Pose){pose.x, pose.y+1, S});
                move.push_back(FWD);
                result.push_back((Pose){pose.x+1,pose.y+1,E});
                move.push_back(FWD_LEFT);
                result.push_back((Pose){pose.x-1,pose.y+1,W});
                move.push_back(FWD_RIGHT);
                result.push_back((Pose){pose.x,pose.y-1,S});
                move.push_back(REV);
                result.push_back((Pose){pose.x+1,pose.y-1,W});
                move.push_back(REV_LEFT);
                result.push_back((Pose){pose.x-1,pose.y-1,E});
                move.push_back(REV_RIGHT);
        }
        else if(pose.orientation == W) {
                result.push_back((Pose){pose.x-1, pose.y, W});
                move.push_back(FWD);
                result.push_back((Pose){pose.x-1,pose.y+1,S});
                move.push_back(FWD_LEFT);
                result.push_back((Pose){pose.x-1,pose.y-1,N});
                move.push_back(FWD_RIGHT);
                result.push_back((Pose){pose.x+1,pose.y,W});
                move.push_back(REV);
                result.push_back((Pose){pose.x+1,pose.y+1,N});
                move.push_back(REV_LEFT);
                result.push_back((Pose){pose.x+1,pose.y-1,S});
                move.push_back(REV_RIGHT);
        }
        else{
                result.push_back((Pose){pose.x+1, pose.y, E});
                move.push_back(FWD);
                result.push_back((Pose){pose.x+1,pose.y-1,N});
                move.push_back(FWD_LEFT);
                result.push_back((Pose){pose.x+1,pose.y+1,S});
                move.push_back(FWD_RIGHT);
                result.push_back((Pose){pose.x-1,pose.y,E});
                move.push_back(REV);
                result.push_back((Pose){pose.x-1,pose.y-1,S});
                move.push_back(REV_LEFT);
                result.push_back((Pose){pose.x-1,pose.y+1,N});
                move.push_back(REV_RIGHT);
        }
        return result;

}

//Function to trace path via cellInfo member 'previous'
std::list<Pose> Astar_planner::TracePath(const std::unordered_map<Pose,Cell,HashTriplet> &cellInfo,std::list<ControlOptions> &moves){

        auto it = cellInfo.find(goal);

        std::list<Pose> path;
        moves.clear();
        path.push_front(goal);

        while(start != (it->first)) {
                path.push_front(it->second.previous);
                moves.push_front(it->second.move);
                it=cellInfo.find(it->second.previous);
        }
        moves.push_front(it->second.move);
        return (path);
}

//Search function implementation
std::list<Pose> Astar_planner::search(int mapWidth, int mapHeight, const std::vector<std::pair<int,int> > &obstacles, const std::vector<std::pair<int,int> > &roughRoads, const Pose &robotPose, const Pose &goalPose, std::list<ControlOptions> &moves)
{


        using std::vector;
        using std::cout;
        using std::endl;
        using std::pair;
        using std::make_pair;
        if(mapWidth == 0 || mapHeight == 0) {
                cout<<"///////////////////////////////////////"<<endl;
                cout<<"////////////Invalid map size!//////////"<<endl;
                cout<<"///////////////////////////////////////"<<endl;
                return std::list<Pose> ();
        }

        if(mapWidth<1+robotPose.x || mapWidth<1+goalPose.x || robotPose.x <0 || goalPose.x<0) {
                cout<<"///////////////////////////////////////"<<endl;
                cout<<"////////robotPose is not valid/////////"<<endl;
                cout<<"///////////////////////////////////////"<<endl;
                return std::list<Pose> ();
        }
        else if(mapHeight<1+robotPose.y || mapHeight<1+goalPose.y || robotPose.y <0 || goalPose.y<0) {
                cout<<"///////////////////////////////////////"<<endl;
                cout<<"/////////goalPose is not valid////////"<<endl;
                cout<<"///////////////////////////////////////"<<endl;
                return std::list<Pose> ();
        }


        //Updating private state variables
        this->mapWidth = mapWidth;
        this->mapHeight = mapHeight;

        start = robotPose;
        goal = goalPose;

        //Populating the obstacles set with the obstacles input
        for(auto p: obstacles) {
                this->obstacles.insert(p);
                this->obstacles.insert(make_pair(p.first+1,p.second));
                this->obstacles.insert(make_pair(p.first+1,p.second+1));
                this->obstacles.insert(make_pair(p.first,p.second+1));
        }

        if(CheckWall(start)) {
                cout<<"///////////////////////////////////////"<<endl;
                cout<<"//////Obstacles overlap start pose!////"<<endl;
                cout<<"///////////////////////////////////////"<<endl;
                return std::list<Pose> ();
        }
        else if(CheckWall(goal)) {
                cout<<"///////////////////////////////////////"<<endl;
                cout<<"//////Obstacles overlap goal pose!/////"<<endl;
                cout<<"///////////////////////////////////////"<<endl;
                return std::list<Pose> ();
        }

        //Populating the unordered_map roughRoads for fast lookup
        for(auto p: roughRoads) {
                pair<int,int> p1(p.first+1,p.second);
                pair<int,int> p2(p.first+1,p.second+1);
                pair<int,int> p3(p.first,p.second+1);

                this->roughRoads[p].insert(p1);
                this->roughRoads[p].insert(p2);
                this->roughRoads[p].insert(p3);

                this->roughRoads[p1].insert(p);
                this->roughRoads[p1].insert(p2);
                this->roughRoads[p1].insert(p3);

                this->roughRoads[p2].insert(p);
                this->roughRoads[p2].insert(p3);
                this->roughRoads[p2].insert(p1);

                this->roughRoads[p3].insert(p);
                this->roughRoads[p3].insert(p1);
                this->roughRoads[p3].insert(p2);

        }

        //Temp variables for A* search
        float f_current,g_current,h_current;
        //closedList to keep track of visited nodes
        std::unordered_set<Pose, HashTriplet> closedList;

        //Cout commands for verbose flag
        if(verbose) {
                cout<<"Processing World state with following paramenters:"<<endl;
                cout<<"Number of mapWidths = "<<mapWidth<<endl;
                cout<<"Number of columns = "<<mapHeight<<endl<<endl;

                cout<<"Starting position : "<<start.x<<","<<start.y<<endl;
                cout<<"Goal position : "<<goal.x<<","<<goal.y<<endl<<endl;


        }

        //Adding properties of Starting Cell into cellInfo
        h_current=Hvalue(robotPose);

        cellInfo[robotPose] = (Cell){robotPose,FWD,h_current,h_current,0.0};

        //Pushing <0,robotPose> into open list; F=0 to ensure maximum priority;
        openlist.push(make_pair(0.0,robotPose));
        //Setting flag variable
        reached_goal=false;

        //count to keep track of number of poses explored
        int count = 0;

        //Running while loop till openlist is emptied
        while(!openlist.empty()) {

                //Popping the highest priority pair from openlist
                pair<float,Pose> current = openlist.top();
                openlist.pop();
                count++;

                //Adding current Cell to closedlist
                Pose temp = current.second;
                closedList.insert(temp);



                //Cout statements for verbose option
                if(verbose) {
                        cout<<"Exploring Cell ("<<temp.x<<","<<temp.y<<","<<temp.orientation<<")"<<endl;
                        cout<<"G value is = "<<cellInfo.find(temp)->second.g<<endl;
                        cout<<"H value is = "<<Hvalue(temp)<<endl;
                        cout<<"F value is = "<<current.first<<endl<<endl;
                }

                //Getting list of next moves
                vector<ControlOptions> move;
                vector<Pose> options = NextStates(temp,move);

                for(int i=0; i<options.size(); i++) {
                        g_current = cellInfo.find(temp)->second.g;
                        if(!CheckWall(options[i])) {


                                h_current = Hvalue(options[i]);
                                g_current += StepCost(temp,options[i],move[i]);
                                f_current = g_current+h_current;
                                //If goal Cell is found, update cellInfo and return TracePath()
                                if(options[i]==goalPose) {

                                        cellInfo.insert(pair<Pose,Cell>(options[i],(Cell){temp,move[i],g_current,h_current,f_current}));
                                        reached_goal=true;
                                        cout<<"Goal reached"<<std::endl;
                                        cout<<"Cost of path = "<<g_current<<std::endl;
                                        cout<<"number of nodes opened = "<<count<<std::endl;
                                        return TracePath(cellInfo,moves);

                                }
                                //Else if Cell isnt part of closedlist, calcualte F(Cell)
                                else if(closedList.count(options[i])==0) {

                                        auto Cell_it=cellInfo.find(options[i]);
                                        //If Cell isnt part of cellInfo, add to openlist and add cellInfo
                                        if(Cell_it==cellInfo.end()) {
                                                openlist.push(make_pair(f_current,options[i]));

                                                cellInfo.insert(pair<Pose,Cell>(options[i],(Cell){temp,move[i],f_current,h_current,g_current}));

                                        }
                                        //If Cell is part of cellInfo, check if F(Cell) can be updated with new value. Update cellInfo and openlist
                                        else if(Cell_it->second.f > f_current) {
                                                openlist.push(make_pair(f_current,options[i]));

                                                Cell_it->second.previous=temp;
                                                Cell_it->second.move = move[i];
                                                Cell_it->second.f=f_current;
                                                Cell_it->second.g=g_current;
                                        }

                                }
                        }
                }

        }
//IF goal was not found, return empty path
        if(!reached_goal) {

                cout<<"Goal could not be reached "<<endl;
                return std::list<Pose> ();

        }



}

#endif
