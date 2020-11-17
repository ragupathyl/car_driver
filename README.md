# car_driver
A simple A* based path planner for moving a car on a grid, with movement constraints, obstacles and roads with varying cost

Build the project by calling catkin_make from <catkin_workspace> and then $ source devel/setup.bash for good measure

To test the module with default parameters for maps and poses, run 

    roslaunch car_driver car_driver.launch

If using the launch file with custom parameters, use the following command and modify the args accordingly. Note the client ignores the args unless all 10 args are defined in cmd line.

    roslaunch car_driver car_driver.launch mapWidth:=20 mapHeight:=20 startX:=0 startY:=0 startD:=2 goalX:=18 goalY:=18 goalD:=2 obstacleCount:=25 roughRoadCount:=25 

Where, startD and goalD are the orientations of the car at start and goal poses.Key : North = 0 ; West = 1 ; East = 2 ; South = 3 ;
obstacleCount and roughRoadCount are the number of obstacles and rough road gridcells to be randomly inserted into the map respectively.


If running the nodes manually, execute the following commands in order

(a) Launch AstarPlannerSever:

    rosrun car_driver AstarPlannerServer

(b) Launch AstarPlannerClient with default parameters

    rosrun car_driver AstarPlannerClient

(c) Alternatively, launch AstarPlannerClient with custom parameters(args ordering follow that of the
roslaunch car_driver.launch file

    rosrun car_driver AstarPlannerClient 20 20 0 0 2 18 18 2 25 25

(d) Launch the tf publisher to assist rviz in frame transform

    rosrun tf static_transform_publisher 0 0 0 0 0 0 map Car_driver 50

(e) Launch rviz and load Rviz_config.rviz from <catkin_workspace>/src/car_driver/

    rosrun rviz rviz

File layout:

1)include/Astar_planner.h - Header file for Astar_planner class

2)msg/Pair.msg - defines Pair rosmsg

3)msg/Pose2D.msg - defines Pose2D rosmsg

4)msg/VectorPair.msg - defines VectorPair rosmsg

5)src/Astar_planner.cpp - Astar_planner class implementation

6)src/AstarPlannerClient.cpp - Ros service client file for AstarPlanner Service

7)src/AstarPlannerServer.cpp - Ros service server file for AstarPlanner Service

8)srv/AstarPlanner.srv - defines AstarPlanner service format

9)car_driver.launch - Launch file for quick demo

10)car_driver.pdf - detailed description of the algorithm

11)CmakeLists.txt & package.xml - Ros module files

12)Rviz_config.rviz - customized config file for rviz

13)rviz_frame_fix - script to launch tf publisher that fixes frame for rviz
