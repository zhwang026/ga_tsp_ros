# ga_tsp_ros
Solving the Traveling Salesman Problem (TSP) with Genetic Algorithm (GA)  with rviz Simulation under ROS

# Running Executables
Compiling tests passed on ubuntu 20.04 with ros-1 noetic installed. 
The executable commands as follows.
```
git clone https://github.com/zhwang026/ga_tsp_ros.git
cd ga_tsp_ros/ga/
catkin_make
source devel/setup.bash
roslaunch ga_tsp ga_tsp.launch
```
using rviz-2D nav Goal tool to start the program and select a random point to add to the sequence as a random city for the TSP problem.


