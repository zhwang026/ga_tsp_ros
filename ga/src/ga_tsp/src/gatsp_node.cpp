#include <ros/ros.h>
#include <ga.h>

using namespace std;

int main(int argc, char **argv){
    ros::init(argc, argv, "gatsp_node");
    ros::NodeHandle nh("~");

    GA solver;
    solver.init(nh);

    ros::spin();
    return 0;
}