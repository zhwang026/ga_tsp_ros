#ifndef _GA_H_
#define _GA_H_
// 用GA算法解决TSP问题
#include <iostream>
#include <vector>
#include <algorithm>
#include <map>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <Eigen/Eigen>

using namespace std;

struct City
{   
    int id;
    double pos_x;
    double pos_y;
};


class GA{
public:
    GA();
    ~GA();
    /* rviz */
    void pathShow(const vector<Eigen::Vector3d> &waypoints);
    void markerCallback(const ros::TimerEvent &e);
    void posCallback(const geometry_msgs::PoseStamped &msg);
    ros::Publisher marker_pub, path_pub, path_arrow_pub;
    ros::Subscriber pos_sub;
    ros::Timer marker_timer;
    
    /* main function*/
    void init(ros::NodeHandle &nh);
    void initPopulation(vector<vector<int>> &population);
    void fitnessFunc(vector<vector<int>> &population, vector<double> &fitness);
    void select(vector<vector<int>> &rand_population);
    void crossover(vector<vector<int>> &cross_population);
    void alternateMix(vector<int> &parent_1, vector<int> &parent_2);
    void mutation(vector<vector<int>> &muta_population);
    void solve();
    void bestResult(map<double, vector<City>> &result);
    double gap(const City &a, const City &b);

    /* city info*/
    vector<City> city_list;
    int city_num;

    /* GA */
    int population_num, iter_num; // size of answer
    double cross_probability, mutation_probability;
    bool flag;
    vector<double> fitness;
    vector<int> res_population;
    vector<vector<int>> population, rand_population, cross_population, mutation_population;

};
#endif
