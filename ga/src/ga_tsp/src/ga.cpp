// 用GA算法解决TSP问题
#include "ga.h"
#include <string>
#include <cmath>
#include <algorithm>  
#include <random>  
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>


using namespace std;

GA::GA()
{
    ROS_INFO("start of the program\n" );
}

void GA::init(ros::NodeHandle &nh){
    ROS_INFO("initialization of the program\n" );
    // 获取城市坐标
    nh.getParam("city/num", city_num);
    for (int i=0;i<city_num;++i) {
        City c;
        if (!nh.getParam("city/id_" + to_string(i+1), c.id) ||
            !nh.getParam("city/pos_x_"+ to_string(i+1), c.pos_x) ||
            !nh.getParam("city/pos_y_"+ to_string(i+1), c.pos_y)) {
            ROS_ERROR("Failed to get parameters for city: %d", c.id);
        }
        city_list.emplace_back(c);
    }
    // 获取GA算法基本信息
    nh.getParam("population/num", population_num);
    nh.getParam("population/iter_num", iter_num);
    nh.getParam("population/cross_probability", cross_probability);
    nh.getParam("population/mutation_probability", mutation_probability);

    
    marker_timer = nh.createTimer(ros::Duration(0.1), &GA::markerCallback, this); 

    marker_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_city", 1);
    path_pub = nh.advertise<visualization_msgs::Marker>("visualization_path", 10);
    path_arrow_pub = nh.advertise<geometry_msgs::PoseArray>("path_arrows", 10);

    pos_sub = nh.subscribe("/move_base_simple/goal", 1, &GA::posCallback, this);
     
}

void GA::initPopulation(vector<vector<int>> &population){
    ROS_INFO("initialization of the population\n" );
    //用id表示的population
    vector<int> p;
    for(int i=0;i<city_list.size();++i){
        p.emplace_back(city_list[i].id);
    }
    population = vector<vector<int>>(population_num, vector<int>(city_list.size(), 0));
    for(int i=0;i<population_num;++i){
        std::random_device rd;  
        std::mt19937 g(rd());
        // 打乱顺序，创建初始population
        std::shuffle(p.begin(), p.end(), g); 
        population[i] = p;
    }  
}

void GA::pathShow(const vector<Eigen::Vector3d> &waypoints){
    geometry_msgs::PoseArray poses_msg;  
    poses_msg.header.stamp = ros::Time::now();  
    poses_msg.header.frame_id = "map"; 
    poses_msg.poses.resize(city_list.size());
  
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.ns = "path";
    marker.action = visualization_msgs::Marker::ADD;
    marker.header.stamp = ros::Time::now();
    marker.type = visualization_msgs::Marker::LINE_STRIP; //ARROW  LINE_STRIP
    marker.id = 0; 
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.color.r = 0;
    marker.color.g = 0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
    for(int i=0;i<waypoints.size();++i){
        geometry_msgs::Point p;
        p.x = waypoints[i].x();
        p.y = waypoints[i].y();
        p.z = waypoints[i].z();
        marker.points.emplace_back(p);
        poses_msg.poses[i].position.x = waypoints[i].x();  
        poses_msg.poses[i].position.y = waypoints[i].y(); 
        poses_msg.poses[i].position.z = waypoints[i].z(); 
        double dx = (i+1) < waypoints.size() ? waypoints[i+1].x() - waypoints[i].x() : 0; 
        double dy = (i+1) < waypoints.size() ? waypoints[i+1].y() - waypoints[i].y() : 1; 
        double theta = (i+1) < waypoints.size() ? atan2(dy,dx) : 0;
        poses_msg.poses[i].orientation.x = 0.0;  
        poses_msg.poses[i].orientation.y = 0.0;  
        poses_msg.poses[i].orientation.z = sin(theta/2);  
        poses_msg.poses[i].orientation.w = cos(theta/2); 
    }
    path_pub.publish(marker);
    path_arrow_pub.publish(poses_msg); 
}

void GA::solve(){
    map<double, vector<City>> result;
    GA::initPopulation(population);
    GA::fitnessFunc(population, fitness);
    for(int i=0;i<iter_num;++i){
        GA::select(rand_population);
        GA::crossover(cross_population);
        GA::mutation(mutation_population);
        GA::fitnessFunc(mutation_population, fitness);
        GA::bestResult(result);
        cout << "__________" << "loop" << i+1 << "__________" << endl;
    }
    
    // result是升序排列，故取fitness最小的点为解
    vector<City> result_final = result.begin()->second;

    ROS_INFO("best result is: ");
    for(int i=0;i<result_final.size();++i){
        cout << i << ": " << "id: " << result_final[i].id << endl;
    }

    // rviz显示路线和方向
    vector<Eigen::Vector3d> waypoints;
    for(auto &c : result_final){
        Eigen::Vector3d cc;
        cc.x() = c.pos_x;
        cc.y() = c.pos_y;
        cc.z() = 0.5;
        waypoints.emplace_back(cc);
    }
    GA::pathShow(waypoints);

}

void GA::select(vector<vector<int>> &rand_population){
    ROS_INFO("In the select step of the program\n" );
    // 轮盘赌选择方法
    vector<double> fitness_inv(population_num, 0);
    vector<double> fitness_accumu(population_num, 0); // fitness_accumu 为累计概率
    rand_population = vector<vector<int>>(population_num, vector<int>(city_list.size(), 0));
    double totalFitness = accumulate(fitness.begin(), fitness.end(), 0);
    double f_a = 0;
    // 用倒数可以限定范围在【0，1】
    for(int i=0;i<population_num;++i){
        fitness_inv[i] = (double)fitness[i] / totalFitness;
        f_a += fitness_inv[i];
        fitness_accumu[i] = f_a;
    }

    totalFitness = accumulate(fitness_inv.begin(), fitness_inv.end(), 0.0);
    for (int i = 0; i < population_num; ++i) {    
        double rp = (static_cast<double>(rand()) / static_cast<double>(RAND_MAX))* totalFitness;
        for(int j=0; j< population_num; ++j){
            if (rp <= fitness_accumu[j]) { 
                rand_population[i] = population[j];
                break;
            }
        }
    }
}

void GA::crossover(vector<vector<int>> &cross_population){
    ROS_INFO("In the crossover step of the program\n" );
    cross_population = rand_population;
    double cp = static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
    // 随机选2个population进行顺序交替交换
    if(cp <= cross_probability){
        int parent_1 = rand() % population_num;
        int parent_2 = rand() % population_num;
        vector<int> p_1 = cross_population[parent_1];
        vector<int> p_2 = cross_population[parent_2];
        GA::alternateMix(p_1, p_2);
        cross_population[parent_1] = p_1;
        cross_population[parent_2] = p_2;
    }
}

void GA::mutation(vector<vector<int>> &mutation_population){
    ROS_INFO("In the mutation step of the program\n" );
    // 随机选population的两个点进行交换
    mutation_population = cross_population;
    for(int i=0;i<population_num;++i){
        double mp = static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
        if(mp <= mutation_probability){
            int pos_1 = rand() % city_list.size();
            int pos_2 = rand() % city_list.size();
            swap(mutation_population[i][pos_1], mutation_population[i][pos_2]);
        }
    }
}   

void GA::bestResult(map<double, vector<City>> &result){
    City c;
    for(int i=0;i<population_num;++i){
        vector<City> c_list;
        for(int j=0;j<city_list.size();++j){
            c.id = mutation_population[i][j];
            c.pos_x = city_list[mutation_population[i][j]].pos_x;
            c.pos_y = city_list[mutation_population[i][j]].pos_y;
            c_list.emplace_back(c);
        }
        result.emplace(fitness[i]-(0.0001*c_list[0].id), c_list);  // 增加一项对序号的处理key避免重复，起始点如果是大序号则占优
        // 该项可自行调整，只要使key值互不相等即可 
    }
    
}

double GA::gap(const City &a, const City &b){
    // 计算两点间欧式距离为适应度函数
    return sqrt(pow((a.pos_x - b.pos_x), 2) + pow((a.pos_y - b.pos_y), 2));
}

void GA::fitnessFunc(vector<vector<int>> &population, vector<double> &fitness){
    fitness = vector<double>(population_num, 0);
    for(int i=0;i<population_num;++i){
        for(int j=0;j<city_list.size()-1;++j){
            fitness[i] += gap(city_list[population[i][j]], city_list[population[i][j+1]]);
        }
        fitness[i] += gap(city_list[population[i][city_list.size()-1]], city_list[population[i][0]]);
    }

}

void GA::alternateMix(vector<int> &parent_1, vector<int> &parent_2){
    if(parent_1.size() != city_list.size() && parent_2.size() != city_list.size()) ROS_ERROR("population raise a problem in: parent cross");
    // 两个vector顺序交替替换
    for(int i=0; i<parent_1.size();++i){
        if(i%2 == 0){
            swap(parent_1[i], parent_2[i]);
        }
    }
    // 检查两个vector重复项
    map<int, int> m1;
    map<int, int> m2;
    for(int i=0;i<parent_1.size();++i){
        if(find(parent_1.begin(), parent_1.end(), i) != parent_1.end() && count(parent_1.begin(), parent_1.end(), i) == 2){
            m1[i] = distance(parent_1.begin(),find(parent_1.begin(), parent_1.end(), i));
        }
        else if(find(parent_2.begin(), parent_2.end(), i) != parent_2.end() && count(parent_2.begin(), parent_2.end(), i) == 2){
            m2[i] = distance(parent_2.begin(),find(parent_2.begin(), parent_2.end(), i));
        }
    }
    //重复项处理，两个vector中的重复项分别的替换到重复项出现的第一个idx
    if(m1.size() != m2.size()) ROS_ERROR("population raise a problem in: alternateMix");
    auto iter1 = m1.begin(); auto iter2 = m2.begin();
    while(iter1 != m1.end() && iter2 != m2.end()){
        parent_1[iter1->second] = iter2->first;
        parent_2[iter2->second] = iter1->first;
        ++iter1;++iter2;
    }
}

void GA::markerCallback(const ros::TimerEvent &e){
    // 在rviz里绘制marker, 发布city marker
    visualization_msgs::MarkerArray M;
    visualization_msgs::Marker marker;
    for(int i=0;i<city_list.size();++i){
        marker.header.frame_id = "map";
        marker.ns = "city";
        marker.header.stamp = ros::Time::now();
        marker.type = visualization_msgs::Marker::CUBE;
        marker.id = city_list[i].id;
        marker.pose.position.x = city_list[i].pos_x;
        marker.pose.position.y = city_list[i].pos_y;
        marker.pose.position.z = 0.5;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 1;
        marker.scale.y = 1;
        marker.scale.z = 1;
        marker.color.r = 255;
        marker.color.g = 255;
        marker.color.b = 0;
        marker.color.a = 1.0;
        M.markers.emplace_back(marker);
    }
    marker_pub.publish(M);
}

void GA::posCallback(const geometry_msgs::PoseStamped &msg){
    // 接收到任意点坐标后开始求解
    City c_pos;
    c_pos.id = city_num;
    c_pos.pos_x = msg.pose.position.x;
    c_pos.pos_y = msg.pose.position.y;
    if(city_list.back().id == city_num) city_list.pop_back(); // 删除上一个设置的任意点
    city_list.emplace_back(c_pos);
    GA::solve(); 
}

GA::~GA()
{
    ROS_WARN("end of the program\n");
}
