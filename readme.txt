1、执行命令：
mkdir mkdir /home/名称/ga
cd /home/zhw/gga
mkdir src
cd src
catkin_init_workspace 
（把ga_tsp包拖入src中）
cd /home/zhw/gga
cakin_make
source devel/setup.bash
roslaunch ga_tsp ga_tsp.launch
2、Ubuntu下ROS环境使用，rviz仿真，用2D nav goal选择任意点后程序开始运行
