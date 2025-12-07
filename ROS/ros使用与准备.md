# 启动roscore
```
roscore
```
提供 ROS master，所有节点、话题、服务都需要向它注册
# 加载 ROS 环境（官方环境）
```
source /opt/ros/noetic/setup.bash
```
加载 ROS 的基础环境变量，使终端具备运行 ROS 命令（rosrun、roslaunch、rostopic 等）的能力
# 准备
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
```
创建工作空间，在其中创建src目录，存放代码
# 创建功能包
```
cd src
catkin_create_pkg 自定义ROS包名 roscpp rospy std_msgs
```
进入 src 目录，创建功能包并声明所需依赖,在其中写所需的功能（更详细的笔记在 `ros功能包.md`）
# 编译
```
catkin_make
```
使用 catkin_make 命令，进行编译
# 注册自定义工作空间环境
```
source ~/catkin_ws/devel/setup.bash
```
注册到系统环境(当前终端)
