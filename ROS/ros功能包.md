# 创建功能包
```
catkin_create_pkg 自定义ROS包名 roscpp rospy std_msgs
```
在工作空间的src目录下,创建需要的功能包
# 编写代码
在功能包的src目录（不是工作空间的src）下写c++代码
在功能包的script目录下写python脚本
# 多个功能包
可以创建多个功能包，或者在一个新的文件夹中，创建多个功能包
catkin_make 会递归扫描 src/ 下的所有子目录，只有包含 package.xml 的目录才会被识别为“一个包”
# 构建 ROS 工作空间
catkin_make 并不是“编译 Python”，而是在构建 ROS 工作空间

## C++
对 C++ 节点进行正常编译
catkin 会生成可执行文件到：
```
devel/lib/<pkg_name>/<node_name>
```
## Python
对Python代码,安装 Python 节点到运行目录
```
devel/lib/slope_estimator/node.py
# 补充
# rosrun 执行的脚本来自 devel/lib/
# 但脚本内的相对路径，是基于运行 rosrun 时的当前终端目录，而不是脚本所在目录
```
如果新定义了消息,也会生成 Python 的消息、服务相关代码API
```
devel/lib/python3/dist-packages/<pkg_name>/msg/_MyData.py
```
并创建运行环境例如
```
PYTHONPATH
ROS_PACKAGE_PATH
PATH
```
# 启动
使用rosrun
```
rosrun <包名> <可执行文件名> [重映射/特殊参数]
例如
rosrun lslidar_driver lslidar_c16_node
rosrun <包名> <可执行文件名> __name:=新节点名
rosrun my_pkg my_node input:=/camera/rgb/image_raw
```
使用roslaunch,更高效。<launch文件名> 存在于该包的 launch/ 目录下
```
roslaunch <包名> <launch文件名> [arg_name:=value ...]
```
launch文件写法(xml结构)
```
<launch>
  <!-- 1. 定义启动参数（仅在启动时会使用的参数） -->
  <arg name="mode" default="front" />
  <arg name="slope_threshold" default="10.0" />
  <arg name="debug" default="false" />
  
  <!-- 2. 设置全局参数（param）,这个是给ros的参数 -->
  <!-- 设置到全局命名空间 / -->
  <param name="use_sim_time" value="false" />
  <!-- 加载 YAML 文件到参数服务器 -->
  <rosparam file="$(find slope_estimator)/config/params.yaml" command="load" />

  <!-- 3. Include 其他包的 launch 文件 -->
  <!-- 启动 LiDAR 驱动（来自另一个包） -->
  <include file="$(find lslidar_driver)/launch/lslidar_c16.launch">
    <!-- 给 include 的 launch 传入参数 -->
    <arg name="frame_id" value="laser" />
  </include>

  <!-- 4. 启动节点（node）-->
  <node pkg="slope_estimator"
        type="slope_node.py"
        name="slope_node"
        output="screen">
    <!-- (1) 给节点传参数（private param → ~param_name） -->
    <param name="threshold" value="$(arg slope_threshold)" />
    <!-- (2) 重映射（将节点内部的话题映射到其他话题） -->
    <remap from="~input_cloud" to="/point_cloud_raw" />
    <remap from="~imu_data"    to="/imu" />
    <!-- (3) 环境变量（可选，高级用法） -->
    <env name="PYTHONUNBUFFERED" value="1" />
  </node>

  <!-- 5. 启动多个相同节点（多实例） -->
  <node pkg="slope_estimator"
        type="worker.py"
        name="worker1"
        output="log">
    <param name="task_id" value="1" />
  </node>
  <node pkg="slope_estimator"
        type="worker.py"
        name="worker2"
        output="log">
    <param name="task_id" value="2" />
  </node>

  <!-- 6. 使用 namespace 管理节点 -->
  <group ns="robot1">
    <node pkg="nav_pkg" type="controller" name="controller" />
  </group>
  <group ns="robot2">
    <node pkg="nav_pkg" type="controller" name="controller" />
  </group>
  <!-- 7. 条件运行（if / unless） -->
  <node pkg="rviz"
        type="rviz"
        name="rviz"
        if="$(arg debug)" />
</launch>
```
