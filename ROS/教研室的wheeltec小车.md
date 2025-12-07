# 启动
密码都是 dongguan
打开后,连接热点后,即可ssh连接小车
```
ssh wheeltec@192.168.0.100
```
可能需要再当前终端配置环境,ros的以及功能包的
```
source /opt/ros/noetic/setup.bash
source ~/wheeltec_robot/devel/setup.bash
```
之后启动wheeltec终端（launch会自动运行roscore）,打开很多小车官方的节点
```
roslaunch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch
```