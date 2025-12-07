# ROS通信
## 话题
| 特性       | 描述                  |
| -------- | ------------------- |
| 通信方向     | 单向（发布 → 订阅）         |
| 通信方式     | 异步（发布方不等订阅方回应）      |
| 适用场景     | 持续发送数据流             |
| 是否阻塞     | 否                   |
| 是否有返回值   | 无                   |
| 是否保存历史消息 | 一般不保存（除非使用 `latch`） |

发布话题
* 初始化节点
* 初始化发布
* 设置频率,ros自带的rate,比sleep更准(会算入运行消耗的时间)
* 循环发布
```
#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

if __name__ == "__main__":
    rospy.init_node("topic_pub")
    pub = rospy.Publisher("/chatter", String, queue_size=10)
    rate = rospy.Rate(5)  # 5 Hz
    i = 0
    while not rospy.is_shutdown():
        msg = String(data=f"hello {i}")
        pub.publish(msg)
        rospy.loginfo(f"PUB -> {msg.data}")
        i += 1
        rate.sleep()
```  
C++同理,还需要一个句柄
```
//C++
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>

int main(int argc, char** argv) {
  ros::init(argc, argv, "topic_pub_cpp");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<std_msgs::String>("/chatter_cpp", 10);
  ros::Rate rate(5); // 5 Hz
  int i = 0;
  while (ros::ok()) {
    std_msgs::String msg;
    std::stringstream ss;
    ss << "hello_cpp " << i++;
    msg.data = ss.str();
    pub.publish(msg);
    ROS_INFO("PUB -> %s", msg.data.c_str());
    rate.sleep();
  }
  return 0;
}
```
订阅话题
* 初始化节点
* 初始化订阅,并绑定回调函数
* spin(),无需手动写循环
```
#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def cb(msg: String):
    rospy.loginfo(f"SUB <- {msg.data}")

if __name__ == "__main__":
    rospy.init_node("topic_sub")
    rospy.Subscriber("/chatter", String, cb, queue_size=10)
    rospy.spin()
```
C++同理
```
//C++
#include <ros/ros.h>
#include <std_msgs/String.h>

void cb(const std_msgs::String::ConstPtr& msg) {
  ROS_INFO("SUB <- %s", msg->data.c_str());
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "topic_sub_cpp");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/chatter_cpp", 10, cb);
  ros::spin();
  return 0;
}
```
## 服务
| 特性       | 描述          |
| -------- | ----------- |
| 通信方向     | 双向（请求 ↔ 响应） |
| 通信方式     | 同步（请求必须等响应） |
| 适用场景     | 一次性任务、命令调用  |
| 是否阻塞     | 是（客户端等待响应）  |
| 是否有返回值   | 有（response） |
| 是否保存历史消息 | 否           |

服务的使用类似http请求

服务端处理请求
```
#!/usr/bin/env python3
import rospy
from std_srvs.srv import SetBool, SetBoolResponse

def handle(req):
    # req.data 是 bool
    rospy.loginfo(f"SERVER <- request: data={req.data}")
    success = True
    message = "turned ON" if req.data else "turned OFF"
    return SetBoolResponse(success, message)

if __name__ == "__main__":
    rospy.init_node("srv_server_py")
    srv = rospy.Service("/switch", SetBool, handle)
    rospy.loginfo("Service /switch (SetBool) ready.")
    rospy.spin()
```
客户端发送请求,并接收响应
```
#!/usr/bin/env python3
import rospy
from std_srvs.srv import SetBool

if __name__ == "__main__":
    rospy.init_node("srv_client_py")
    rospy.wait_for_service("/switch")
    call = rospy.ServiceProxy("/switch", SetBool)

    resp1 = call(True)
    rospy.loginfo(f"CLIENT -> True | resp: ok={resp1.success}, msg='{resp1.message}'")

    resp2 = call(False)
    rospy.loginfo(f"CLIENT -> False | resp: ok={resp2.success}, msg='{resp2.message}'")
```
C++代码同理
服务端
```
#include <ros/ros.h>
#include <std_srvs/SetBool.h>

bool handle(std_srvs::SetBool::Request &req,
            std_srvs::SetBool::Response &res) {
  ROS_INFO("SERVER <- request: data=%s", req.data ? "true" : "false");
  res.success = true;
  res.message = req.data ? "turned ON" : "turned OFF";
  return true; // 返回true表示服务调用成功（非业务语义）
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "srv_server_cpp");
  ros::NodeHandle nh;
  ros::ServiceServer server = nh.advertiseService("/switch", handle);
  ROS_INFO("Service /switch (SetBool) ready (C++).");
  ros::spin();
  return 0;
}
```
客户端
```
#include <ros/ros.h>
#include <std_srvs/SetBool.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "srv_client_cpp");
  ros::NodeHandle nh;

  ros::ServiceClient client = nh.serviceClient<std_srvs::SetBool>("/switch");
  std_srvs::SetBool srv;

  srv.request.data = true;
  if (client.call(srv)) {
    ROS_INFO("CLIENT -> true | ok=%d msg=%s", srv.response.success, srv.response.message.c_str());
  } else {
    ROS_ERROR("Failed to call service /switch");
  }

  srv.request.data = false;
  if (client.call(srv)) {
    ROS_INFO("CLIENT -> false | ok=%d msg=%s", srv.response.success, srv.response.message.c_str());
  } else {
    ROS_ERROR("Failed to call service /switch");
  }

  return 0;
}
```
## 参数服务器
参数服务器的使用非常简单
```
//C++通过 ros::NodeHandle 访问参数服务器
bool getParam(const std::string &key, T &val);   // 拿参数（没有就返回 false）
void setParam(const std::string &key, const T &val);   // 设置参数
bool hasParam(const std::string &key);                 // 是否存在
bool deleteParam(const std::string &key);              // 删除
```
```
# Python
rospy.get_param(name, default=None)
rospy.set_param(name, value)
rospy.has_param(name)
rospy.delete_param(name)
rospy.search_param(name)  # 在 namespace 里向上找参数
```