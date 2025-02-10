# 话题

## 实践

### 实验一

#### 目标

下载小说并每 5 秒展示 1 行

#### 实现方法

1. httplib 实现 http 请求
2. 确认名字以及接口
3. 使用 chrono 实现定时发布

### 实验二

### 实验三

控制海龟模拟器中的小海龟转指定半径的圆

## Appendix

### 常用命令

```bash
# ros2 节点操作
ros2 node list
ros2 node info /turtlesim
# 查看话题内容
ros2 topic echo /turtle1/pose
ros2 topic info /turtle1/pose -v
ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: -1.0}}"
# 查看接口定义
ros2 interface show geometry_msgs/msg/Twist
# 创建接口
ros2 pkg create chapter_interfaces --build-type ament_cmake --dependencies rosidl_default_generators sensor_msgs --license Apache-2.0
```
