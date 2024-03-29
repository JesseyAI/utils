# This is a repo for utils

|工具名|用途|
|---|------|
|ros2_bag_covert|将ros2的bag转化成txt格式|


## **1. ros2_bag_covert**

**ros2_bag_covert**：将ros2的bag转化成txt格式

### 1.1 支持的话题类型
* sensor_msgs::msg::PointCloud2: 点云话题
* irobot_create_msgs::msg::WheelVels: 轮速计话题
* sensor_msgs::msg::Imu: Imu话题
* livox_ros_driver2::msg::CustomMsg: Livox点云话题

### 1.2 使用方法
```
python3 ros2_bag_covert path/to/bag/.db3 path/to/save/data
```
