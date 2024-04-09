# This is a repo for utils

|工具名|用途|
|---|------|
|ros2_bag_covert|将ros2的bag转化成txt格式|
|compare_pose|批量对比pose|
|draw_corner_and_edge|绘制corner和edge|


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
## **2. compare_pose**
**compare_pose**：调用evo，批量对比真值和slam的轨迹
### 2.1 Requirement
* evo
### 2.2 使用方法
```python
pyhon3 compare_pose
```

## **3. draw_corner_and_edge**
**draw_corner_and_edge**：使用open3D绘制corner和edge
* open3d
### 2.2 使用方法
```python
pyhon3 draw_corner_and_edge
```
