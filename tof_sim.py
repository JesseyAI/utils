import open3d as o3d
import numpy as np
import time
import os

def load_point_cloud_from_txt(file_path):
    """从TXT文件中加载点云数据"""
    data = np.loadtxt(file_path, delimiter=' ')
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(data[:, :3])
    return pcd

def points_in_fov(pcd, R, h_fov, v_fov, max_distance):
    """过滤出视场内且在最大观测距离内的点云"""
    points = np.asarray(pcd.points).dot(R.T)
    distances = np.linalg.norm(points, axis=1)
    horizontal_angles = np.arctan2(points[:, 0], points[:, 2])
    vertical_angles = np.arcsin(points[:, 1] / distances)
    mask = (np.abs(horizontal_angles) < h_fov / 2) & (np.abs(vertical_angles) < v_fov / 2) & (distances < max_distance)
    return pcd.select_by_index(np.where(mask)[0], invert=True)

# 载入点云
pcd = load_point_cloud_from_txt("/home/qijie/Data/reloc_data/hypercube.txt")
pcd.paint_uniform_color([1, 1, 1])  # 设置点云颜色为白色

# 相机参数
horizontal_fov = np.radians(90)  # 水平FOV, 90度
vertical_fov = np.radians(60)    # 垂直FOV, 60度
max_distance = 7.0  # 最大观测距离，7米

# 可视化设置
vis = o3d.visualization.Visualizer()
vis.create_window()
vis.add_geometry(pcd)

# 设置初始俯视视角
ctr = vis.get_view_control()
ctr.set_front([0, -1, -0.5])  # 设置视向向量
ctr.set_lookat([0, 0, 0])  # 设置视点位置
ctr.set_up([0, 0, 1])  # 设置向上向量
ctr.set_zoom(0.5)  # 设置缩放

output_dir = "/home/qijie/Data/reloc_data/aaaa"
os.makedirs(output_dir, exist_ok=True)  # 确保目录存在

angle = 0
frame = 0
while True:
    R = o3d.geometry.get_rotation_matrix_from_axis_angle([0, angle, 0])
    fov_pcd = points_in_fov(pcd, R, horizontal_fov, vertical_fov, max_distance)
    fov_pcd.paint_uniform_color([1, 0, 0])  # 设置点云颜色为红色
    
    vis.add_geometry(fov_pcd)
    vis.update_geometry(fov_pcd)
    vis.poll_events()
    vis.update_renderer()

    # 保存当前帧的点云
    frame_path = os.path.join(output_dir, f"{frame}.txt")
    np.savetxt(frame_path, np.asarray(fov_pcd.points), fmt='%f %f %f')

    time.sleep(1/np.pi)  # 每秒旋转1弧度，即每帧约为1/π秒

    angle += 1  # 更新角度
    if angle >= 2 * np.pi:  # 如果旋转超过360度，则重新开始
        angle = 0

    # 清除当前帧的点云，为下一帧做准备
    vis.remove_geometry(fov_pcd)

    frame += 1  # 帧号递增