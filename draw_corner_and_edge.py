import open3d as o3d
import numpy as np
from time import sleep

def load_corner_triplet_txt(file_path):
    points = []
    with open(file_path, 'r') as file:
        for line in file:
            coordinates = [float(x) for x in line.split()]
            for i in range(0, len(coordinates), 3):
                point = coordinates[i:i+3]
                points.append(point)
    return points            
 
def draw_signgle_triplet(points):
    #绘制顶点
    #连接的顺序，封闭链接
    lines = [[0, 1], [1, 2], [2, 0]]
    color = [[0, 1, 0] for i in range(len(lines))] 
    #添加顶点，点云
    points_pcd = o3d.geometry.PointCloud()
    points_pcd.points = o3d.utility.Vector3dVector(points)
    points_pcd.paint_uniform_color([1, 0, 0]) #点云颜色
 
    #绘制线条
    lines_pcd = o3d.geometry.LineSet()
    lines_pcd.lines = o3d.utility.Vector2iVector(lines)
    lines_pcd.colors = o3d.utility.Vector3dVector(color) #线条颜色
    lines_pcd.points = o3d.utility.Vector3dVector(points)
 
    return lines_pcd, points_pcd

def draw_corner_triplet(file_path):
    corners = load_corner_triplet_txt(file_path)

    triplet_pcd = []
    corners_pcd = []

    for i in range(0, len(corners), 3):
        polygon_points = corners[i:i+3]
        lines_pcd, points_pcd = draw_signgle_triplet(polygon_points)
        triplet_pcd.append(lines_pcd)
        corners_pcd.append(points_pcd)

    return triplet_pcd, corners_pcd

def get_point_cloud(file_path, color):
    points = []
    with open(file_path, 'r') as file:
        for line in file:
            coordinates = [float(x) for x in line.split()]
            points.append(coordinates)
    
    # Create a PointCloud object
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points)
    cloud.paint_uniform_color(color)
    
    return cloud

def VizSingleCornerTripletOnce(lines, points, corner_cloud, edges_cloud):
    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window(window_name='CESI')
    # vis.toggle_full_screen() #全屏
    
    #设置
    opt = vis.get_render_option()
    opt.background_color = np.asarray([0, 0, 0]) #背景
    opt.point_size = 5 #点云大小

    vis.add_geometry(corner_cloud) 
    vis.add_geometry(edges_cloud)       

    #vis.add_geometry(axis_pcd)
    for point_i in points:
        vis.add_geometry(point_i)

    run_flag = True
    def callback1(vis, act, mods):
        nonlocal run_flag
        if act == 1:
            run_flag = not run_flag # 当按键被按下，对run_flag取反，控制播放/暂停
        elif act == 2:
            exit(0) # 当按键被长按，退出程序
        else:
            pass
        return True
    
    # idx = 0
    # vis.add_geometry(lines[idx])
    # def callback2(vis):
    #     nonlocal run_flag, lines, idx
    #     if run_flag and idx < len(lines):
    #         vis.add_geometry(lines[idx])
    #         vis.poll_events() # 轮询event 
    #         vis.update_renderer()
    #         #vis.remove_geometry(lines[idx])
    #         idx += 1
    #         sleep(0.1)

    for line_i in lines:
        vis.add_geometry(line_i)     

    # vis.register_key_action_callback(32, callback1)
    # vis.register_animation_callback(callback2) 
    vis.run()
    vis.destroy_window() 

def VizAllCornerTriplet(lines, points, corner_cloud, edges_cloud):
    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window(window_name='CESI')
    # vis.toggle_full_screen() #全屏
    
    #设置
    opt = vis.get_render_option()
    opt.background_color = np.asarray([0, 0, 0]) #背景
    opt.point_size = 5 #点云大小

    vis.add_geometry(corner_cloud) 
    vis.add_geometry(edges_cloud)       

    #vis.add_geometry(axis_pcd)
    for point_i in points:
        vis.add_geometry(point_i)

    for line_i in lines:
        vis.add_geometry(line_i)     

    vis.run()
    vis.destroy_window() 

if __name__ == "__main__":
    
    corner_triplet_path = '/home/qijie/Data/reloc_data/hypercube/1101/1/res/corner_triplet1.txt'

    corner_cloud_path = '/home/qijie/Data/reloc_data/hypercube/1101/1/conrners.txt'

    edges_cloud_path = '/home/qijie/Data/reloc_data/hypercube/1101/1/edges.txt'

    lines, points = draw_corner_triplet(corner_triplet_path)

    corner_cloud_color = [1,0,0]
    corner_cloud = get_point_cloud(corner_cloud_path,corner_cloud_color)

    edges_cloud_color = [1,1,1]
    edges_cloud = get_point_cloud(edges_cloud_path,edges_cloud_color)

    VizAllCornerTriplet(lines, points, corner_cloud, edges_cloud)

    # axis_pcd = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=[0, 0, 0])
 
   
