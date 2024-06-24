import cv2
import numpy as np
import os
import re
import random

# 定义像素分辨率：每米多少像素
PIXELS_PER_METER = 10

# 读取数据
def load_data(file_path):
    voxel_points = []  # 用于存储最后一列为1的点的坐标
    edges = {}     # 用于存储最后一列大于等于0的点的坐标

    # 检查文件是否存在
    if not os.path.exists(file_path):
        return voxel_points, edges  # 如果文件不存在，直接返回空的列表和字典

    with open(file_path, 'r') as file:
        for line in file:
            parts = line.strip().split()
            if len(parts) >= 4:  # 确保每行有四个数据
                x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                flag = int(parts[3])

                # 处理标志位为1的坐标
                if flag == -1:
                    voxel_points.append((x, y))

                # 处理标志位大于等于0的坐标，存储到字典中
                if flag >= 0:
                    if flag not in edges:
                        edges[flag] = []
                    edges[flag].append((x, y))

    return voxel_points, edges

# 找到最小和最大点
def find_min_max_points(voxel_points, edges):
    # 合并来自 points_flag_1 的点和来自 points_map 的所有点
    all_points = list(voxel_points)  # 开始时包括所有 points_flag_1 的点

    # 将 points_map 中的所有点添加到 all_points 列表中
    for sublist in edges.values():
        all_points.extend(sublist)

    # 检查合并后的点列表是否为空
    if not all_points:
        return None, None  # 如果没有点，则返回None

    # 计算最小和最大坐标
    min_x = min(all_points, key=lambda point: point[0])[0]
    min_y = min(all_points, key=lambda point: point[1])[1]
    max_x = max(all_points, key=lambda point: point[0])[0]
    max_y = max(all_points, key=lambda point: point[1])[1]
    
    return (min_x, min_y), (max_x, max_y)

# 绘制点集
def draw_points(voxel_points, edges, min_point, image, color_voxel, color_edge):

    # 绘制每个点
    for point in voxel_points:
        x = int((point[0] - min_point[0]) * PIXELS_PER_METER)
        y = int((point[1] - min_point[1]) * PIXELS_PER_METER)
        cv2.circle(image, (x, y), 1, color_voxel, -1)  


    for key, points in edges.items():
        edge_color = color_edge[key]  # 根据键值获取颜色
        for point in points:
            x = int((point[0] - min_point[0]) * PIXELS_PER_METER)
            y = int((point[1] - min_point[1]) * PIXELS_PER_METER)
            cv2.circle(image, (x, y), 2, edge_color, -1)

    return image


def initialize_color_arrays(map_edges, submap_edges, map_color_array, submap_color_array):
    # 为 map_edges 中的每个键生成随机颜色
    for key in map_edges:
        if key not in map_color_array:
            # 生成一个随机颜色，每个颜色分量的值在0到255之间
            random_color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
            map_color_array[key] = random_color
            #map_color_array[key] = (0, 255, 0)

    # 为 submap_edges 中的每个键分配白色
    for key in submap_edges:
        if key not in submap_color_array:
            # 固定颜色为白色
            submap_color_array[key] = (0, 255, 0)


def show_all(file_path):
    
    # 用正则表达式找到所有以数字开头的txt文件
    pattern = re.compile(r'^-?\d+\.txt$')
    files = [f for f in os.listdir(file_path) if pattern.match(f)]

    # 从文件名中提取数字，并转换为整型数组
    number_array = [int(re.findall(r'^-?\d+', f)[0]) for f in files]

    # 对数组进行从大到小的排序
    number_array.sort(reverse=True)

    map_filename_temp = f"{file_path}/{1}.txt"
    map_voxel_points_temp, map_edges_temp = load_data(map_filename_temp)
    #min_point, max_point = find_min_max_points(map_voxel_points_temp, map_edges_temp)

    min_point = [-43.8702, -29.2706]
    max_point = [17.1472, 28.2632]

    # 确定图像尺寸和点的偏移量
    width = int((max_point[0] - min_point[0]) * PIXELS_PER_METER) + 1
    height = int((max_point[1] - min_point[1]) * PIXELS_PER_METER) + 1

    map_color_array = {}
    submap_color_array = {}

    png_dir = os.path.join(file_path, "png")
    if not os.path.exists(png_dir):
        os.makedirs(png_dir)

    for file_idx in number_array:
        map_filename = f"{file_path}/{file_idx}.txt"
        submap_filename = f"{file_path}/submap_{file_idx}.txt"

        map_voxel_points, map_edges = load_data(map_filename)
        submap_voxel_points, submap_edges = load_data(submap_filename)

        # 如果subamp_points为空，则跳过当前循环
        # if not subamp_points:
        #     continue

        # 创建一个黑色的背景图
        image = np.zeros((height+100, width+100, 3), dtype=np.uint8)

        initialize_color_arrays(map_edges,submap_edges, map_color_array, submap_color_array)

        # min_point, max_point = find_min_max_points(map_points)
        print(f"Showing points for file index: {file_idx}")
        #draw_points(map_voxel_points, map_edges, min_point, image, (0,255,0), map_color_array)

        draw_points(submap_voxel_points, submap_edges, min_point, image, (0,0,255), submap_color_array)

        cv2.imshow(f'{file_idx}', image)
        cv2.waitKey(0)
        # image_filename = os.path.join(png_dir, f"{file_idx}.png")
        # cv2.imwrite(image_filename, image)
    

# 主程序
def main():
    file_path = '/home/qijie/Data/reloc_data/hypercube/proj_im'  # 文件路径

    show_all(file_path)

    # file_idx = -1

    # # 构造文件名
    # map_filename = f"{file_path}/{file_idx}.txt"
    # submap_filename = f"{file_path}/submap_{file_idx}.txt"

    # map_points = load_data(map_filename)
    # subamp_points = load_data(submap_filename)

    # min_point, max_point = find_min_max_points(map_points)

    # image = draw_points(subamp_points, map_points, min_point, max_point)

    # cv2.imshow('Points', image)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
