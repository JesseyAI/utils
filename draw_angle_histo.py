import os
import numpy as np
import matplotlib.pyplot as plt

def plot_histogram_from_txt(file_path, output_folder):
    # 读取数据
    data = np.loadtxt(file_path, delimiter=' ')
    
    # 分离bin索引和计数
    bin_idx = data[:, 0]  # 第一列是bin索引
    counts = data[:, 1]   # 第二列是计数

    # 获取文件名的基名（去掉路径和后缀）
    base_name = os.path.basename(file_path)
    base_name_no_ext = os.path.splitext(base_name)[0]

    # 绘制直方图
    plt.figure(figsize=(10, 6))
    plt.bar(bin_idx, counts, width=0.8, color='blue', alpha=0.7)

    # 设置x轴的刻度标签为每个bin的idx
    plt.xticks(bin_idx, [str(int(idx)) for idx in bin_idx])

    # 添加标题和标签
    plt.title(f'Plane Histogram {base_name_no_ext}')
    plt.xlabel('Bin Index')
    plt.ylabel('Count')

    # 移除网格
    plt.grid(False)

    # 保存图像
    output_path = os.path.join(output_folder, f'{base_name_no_ext}.png')
    plt.savefig(output_path)
    plt.close()

def process_all_histograms(input_folder):
    # 遍历文件夹中的所有文件
    for file_name in os.listdir(input_folder):
        if file_name.endswith('.txt'):  # 只处理txt文件
            file_path = os.path.join(input_folder, file_name)
            plot_histogram_from_txt(file_path, input_folder)

if __name__ == "__main__":
    # 设置文件夹路径
    input_folder = "/home/qijie/Data/Nebula200/nebula220/office_cq/map/histogram"  # 替换为你的文件夹路径

    # 处理文件夹中的所有直方图
    process_all_histograms(input_folder)
