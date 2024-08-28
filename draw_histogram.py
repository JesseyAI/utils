import os
import matplotlib.pyplot as plt
from collections import Counter

# 设定文件夹路径
folder_path = '/home/qijie/Data/Measurements/Reloc/3d_bbs/all_hands/histogram/'

# 获取文件夹中的所有 .txt 文件
files = [f for f in os.listdir(folder_path) if f.endswith('.txt')]

# 读取文件并绘制直方图
for file in files:
    file_path = os.path.join(folder_path, file)
    
    # 读取txt文件中的数据
    with open(file_path, 'r') as f:
        data = f.readlines()
    
    # 将数据转换为整数列表
    data = [int(line.strip()) for line in data]
    
    # 统计每个bin (0到8) 的出现次数
    bin_range = range(9)
    counts = Counter(data)
    histogram_data = [counts.get(i, 0) for i in bin_range]
    
    # 绘制直方图
    plt.figure()
    plt.bar(bin_range, histogram_data, width=0.8, edgecolor='black', align='center')
    plt.xlabel('Bin')
    plt.ylabel('Frequency')
    plt.title(f'Histogram for {file}')
    plt.xticks(bin_range)  # 设置x轴刻度为0到8
    
    # 保存直方图为图片
    output_file = os.path.join(folder_path, f'{os.path.splitext(file)[0]}.png')
    plt.savefig(output_file)
    plt.close()

print("所有直方图已保存为图片。")
