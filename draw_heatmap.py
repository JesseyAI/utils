import numpy as np
import matplotlib.pyplot as plt
import os

# 数据文件路径
dir_path = '/home/qijie/Data/reloc_data/global_reloc/feasible_region/heatmap/'
save_path = '/home/qijie/Data/reloc_data/global_reloc/feasible_region/heatmap_im/'  # 指定保存图片的路径

# 设置是否渲染热力图的标志
enable_rendering_heat = True

# 确保保存路径存在
if not os.path.exists(save_path):
    os.makedirs(save_path)

# 筛选出以数字开头的txt文件
files = [f for f in os.listdir(dir_path) if f.startswith(tuple('0123456789')) and f.endswith('.txt')]

# 初始化行列的总权重
total_weights = {}

# 预处理步骤：找到权重总和最大的行和列，并找出最大的weight值
for file_name in files:
    file_path = os.path.join(dir_path, file_name)
    
    # 读取数据
    data = np.loadtxt(file_path)

    # 计算每行和每列的权重总和
    for row, col, weight, flag in data:
        if (row, col) not in total_weights:
            total_weights[(row, col)] = 0
        total_weights[(row, col)] += weight

# 找到权重总和最大的行和列
reloc_r, reloc_c = max(total_weights, key=total_weights.get)
max_weight_sum = total_weights[(reloc_r, reloc_c)]
max_weight = max_weight_sum / len(files)
# max_weight = 0.5
print(f'Max weight sum at row {reloc_r} and col {reloc_c}')
print(f'Max weight value sum is {max_weight_sum}')
print(f'Max weight value is {max_weight}')

# 主循环，生成图像
for file_name in files:
    file_path = os.path.join(dir_path, file_name)
    
    # 读取数据
    data = np.loadtxt(file_path)

    # 确定图像尺寸
    max_row = int(np.max(data[:, 0])) + 1  # 行的最大值
    max_col = int(np.max(data[:, 1])) + 1  # 列的最大值

    # 初始化图像为白色
    image = np.ones((max_row, max_col, 3), dtype=np.uint8) * 220

    # 设置像素颜色
    for row, col, weight, flag in data:
        if row == reloc_r and col == reloc_c:
            image[int(row), int(col)] = [0, 255, 0]  # 设置为绿色
        elif flag == 1:
            image[int(row), int(col)] = [0, 0, 0]  # 标志为1，设置为黑色
        elif flag == 0:
            continue  # 标志为0，保持为白色

    # 创建图形并显示图像
    plt.figure(figsize=(10, 8))
    plt.imshow(image)
    plt.axis('off')  # 隐藏坐标轴

    # 在图像上添加权重值
    for row, col, weight, flag in data:
        # 以较小的字体大小添加权重值
        plt.text(col, row, f'{weight:.1f}', color='red', ha='center', va='center', fontsize=8)

    # 设置图像大小为原来的两倍
    # fig = plt.gcf()
    # fig.set_size_inches(fig.get_size_inches() * 2)
    
    # 保存图像
    save_file_path = os.path.join(save_path, file_name.replace('.txt', '.png'))
    plt.savefig(save_file_path)
    plt.close()  # 关闭图形，以防内存问题

    # 如果启用热力图渲染
    if enable_rendering_heat:
        # 初始化热力图矩阵
        heatmap = np.zeros((max_row, max_col))

        # 填充热力图矩阵
        for row, col, weight, flag in data:
            heatmap[int(row), int(col)] = min(weight, max_weight)  # weight 超过 max_weight 也视为 max_weight

        # 创建热力图
        plt.figure(figsize=(10, 8))
        plt.imshow(heatmap, cmap='jet', interpolation='nearest', vmin=0.0, vmax=max_weight)
        plt.colorbar(label='Weight')

        # 添加网格
        plt.grid(which='both', color='grey', linestyle='-', linewidth=0.5)
        plt.xticks(np.arange(-0.5, max_col, 1), [])
        plt.yticks(np.arange(-0.5, max_row, 1), [])

        # 设置图像大小为原来的两倍
        fig = plt.gcf()
        fig.set_size_inches(fig.get_size_inches() * 2)
        
        # 保存热力图
        heat_save_file_path = os.path.join(save_path, 'heat_' + file_name.replace('.txt', '.png'))
        plt.savefig(heat_save_file_path)
        plt.close()  # 关闭图形，以防内存问题

print("所有图像已生成并保存.")
