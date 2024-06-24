import matplotlib.pyplot as plt
import pandas as pd
import os

# 设置文件夹路径
folder_path = '/home/qijie/Data/Measurements/Reloc/0623_1'

# 初始化存储结果的列表
reloc_results = []

# 遍历文件夹下所有txt文件
for filename in os.listdir(folder_path):
    if filename.endswith(".txt"):
        # 构造完整的文件路径
        file_path = os.path.join(folder_path, filename)
        # 读取txt文件
        try:
            data = pd.read_csv(file_path, header=None, sep=' ')  # 假设数据是用制表符分隔的
            # 遍历文件中的每一行
            for index, row in data.iterrows():
                # 将文件名（去掉.txt扩展名）和行数据添加到结果列表
                 reloc_results.append([f"{filename[:-4]}", f"{row[0]}, {row[1]}, {row[2]}", f"{row[3]}"])
        except Exception as e:
            print(f"Error reading {file_path}: {e}")

# Create a DataFrame from the results
df = pd.DataFrame(reloc_results, columns=['Scene', 'Submap Name', 'Result'])

# Plotting the results as a table
num_rows = len(reloc_results)
fig_height = max(6, num_rows * 0.3)  # Adjust height based on number of rows
fig, ax = plt.subplots(figsize=(12, fig_height))  # Adjust size as needed
ax.axis('tight')
ax.axis('off')

# 创建表格并存储返回的表格对象
the_table = ax.table(cellText=df.values, colLabels=df.columns, cellLoc='center', loc='center')

cell_height = min(0.05, 1 / (num_rows + 1))  # Adjust cell height dynamically, limit cell height to 0.05

# 设置表格字体大小和单元格大小
table_props = the_table.properties()
table_cells = table_props['child_artists']
for cell in table_cells:
    cell.set_height(cell_height)  # 调整单元格的高度
    cell.set_fontsize(14)  # 调整字体大小

# 单独调整标题行
for (i, j), cell in the_table.get_celld().items():
    if i == 0:  # 只针对标题行
        cell.set_fontsize(16)  # 标题字体更大
        cell.set_facecolor('#D8D8D8')  # 可以给标题设置一个不同的背景颜色
    if i > 0 and j == 2:  # 只检查结果列
        try:
            result_value = int(df.iloc[i-1, j])
            if result_value == 0:
                cell.set_facecolor('red')
        except ValueError:
            continue

plt.savefig( '/home/qijie/Data/Measurements/Reloc/relocation_results.png')



