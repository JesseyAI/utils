import os
import shutil

def natural_sort_key(s):
    # 提取文件名中的数字部分并转换为整数
    return int(s.split('.')[0])

def process_files(mapping_path, begin_idx, save_path):
    # 确保保存路径存在，如果不存在则创建
    if not os.path.exists(save_path):
        os.makedirs(save_path)

    # 获取所有以数字开头的txt文件，并进行自然排序
    files = [f for f in os.listdir(mapping_path) if f.startswith(tuple('0123456789')) and f.endswith('.txt')]
    files.sort(key=natural_sort_key)

    # 读取begin_idx对应的文件，获取时间戳
    target_file = os.path.join(mapping_path, files[begin_idx])
    with open(target_file, 'r') as file:
        first_line = file.readline()
        timestamp = float(first_line.strip()) - 0.05  # 假设时间戳是每个文件第一行的唯一内容

    # 处理wheel_mea.txt文件
    wheel_mea_path = os.path.join(mapping_path, 'wheel_mea.txt')
    save_wheel_mea_path = os.path.join(save_path, 'wheel_mea.txt')
    with open(wheel_mea_path, 'r') as wheel_file, open(save_wheel_mea_path, 'w') as output_file:
        for line in wheel_file:
            parts = line.strip().split()
            line_timestamp = float(parts[2])  # 第三列是时间戳
            if line_timestamp >= timestamp:
                output_file.write(line)

    # 重命名后续的200个txt文件
    rename_count = min(500, len(files) - begin_idx - 1)
    for i in range(rename_count):
        old_path = os.path.join(mapping_path, files[begin_idx + 1 + i])
        new_name = f'{i}.txt'
        new_path = os.path.join(save_path, new_name)
        shutil.copy(old_path, new_path)  # 复制并重命名到新路径

    print(f"完成处理，共重命名并保存了{rename_count}个文件。")

# 参数设置
mapping_path = '/home/qijie/Data/Measurements/Reloc/long_corridor/D01_0611/longcorridor_mapping'  # 替换为实际路径
begin_idx = 614  # 例如，使用索引3开始处理
save_path = '/home/qijie/Data/Measurements/Reloc/long_corridor/D01_0611/longcorridor_reloc7'  # 替换为实际保存路径

process_files(mapping_path, begin_idx, save_path)
