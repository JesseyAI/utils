import os
import subprocess
import pandas as pd
import matplotlib.pyplot as plt

def get_txt_files(folder):
    txt_files = []
    for file_name in os.listdir(folder):
        if file_name.endswith('.txt'):
            name_without_extension = os.path.splitext(file_name)[0]  
            txt_files.append({'name': name_without_extension, 'path': os.path.join(folder, file_name)})
    return txt_files

def find_data_in_current(current_folder, data_name):
    # 遍历 current_folder 的下一级目录
    for dir_name in os.listdir(current_folder):
        dir_path = os.path.join(current_folder, dir_name)
        if os.path.isdir(dir_path) and dir_name == data_name:
            # 在 data_name 对应的文件夹下查找 traj.txt
            traj_file_path = os.path.join(dir_path, 'traj.txt')
            if os.path.isfile(traj_file_path):
                return traj_file_path
    return None

def compare_trajectories(gt_file, traj_file):
    command = f"evo_ape tum {gt_file} {traj_file} -va --t_max_diff 0.2"
    try:
        result = subprocess.run(command, shell=True, capture_output=True, text=True)
        # Extract RMSE from the result
        for line in result.stdout.splitlines():
            if 'rmse' in line:
                return float(line.split()[-1])
    except Exception as e:
        print(f"Error running command: {command}\n{e}")
    return None

def plot_results(results, gt_folder):
    df = pd.DataFrame(results)
    
    # 绘制表格
    fig, ax = plt.subplots(figsize=(10, 6))  # 设置图表大小
    ax.axis('tight')
    ax.axis('off')
    table = ax.table(cellText=df.values, colLabels=df.columns, cellLoc='center', loc='center')
    table.scale(1, 2)  # 调整表格的缩放比例
    
    # 设置图片保存路径
    save_dir = os.path.dirname(gt_folder)  # 获取gt_folder的同级目录
    save_path = os.path.join(save_dir, 'trajectory_comparison_results.png')
    
    # 保存为PNG图片
    plt.savefig(save_path, bbox_inches='tight', dpi=300)
    plt.close()
    
    print(f"Results saved as {save_path}")

def main(gt_folder, liwo_folder):
    gt_files = get_txt_files(gt_folder)
    results = []

    for gt_file in gt_files:
        traj_file = find_data_in_current(liwo_folder, gt_file['name'])
        if traj_file:
            rmse = compare_trajectories(gt_file['path'], traj_file)
            if rmse is not None:
                results.append({'name': gt_file['name'], 'rmse': rmse})
                print(f"Compared {gt_file['name']} -> RMSE: {rmse}")
            else:
                print(f"Failed to compare {gt_file['name']}")
        else:
            print(f"No corresponding traj file found for {gt_file['name']}")

    if results:
        plot_results(results, gt_folder)
    else:
        print("No valid results to plot.")

if __name__ == "__main__":
    gt_folder = '/home/qijie/Data/OpenData/tof/gt'  # 替换为gt文件夹的路径
    liwo_folder = '/home/qijie/Data/OpenData/tof/l515_res'  # 替换为liwo文件夹的路径
    main(gt_folder, liwo_folder)
