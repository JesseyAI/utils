import os
import numpy as np
import scipy.interpolate as interpolate
import open3d as o3d

def read_gt_file(gt_file):
    data = np.loadtxt(gt_file)
    timestamps = data[:, 0]
    positions = data[:, 1:4]
    quaternions = data[:, 4:]
    return timestamps, positions, quaternions

def read_pcd_timestamps(mea_folder):
    pcd_files = sorted([f for f in os.listdir(mea_folder) if f.endswith('.pcd')])
    timestamps = np.array([float(f.split('.')[0]) for f in pcd_files])
    return timestamps, pcd_files

def interpolate_pose(gt_timestamps, gt_positions, gt_quaternions, pcd_timestamps):
    pos_interp = interpolate.make_interp_spline(gt_timestamps, gt_positions, k=3)
    quat_interp = interpolate.make_interp_spline(gt_timestamps, gt_quaternions, k=3)
    
    interp_positions = pos_interp(pcd_timestamps)
    interp_quaternions = quat_interp(pcd_timestamps)
    
    return interp_positions, interp_quaternions

def main(data_folder, gt_folder, output_folder):
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    gt_files = sorted([f for f in os.listdir(gt_folder) if f.endswith('.txt')])

    for gt_file in gt_files:
        base_name = os.path.splitext(gt_file)[0]
        gt_path = os.path.join(gt_folder, gt_file)
        
        timestamps, positions, quaternions = read_gt_file(gt_path)

        mea_folder = os.path.join(data_folder, base_name, 'mea')
        if not os.path.exists(mea_folder):
            print(f"Folder {mea_folder} does not exist. Skipping.")
            continue

        pcd_timestamps, pcd_files = read_pcd_timestamps(mea_folder)
        
        interp_positions, interp_quaternions = interpolate_pose(timestamps, positions, quaternions, pcd_timestamps)

        # 将所有pose写入一个txt文件
        output_file = os.path.join(output_folder, f"{base_name}.txt")
        with open(output_file, 'w') as f:
            for i in range(len(pcd_timestamps)):
                f.write(f"{pcd_timestamps[i]} {interp_positions[i, 0]} {interp_positions[i, 1]} {interp_positions[i, 2]} "
                        f"{interp_quaternions[i, 0]} {interp_quaternions[i, 1]} {interp_quaternions[i, 2]} {interp_quaternions[i, 3]}\n")
        print(f"Processed {base_name} and saved to {output_file}")

if __name__ == "__main__":
    data_folder = "/home/qijie/Data/OpenData/tof/l515"  # 替换为你的data文件夹路径
    gt_folder = "/home/qijie/Data/OpenData/tof/gt"  # 替换为你的gt文件夹路径
    output_folder = "/home/qijie/Data/OpenData/tof/gt_slerp"  # 替换为你想保存结果的output文件夹路径
    
    main(data_folder, gt_folder, output_folder)
