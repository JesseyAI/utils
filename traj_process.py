import numpy as np

def rotate_trajectory(file_path):
    # 读取文件
    data = []
    with open(file_path, 'r') as file:
        for line in file:
            if line.strip():  # 跳过空行
                data.append(list(map(float, line.strip().split())))

    data = np.array(data)
    
    # 分离数据
    timestamps = data[:, 0]
    positions = data[:, 1:4]
    quaternions = data[:, 4:]

    # 旋转180度，即绕z轴旋转180度
    # 对位置进行旋转（相当于将 x, y 坐标取反）
    rotated_positions = positions.copy()
    rotated_positions[:, 0] = -positions[:, 0]
    rotated_positions[:, 1] = -positions[:, 1]

    # 对四元数进行旋转（绕z轴旋转180度）
    rotated_quaternions = quaternions.copy()
    rotated_quaternions[:, 2] = -quaternions[:, 2]  # qz取反
    rotated_quaternions[:, 3] = -quaternions[:, 3]  # qw取反

    # 将旋转后的数据组合起来
    rotated_data = np.hstack((timestamps[:, np.newaxis], rotated_positions, rotated_quaternions))

    # 保存到新的文件
    new_file_path = file_path.replace('.txt', '_new.txt')
    np.savetxt(new_file_path, rotated_data, fmt='%.6f')
    print(f"旋转后的轨迹已保存为: {new_file_path}")

# 使用函数
rotate_trajectory('/home/qijie/Data/Nebula200/BackendData/20240813/north/opt_traj.txt')
