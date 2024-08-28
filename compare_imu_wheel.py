import pandas as pd
import matplotlib.pyplot as plt

# 读取IMU数据
imu_data = pd.read_csv('/home/qijie/Data/Stereo/res/imu.txt', delim_whitespace=True, names=['timestamp', 'speed_x', 'speed_y', 'speed_z'])

# 读取Wheel数据
wheel_data = pd.read_csv('/home/qijie/Data/Stereo/res/wheel.txt', delim_whitespace=True, names=['timestamp', 'speed_x', 'speed_y', 'speed_z'])

# 确保数据按时间戳排序
imu_data.sort_values('timestamp', inplace=True)
wheel_data.sort_values('timestamp', inplace=True)

# 合并数据，按时间戳对齐，只保留匹配的行
merged_data = pd.merge(imu_data, wheel_data, on='timestamp', suffixes=('_imu', '_wheel'), how='inner')

# 计算速度差异
merged_data['speed_x_diff'] = merged_data['speed_x_imu'] - merged_data['speed_x_wheel']
merged_data['speed_y_diff'] = merged_data['speed_y_imu'] - merged_data['speed_y_wheel']
merged_data['speed_z_diff'] = merged_data['speed_z_imu'] - merged_data['speed_z_wheel']

# 打印结果
print(merged_data)

# 保存结果到Excel文件，并设置列宽
output_file = 'comparison_result.xlsx'
with pd.ExcelWriter(output_file, engine='xlsxwriter') as writer:
    merged_data.to_excel(writer, sheet_name='Comparison', index=False)
    
    # 获取 xlsxwriter workbook 和 worksheet 对象
    workbook  = writer.book
    worksheet = writer.sheets['Comparison']
    
    # 设置列宽
    for i, col in enumerate(merged_data.columns):
        max_length = max(merged_data[col].astype(str).map(len).max(), len(col)) + 2  # 设置合适的列宽
        worksheet.set_column(i, i, max_length)

print(f"Results saved to {output_file}")
