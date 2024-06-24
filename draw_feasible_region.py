import cv2
import numpy as np

def load_image_from_txt(file_path):
    with open(file_path, 'r') as file:
        # 读取第一行，获取图像的行和列
        rows, cols = map(int, file.readline().strip().split())
        
        # 初始化图像数组，这次使用三通道表示颜色
        image = np.zeros((rows, cols, 3), dtype=np.uint8)
        
        # 读取像素值并填充到图像数组
        for i in range(rows):
            row_data = list(map(int, file.readline().strip().split()))
            for j in range(cols):
                if row_data[j] == 255:
                    color = (255, 255, 255)  # 白色
                elif row_data[j] == 128:
                    color = (0, 0, 255)  # 红色，注意OpenCV使用BGR格式
                else:
                    color = (0, 0, 0)  # 黑色
                image[i, j] = color
        
        # 读取最后一行，包含修改像素的x, y坐标和像素值p
        x, y, p = map(int, file.readline().strip().split())
        
        # 根据像素值p设置颜色并更新图像
        if p == 255:
            new_color = (255, 255, 255)  # 白色
        elif p == 128:
            new_color = (0, 0, 255)  # 红色
        else:
            new_color = (0, 0, 0)  # 黑色
        image[x, y] = new_color
    
    return image

def main():
    # TXT文件路径
    txt_file_path = '/home/qijie/Data/reloc_data/global_reloc/feasible_region/image.txt'
    
    # 加载图像
    image = load_image_from_txt(txt_file_path)
    
    # 放大图像10倍
    image_large = cv2.resize(image, (image.shape[1] * 10, image.shape[0] * 10), interpolation=cv2.INTER_NEAREST)
    
    # 使用OpenCV显示图像
    cv2.namedWindow('Loaded Image', cv2.WINDOW_NORMAL)
    cv2.imshow('Loaded Image', image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()



