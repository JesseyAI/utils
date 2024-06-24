#!/bin/bash


# 目录路径
MAP_PATH=$1
MAP_CORNERS_PATH=$2
DIR_PATH=$3

# 检查提供的目录是否存在
if [ ! -d "$DIR_PATH" ]; then
    echo "错误：目录不存在。"
    exit 1
fi

# 遍历目录中所有的 .txt 文件
for txt_file in "$DIR_PATH"/*.txt
do
    # 检查是否存在.txt文件
    if [ ! -e "$txt_file" ]; then
        echo "目录中没有找到.txt文件。"
        exit 1
    fi

    echo "正在处理 $txt_file..."
    python3 draw_matches.py $MAP_PATH $MAP_CORNERS_PATH "$txt_file"
done

echo "所有文件处理完毕。"
