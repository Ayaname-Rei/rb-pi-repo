import os

def batch_rename_images():
    # 1. 设置图片所在的文件夹路径 (请替换为你自己的实际路径)
    # 在路径前加 r 可以避免 Windows 系统下的转义字符错误
    folder_path = r"D:\RoboGame\yolo\RG_Dataset\raw_images" 

    # 2. 定义你要重命名的图片格式
    valid_extensions = ('.jpg')

    # 3. 获取文件夹中的所有文件
    try:
        files = os.listdir(folder_path)
    except FileNotFoundError:
        print("指定的文件夹路径不存在，请检查后重试！")
        return

    # 4. 初始化计数器
    count = 1

    # 5. 遍历并重命名
    for filename in files:
        # 获取文件的扩展名，并转为小写 (例如将 .JPG 转为 .jpg)
        ext = os.path.splitext(filename)[1].lower()
        
        # 检查该文件是否为图片
        if ext in valid_extensions:
            # 构造新的文件名
            # zfill(2) 的作用是补零，因为你有58张图，补零后会变成 01 到 58，这样在电脑里排序会更整齐
            new_name = f"image_{str(count).zfill(2)}{ext}"
            
            # 拼接完整的文件路径
            old_file_path = os.path.join(folder_path, filename)
            new_file_path = os.path.join(folder_path, new_name)
            
            # 执行重命名
            os.rename(old_file_path, new_file_path)
            print(f"成功: {filename} -> {new_name}")
            
            count += 1

    print(f"\n批量重命名完成！共重命名了 {count - 1} 张图片。")

# 运行函数
if __name__ == "__main__":
    batch_rename_images()