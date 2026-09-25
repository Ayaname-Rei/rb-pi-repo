from ultralytics import YOLO

def main():
    # 1. 加载你刚刚下载的模型权重
    model = YOLO("D:/RoboGame/yolo/best.pt")

    # 2. 对文件夹中的所有照片进行批量预测
    results = model.predict(
        source="./assets/test_1.jpg", # 测试图片所在的文件夹
        save=True,       # 必须设为 True，才会把画好框的图片保存下来
        conf=0.5,        # 置信度阈值：只有模型确信度大于 50% 时才输出框
        device="cpu"     # 使用本地 CPU 进行推理
    )
    
    print("所有测试图片已处理完毕！")

if __name__ == '__main__':
    main()


