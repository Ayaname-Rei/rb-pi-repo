from ultralytics import YOLO

def main():
    # 加载官方的轻量级预训练模型 (自动下载)
    model = YOLO("yolo26n.pt") 

    # 启动训练
    results = model.train(
        data="D:/RoboGame/yolo/data.yaml",
        epochs=100,          # 训练 100 轮
        imgsz=640,           # 统一缩放到的图像尺寸
        batch=16,            # 每次处理的图片数 (若电脑内存/显存卡顿，可改为 8 或 4)
        hsv_h=0.0,           # 【核心关键】关闭色相扰动，保全紫块和橙块的绝对颜色！
        hsv_s=0.2,           # 允许轻微饱和度变化 (适应反光)
        hsv_v=0.3,           # 允许轻微明度变化 (适应阴影)
        workers=2,           # Windows 下建议设为 0 或 2
        device='cpu'         # 如果你的电脑有 NVIDIA 显卡并配好了环境，这里改成 0，速度会快几十倍
    )

if __name__ == '__main__':
    main()


    