import os
import cv2

# ==================== 1. 路径与类别配置 ====================
BASE_DIR = r"D:\RoboGame\yolo\RG_Dataset"
IMG_DIRS = {
    "train": os.path.join(BASE_DIR, "images", "train"),
    "val": os.path.join(BASE_DIR, "images", "val")
}
LABEL_DIRS = {
    "train": os.path.join(BASE_DIR, "labels", "train"),
    "val": os.path.join(BASE_DIR, "labels", "val")
}

# 针对你的需求定制的颜色 (注意：OpenCV 使用的是 BGR 格式，不是 RGB)
CLASSES = {
    0: {"name": "Purple_Block", "color": (211, 0, 148)},  # 紫色框
    1: {"name": "Orange_Block", "color": (0, 140, 255)}   # 橙色框
}
VALID_EXTS = (".jpg", ".jpeg", ".png", ".bmp")

# ==================== 2. 全局状态变量 ====================
drawing = False
ix, iy = -1, -1
current_box = None
current_class = 0
boxes = []  # 存储格式: [(class_id, x1, y1, x2, y2)]

# ==================== 3. 核心功能函数 ====================
def mouse_callback(event, x, y, flags, param):
    """处理鼠标拖拽画框事件"""
    global ix, iy, drawing, current_box, boxes, current_class
    if event == cv2.EVENT_LBUTTONDOWN:
        drawing = True
        ix, iy = x, y
        current_box = (ix, iy, x, y)
    elif event == cv2.EVENT_MOUSEMOVE:
        if drawing:
            current_box = (ix, iy, x, y)
    elif event == cv2.EVENT_LBUTTONUP:
        drawing = False
        x1, y1 = min(ix, x), min(iy, y)
        x2, y2 = max(ix, x), max(iy, y)
        # 过滤掉像素小于 5x5 的误触点击
        if abs(x2 - x1) > 5 and abs(y2 - y1) > 5:
            boxes.append((current_class, x1, y1, x2, y2))
        current_box = None

def convert_to_yolo(box, img_w, img_h):
    """将像素坐标 (x1,y1,x2,y2) 转换为 YOLO 归一化中心点坐标"""
    cls_id, x1, y1, x2, y2 = box
    x_center = ((x1 + x2) / 2.0) / img_w
    y_center = ((y1 + y2) / 2.0) / img_h
    width = (x2 - x1) / img_w
    height = (y2 - y1) / img_h
    return f"{cls_id} {x_center:.6f} {y_center:.6f} {width:.6f} {height:.6f}\n"

def load_existing_labels(txt_path, img_w, img_h):
    """如果该图片已经标过，加载并还原为像素坐标以便显示"""
    loaded_boxes = []
    if os.path.exists(txt_path):
        with open(txt_path, "r") as f:
            for line in f.readlines():
                parts = line.strip().split()
                if len(parts) == 5:
                    c_id, xc, yc, bw, bh = map(float, parts)
                    x1 = int((xc - bw / 2) * img_w)
                    y1 = int((yc - bh / 2) * img_h)
                    x2 = int((xc + bw / 2) * img_w)
                    y2 = int((yc + bh / 2) * img_h)
                    loaded_boxes.append((int(c_id), x1, y1, x2, y2))
    return loaded_boxes

# ==================== 4. 主程序 ====================
def main():
    global current_box, boxes, current_class
    cv2.namedWindow("YOLO Annotator", cv2.WINDOW_NORMAL)
    cv2.setMouseCallback("YOLO Annotator", mouse_callback)

    print("\n" + "="*45)
    print("【RoboGame 视觉打标工具】")
    print(" 鼠标左键拖拽 : 画出目标边界框")
    print(" 按键盘 [0]   : 切换为 紫色块 (Object A)")
    print(" 按键盘 [1]   : 切换为 橙色块 (Object B)")
    print(" 按键盘 [u]   : 撤销上一个框 (Undo)")
    print(" 按键盘 [c]   : 清空当前图片所有框 (Clear)")
    print(" 按键盘 [Space]: 保存并进入下一张")
    print(" 按键盘 [q]   : 退出程序")
    print("="*45 + "\n")

    for split in ["train", "val"]:
        img_dir = IMG_DIRS[split]
        label_dir = LABEL_DIRS[split]
        os.makedirs(label_dir, exist_ok=True)  # 确保标签目录存在

        if not os.path.exists(img_dir):
            continue

        img_files = sorted([f for f in os.listdir(img_dir) if f.lower().endswith(VALID_EXTS)])
        
        for img_name in img_files:
            img_path = os.path.join(img_dir, img_name)
            base_name, _ = os.path.splitext(img_name)
            txt_path = os.path.join(label_dir, f"{base_name}.txt")

            img = cv2.imread(img_path)
            if img is None:
                continue

            h, w = img.shape[:2]
            boxes = load_existing_labels(txt_path, w, h)

            while True:
                display_img = img.copy()

                # 1. 绘制已保存的框
                for c_id, x1, y1, x2, y2 in boxes:
                    color = CLASSES[c_id]["color"]
                    label_text = CLASSES[c_id]["name"]
                    cv2.rectangle(display_img, (x1, y1), (x2, y2), color, 2)
                    cv2.putText(display_img, label_text, (x1, max(y1 - 5, 15)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

                # 2. 绘制正在拖拽的临时框
                if current_box:
                    cx1, cy1, cx2, cy2 = current_box
                    temp_color = CLASSES[current_class]["color"]
                    cv2.rectangle(display_img, (cx1, cy1), (cx2, cy2), temp_color, 1)

                # 3. 顶部 HUD 状态栏
                info_text = f"[{split.upper()}] {img_name} | Key: [{current_class}] {CLASSES[current_class]['name']} | Boxes: {len(boxes)}"
                cv2.rectangle(display_img, (0, 0), (w, 35), (40, 40, 40), -1)
                cv2.putText(display_img, info_text, (10, 24),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

                cv2.imshow("YOLO Annotator", display_img)
                key = cv2.waitKey(20) & 0xFF

                if key == ord('q'):
                    print("[Info] 退出标注。")
                    cv2.destroyAllWindows()
                    return
                elif key == ord('0'):
                    current_class = 0
                elif key == ord('1'):
                    current_class = 1
                elif key == ord('u'):
                    if boxes:
                        boxes.pop()
                elif key == ord('c'):
                    boxes.clear()
                elif key == 32 or key == ord('n'):  # 空格键
                    # 保存到 txt
                    with open(txt_path, "w") as f:
                        for b in boxes:
                            f.write(convert_to_yolo(b, w, h))
                    print(f"[Saved] {img_name} -> 包含 {len(boxes)} 个目标")
                    break

    cv2.destroyAllWindows()
    print("[Info] 所有目录内的图片已处理完毕，可以开始训练了！")

if __name__ == "__main__":
    main()