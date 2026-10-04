#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import cv2
import json
import time
import socket
import threading
from ultralytics import YOLO

from config import (
    WEIGHTS, CAMERA_LOW, CAMERA_HIGH, TARGET_U, TARGET_V, 
    TARGET_U_ORANGE, TARGET_V_ORANGE,
    TARGET_U_ORANGE_HIGH, TARGET_V_ORANGE_HIGH,
    AXIS_U, AXIS_V, CONF, IMGSZ, SERVER_HOST, SERVER_PORT
)

def _box_area(d):
    return (d["x2"] - d["x1"]) * (d["y2"] - d["y1"])

def find_purple(dets):
    purple = [d for d in dets if "purple" in d["name"].lower()]
    if not purple:
        return None
    return max(purple, key=lambda d: d["cx"])

def find_orange(dets, min_area=15000):
    orange = [d for d in dets if "orange" in d["name"].lower()]
    if min_area > 0:
        orange = [d for d in orange if _box_area(d) >= min_area]
    if not orange:
        return None
    return max(orange, key=_box_area)

latest_data = {
    "purple": {"found": False, "aligned": False, "eu": 0.0, "ev": 0.0},
    "orange_low": {"found": False, "aligned": False, "eu": 0.0, "ev": 0.0},
    "orange_high": {"found": False, "aligned": False, "eu": 0.0, "ev": 0.0}
}
data_lock = threading.Lock()

target_camera_index = CAMERA_LOW

def handle_client(conn, addr):
    global target_camera_index
    print(f"[通信] 树莓派 {addr} 已连接")
    try:
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        conn.settimeout(0.01) # 非阻塞读取
    except Exception:
        pass
    try:
        while True:
            # 接收指令（非阻塞）
            try:
                data = conn.recv(1024)
                if not data:
                    break
                lines = data.decode('utf-8').strip().split('\n')
                for line in lines:
                    if not line: continue
                    msg = json.loads(line)
                    if msg.get("cmd") == "switch_camera":
                        req_cam = msg.get("camera")
                        new_cam = CAMERA_HIGH if req_cam == "high" else CAMERA_LOW
                        if target_camera_index != new_cam:
                            print(f"[通信] 收到树莓派指令，准备切换摄像头至: {req_cam} ({new_cam})")
                            target_camera_index = new_cam
            except socket.timeout:
                pass
            except Exception as e:
                print(f"[通信] 接收解析异常: {e}")
                break

            with data_lock:
                data_str = json.dumps(latest_data) + "\n"
            conn.sendall(data_str.encode('utf-8'))
            time.sleep(0.03)  # 大约 30Hz 发送频率
    except (ConnectionResetError, BrokenPipeError, socket.error):
        print(f"[通信] 树莓派 {addr} 断开连接")
    finally:
        try:
            conn.close()
        except Exception:
            pass

def server_loop():
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((SERVER_HOST, SERVER_PORT))
    server.listen(5)
    print(f"[通信] TCP 服务端启动成功，监听 {SERVER_HOST}:{SERVER_PORT}，等待树莓派连接...")
    while True:
        try:
            conn, addr = server.accept()
            threading.Thread(target=handle_client, args=(conn, addr), daemon=True).start()
        except Exception as e:
            print(f"[通信] 监听异常: {e}")
            time.sleep(1)

def open_camera(index):
    # 兼容字符串路径与整数（例如 "/dev/video0" -> 0）
    if isinstance(index, str):
        import re
        m = re.search(r'\d+', index)
        dev_idx = int(m.group()) if m else 0
    else:
        dev_idx = int(index)

    # 优先使用 CAP_V4L2 驱动以数字索引打开
    cap = cv2.VideoCapture(dev_idx, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap = cv2.VideoCapture(dev_idx)

    if cap.isOpened():
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        return cap
    return None

def read_latest(cap, max_drop=3):
    for _ in range(max_drop):
        if not cap.grab():
            break
    return cap.retrieve()

def detect(model, frame, conf):
    results = model.predict(source=frame, conf=conf, imgsz=IMGSZ, verbose=False)
    dets = []
    for r in results:
        names = r.names
        for box in r.boxes:
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            dets.append({
                "name": names[int(box.cls[0])],
                "cx": (x1 + x2) / 2.0, "cy": (y1 + y2) / 2.0,
                "x1": x1, "y1": y1, "x2": x2, "y2": y2,
                "conf": float(box.conf[0]),
            })
    return dets

def eval_target(block, w, h, target_u, target_v):
    if block is None:
        return {"found": False, "aligned": False, "eu": 0.0, "ev": 0.0}
    eu = block["cx"] - target_u * w
    ev = block["cy"] - target_v * h
    aligned = (eu / AXIS_U) ** 2 + (ev / AXIS_V) ** 2 <= 1.0
    return {"found": True, "aligned": aligned, "eu": float(eu), "ev": float(ev)}

def process_frame(model, frame):
    global latest_data
    h, w = frame.shape[:2]
    dets = detect(model, frame, CONF)
    
    purple = find_purple(dets)
    orange = find_orange(dets, min_area=15000)
    
    res_purple = eval_target(purple, w, h, TARGET_U, TARGET_V)
    res_orange_low = eval_target(orange, w, h, TARGET_U_ORANGE, TARGET_V_ORANGE)
    res_orange_high = eval_target(orange, w, h, TARGET_U_ORANGE_HIGH, TARGET_V_ORANGE_HIGH)
    
    with data_lock:
        latest_data = {
            "purple": res_purple,
            "orange_low": res_orange_low,
            "orange_high": res_orange_high
        }
    
    # 简单绘图展示（以紫块和低位橙色块为中心画框，防止画面太乱只画找到的）
    if purple:
        x1, y1, x2, y2 = int(purple["x1"]), int(purple["y1"]), int(purple["x2"]), int(purple["y2"])
        cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 255), 2)
    if orange:
        x1, y1, x2, y2 = int(orange["x1"]), int(orange["y1"]), int(orange["x2"]), int(orange["y2"])
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 165, 255), 2)
        
    return frame

def main():
    global target_camera_index
    print("[初始化] 启动网络通信线程...")
    threading.Thread(target=server_loop, daemon=True).start()
    
    print("[初始化] 加载 YOLO 模型...")
    model = YOLO(WEIGHTS)
    print("[初始化] YOLO 模型加载完成")
    
    cap = None
    current_camera_index = None
    
    print("[运行] 视觉服务主循环就绪，正在处理画面...")
    while True:
        # 检查是否需要切换摄像头
        if current_camera_index != target_camera_index:
            if cap is not None:
                print(f"[视觉] 正在释放当前摄像头 {current_camera_index}...")
                cap.release()
                cap = None
            print(f"[视觉] 尝试连接新摄像头 {target_camera_index}...")
            cap = open_camera(target_camera_index)
            if cap is not None and cap.isOpened():
                current_camera_index = target_camera_index
                print(f"[视觉] 摄像头 {current_camera_index} 已成功连接！")
            else:
                print(f"[错误] 无法连接摄像头 {target_camera_index}！1秒后重试...")
                time.sleep(1.0)
                continue

        ok, frame = read_latest(cap)
        if ok and frame is not None:
            frame = process_frame(model, frame)
        else:
            print("[警告] 视频帧读取失败，可能摄像头意外断开，正在重新初始化...")
            if cap is not None:
                try:
                    cap.release()
                except Exception:
                    pass
            cap = None
            current_camera_index = None
            time.sleep(0.5)

if __name__ == "__main__":
    main()
