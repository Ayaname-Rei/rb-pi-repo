# 香橙派/vision_node/vision_server.py
"""香橙派视觉节点服务端：相机 + YOLO 推理，TCP/JSON 行协议。

树莓派卸掉 torch 后（4GB 内存方案），视觉整体迁到本机：
  树莓派（客户端）── detect/ping 请求 ──▶ 本服务（逐请求处理）
应答里的检测框坐标是原图 1280×720 像素系，与树莓派 VisionConfig.target_u/v
标定坐标系一致，两侧无需任何坐标换算。

协议（JSON + 换行分帧，UTF-8，详见 docs/通信协议_树莓派-香橙派.md）：
  请求: {"cmd": "ping"}                          → {"ok": true, "pong": true, ...}
        {"cmd": "detect", "conf": 0.5}           → {"ok": true, "dets": [...], "infer_ms": ..}
  应答统一带 ok；detect 失败（相机/模型异常）时 {"ok": false, "error": "..."}。
  连接断开不退出进程：关闭后循环收下一个连接（树莓派随时可重连）。

单连接顺序处理：视觉伺服同一时刻只有一个请求在途，串行天然够用，
也避免多客户端并发抢相机。
"""
import json
import os
import socket
import sys
import threading
import time
import traceback

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import server_config as cfg

_MAX_LINE_BYTES = 256 * 1024


class VisionNode:
    """相机 + YOLO 检测器封装：惰性初始化 + 采帧失败计数熔断。"""

    def __init__(self):
        self.detector = None
        self.camera = None
        self.camera_fail_streak = 0   # 连续采帧失败次数（≥5 判定相机坏，重建）
        self.model_fail_streak = 0
        self.lock = threading.Lock()  # 防并发 detect（顺序服务下基本不竞争）

    # ---------------- 惰性初始化 ----------------
    def _ensure_camera(self):
        if self.camera is not None:
            return True
        from vision.camera import CvCamera
        try:
            self.camera = CvCamera(index=cfg.CAMERA_INDEX,
                                   width=cfg.FRAME_WIDTH, height=cfg.FRAME_HEIGHT)
            print(f"[Info] 相机已打开 index={cfg.CAMERA_INDEX} "
                  f"({cfg.FRAME_WIDTH}x{cfg.FRAME_HEIGHT})")
            return True
        except Exception as e:
            print(f"[Error] 相机打开失败: {e}")
            self.camera = None
            return False

    def _ensure_detector(self):
        if self.detector is not None:
            return True
        from vision.yolo_detector import YoloDetector
        print(f"[Info] 加载 YOLO 模型 {cfg.WEIGHTS_PATH} (imgsz={cfg.IMGSZ}) ...")
        t0 = time.monotonic()
        try:
            self.detector = YoloDetector(cfg.WEIGHTS_PATH, imgsz=cfg.IMGSZ)
            if cfg.WARMUP_ON_START:
                self.detector.warmup()
            print(f"[Info] 模型就绪，耗时 {time.monotonic() - t0:.1f}s（含预热）")
            return True
        except Exception as e:
            traceback.print_exc()
            print("[Error] 模型加载失败（检查 ultralytics/torch 是否装好、best.pt 是否在位）")
            self.detector = None
            return False

    # ---------------- 指令处理 ----------------
    def handle_detect(self, conf: float):
        if not self._ensure_camera():
            return {"ok": False, "error": "相机不可用"}
        if not self._ensure_detector():
            return {"ok": False, "error": "模型未就绪"}

        with self.lock:
            t0 = time.monotonic()
            try:
                frame = self.camera.capture()
            except Exception as e:
                frame = None
                print(f"[Error] 相机采帧异常: {e}")
            if frame is None:
                self.camera_fail_streak += 1
                if self.camera_fail_streak >= 5:
                    print("[Error] 连续 5 次采帧失败，重建相机（可能被拔出/USB 掉线）")
                    try:
                        self.camera.release()
                    except Exception:
                        pass
                    self.camera = None
                return {"ok": False, "error": "采帧失败"}
            self.camera_fail_streak = 0

            try:
                dets = self.detector.detect(frame, conf=conf)
            except Exception as e:
                traceback.print_exc()
                self.model_fail_streak += 1
                if self.model_fail_streak >= 3:   # 推理连续崩 → 重建模型
                    print("[Error] 连续 3 次推理异常，重建检测器")
                    self.detector = None
                return {"ok": False, "error": f"推理异常: {e}"}
            self.model_fail_streak = 0   # 成功即清零，偶发异常不跨请求累计
            infer_ms = (time.monotonic() - t0) * 1000.0

        if cfg.SAVE_LAST_FRAME_DIR:
            try:
                import cv2
                cv2.imwrite(os.path.join(cfg.SAVE_LAST_FRAME_DIR, "last_frame.jpg"), frame)
            except Exception:
                pass
        self._detect_count = getattr(self, "_detect_count", 0) + 1
        if self._detect_count % cfg.LOG_EVERY_N_DETECT == 0:
            print(f"[Info] detect #{self._detect_count}: {len(dets)} 目标, {infer_ms:.0f}ms")
        return {"ok": True, "dets": dets, "infer_ms": round(infer_ms, 1)}

    def handle_status(self):
        return {
            "ok": True,
            "camera_ready": self.camera is not None,
            "model_ready": self.detector is not None,
            "imgsz": cfg.IMGSZ,
            "frame": f"{cfg.FRAME_WIDTH}x{cfg.FRAME_HEIGHT}",
            "uptime_s": round(time.monotonic() - _START_T, 1),
        }


_START_T = time.monotonic()


def serve_one_client(conn: socket.socket, node: VisionNode):
    """处理一个客户端连接：逐请求应答，直到对端断开。"""
    try:
        addr = conn.getpeername()
    except OSError:
        addr = "unknown"
    print(f"[Info] 客户端接入: {addr}")
    # 空闲 recv 上限：任务前半段纯轨迹可能 >60s 不发视觉请求，
    # 不能踢掉连接（否则树莓派侧要白白重连一次）；120s 足够宽裕
    conn.settimeout(120.0)
    buf = b""
    try:
        while True:
            # 按行取请求
            while b"\n" not in buf:
                chunk = conn.recv(65536)
                if not chunk:
                    print(f"[Info] 客户端断开: {addr}")
                    return
                buf += chunk
                if len(buf) > _MAX_LINE_BYTES:
                    print(f"[Error] 请求超长 (>256KB)，断开 {addr}")
                    return
            line, buf = buf.split(b"\n", 1)
            line = line.strip()
            if not line:
                continue
            # 防御（P0-1）：合法 JSON 但不是对象（如一行 "1"/"null"）不能让进程崩
            try:
                req = json.loads(line.decode("utf-8"))
                if not isinstance(req, dict):
                    raise ValueError("请求必须是 JSON 对象")
            except (json.JSONDecodeError, UnicodeDecodeError, ValueError) as e:
                resp = {"ok": False, "error": f"请求不合法: {e}"}
                conn.sendall((json.dumps(resp) + "\n").encode("utf-8"))
                continue

            cmd = req.get("cmd")
            if cmd == "ping":
                resp = {"ok": True, "pong": True, "t": time.time()}
            elif cmd == "status":
                resp = node.handle_status()
            elif cmd == "detect":
                conf = req.get("conf", 0.5)
                try:
                    conf = max(0.01, min(float(conf), 0.99))
                except (TypeError, ValueError):
                    conf = 0.5
                resp = node.handle_detect(conf)
            else:
                resp = {"ok": False, "error": f"未知指令: {cmd}"}

            conn.sendall((json.dumps(resp, ensure_ascii=False) + "\n").encode("utf-8"))
    except (ConnectionResetError, BrokenPipeError, socket.timeout, OSError):
        print(f"[Info] 客户端链路结束: {addr}")
    finally:
        try:
            conn.close()
        except OSError:
            pass


def main():
    # 后台/重定向运行时 print 默认块缓冲，日志会「迟到」；改行缓冲保证实时可见
    try:
        sys.stdout.reconfigure(line_buffering=True)
    except Exception:
        pass
    node = VisionNode()

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((cfg.HOST, cfg.PORT))
    srv.listen(cfg.MAX_CLIENTS)
    print(f"[Info] 视觉节点监听 {cfg.HOST}:{cfg.PORT} "
          f"(相机 {'就绪' if node.camera else '未就绪'}, 模型 {'就绪' if node.detector else '未就绪'})")
    if cfg.WARMUP_ON_START and not node.detector:
        # 先 listen 再 warmup（P2-3）：模型加载 10~30s 期间端口已可连接，
        # 请求在 backlog 排队，树莓派侧不会误判「服务挂了」
        node._ensure_camera()
        node._ensure_detector()
    try:
        while True:
            conn, _addr = srv.accept()
            serve_one_client(conn, node)   # 顺序处理，一次一个客户端
    except KeyboardInterrupt:
        print("\n[Info] 手动退出")
    finally:
        srv.close()


if __name__ == "__main__":
    main()
