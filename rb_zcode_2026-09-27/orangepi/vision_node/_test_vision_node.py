# 香橙派/vision_node/_test_vision_node.py  视觉节点端到端自测（独立运行，不依赖树莓派）
# 用法: cd vision_node && python3 _test_vision_node.py
#   测试 1: 真实 YOLO 推理（best.pt + 标定样张，验证模型与权重在位）
#   测试 2: 服务端回环（后台起真实 server，用裸 socket 按协议收发）
# 注：树莓派侧客户端与真实服务端的联调验证见 树莓派/robogame_project/_test_vision_stub.py
#     （桩）与部署手册（真机集成）。
import json
import os
import socket
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

FAILURES = 0


def check(name, ok, detail=""):
    global FAILURES
    mark = "✓" if ok else "✗"
    if not ok:
        FAILURES += 1
    print(f"  {mark} {name} {detail}")


def recv_line(sock, timeout_s=15.0):
    """按行收应答（含 EOF 检查），返回 dict；失败抛 OSError/ValueError。"""
    sock.settimeout(timeout_s)
    buf = b""
    while b"\n" not in buf:
        chunk = sock.recv(65536)
        if not chunk:
            raise ConnectionError("服务端关闭了连接")
        buf += chunk
        if len(buf) > 256 * 1024:
            raise ValueError("应答超长")
    return json.loads(buf.split(b"\n", 1)[0].decode("utf-8"))


def test_detector():
    print("测试 1: YOLO 检测器（样张推理）")
    try:
        from vision.yolo_detector import YoloDetector
        det = YoloDetector(os.path.join(HERE, "best.pt"), imgsz=416)
        det.warmup()
        t0 = time.perf_counter()
        dets = det.detect(os.path.join(HERE, "target原始图.jpg"), conf=0.5)
        dt = (time.perf_counter() - t0) * 1000
        check("样张推理", len(dets) > 0, f"检出 {len(dets)} 目标, {dt:.0f}ms")
        purple = [d for d in dets if d["name"] == "Purple_Block"]
        check("含 Purple_Block", len(purple) > 0,
              f"中心=({purple[0]['cx']:.1f},{purple[0]['cy']:.1f})" if purple else "")
    except Exception as e:
        check("YOLO 检测器", False, f"{type(e).__name__}: {e}")


def test_server_loopback():
    print("测试 2: 服务端回环（真实 server + 裸 socket 协议收发）")
    proc = subprocess.Popen(
        [sys.executable, os.path.join(HERE, "vision_server.py")],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        cwd=HERE, env={**os.environ, "PYTHONIOENCODING": "utf-8"},
    )
    sock = None
    try:
        # 等端口就绪（模型 warmup 在 listen 之后，端口应很快可连）
        deadline = time.time() + 60
        ready = False
        while time.time() < deadline:
            if proc.poll() is not None:
                out = proc.stdout.read().decode("utf-8", errors="replace")
                check("server 启动", False, f"进程退出: {out[-300:]}")
                return
            try:
                s = socket.create_connection(("127.0.0.1", 9000), timeout=0.5)
                ready = True
                sock = s
                break
            except OSError:
                time.sleep(0.5)
        check("server 端口就绪", ready)
        if not ready:
            return

        sock.sendall(b'{"cmd": "ping"}\n')
        resp = recv_line(sock)
        check("ping", resp.get("ok") is True)

        sock.sendall(b'{"cmd": "detect", "conf": 0.5}\n')
        resp = recv_line(sock, timeout_s=30.0)
        check("detect", resp.get("ok") is True,
              f"{len(resp.get('dets', []))} 目标, {resp.get('infer_ms')}ms"
              if resp.get("ok") else f"error={resp.get('error')}（无摄像头环境允许）")

        sock.sendall(b'{"cmd": "status"}\n')
        resp = recv_line(sock)
        check("status", resp.get("ok") is True,
              f"model_ready={resp.get('model_ready')}, frame={resp.get('frame')}")

        # 协议健壮性：非对象 JSON / 非法 JSON / 未知指令 —— 服务端必须活着应答
        sock.sendall(b'123\n')
        resp = recv_line(sock)
        check("非对象JSON不崩", resp.get("ok") is False, f"error={resp.get('error')}")
        sock.sendall(b'not-json\n')
        resp = recv_line(sock)
        check("非法JSON不崩", resp.get("ok") is False)
        sock.sendall(b'{"cmd": "nope"}\n')
        resp = recv_line(sock)
        check("未知指令不崩", resp.get("ok") is False)
        sock.sendall(b'{"cmd": "ping"}\n')
        resp = recv_line(sock)
        check("崩溃测试后服务仍正常", resp.get("ok") is True)
    finally:
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()


def main():
    print("=" * 64)
    test_detector()
    print("=" * 64)
    test_server_loopback()
    print("=" * 64)
    print(f"结果: 失败 {FAILURES} 项" + ("（全部通过）" if FAILURES == 0 else ""))
    sys.exit(1 if FAILURES else 0)


if __name__ == "__main__":
    main()
