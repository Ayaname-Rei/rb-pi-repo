# 树莓派/robogame_project/vision_client/remote_vision.py
"""远程视觉服务客户端：经 TCP/JSON 连接香橙派视觉节点。

职责划分：
- 树莓派（本机）：行为树、底盘/机械臂串口、任务决策 —— 不加载 torch/cv2，
  内存占用从 ~700MB 降到 ~100MB 级；
- 香橙派（对端，vision_node/vision_server.py）：相机采帧 + YOLO 推理，
  对外只暴露 detect/ping/status 三个指令。

接口说明：
- detect_async()/poll_detect()：异步检测（推荐，行为树用）——请求在后台
  线程执行，GUI 主线程的 20ms 轮询不被阻塞，急停按钮始终响应；
- detect()/ping()：同步版本（GUI「测试视觉链路」按钮用），若异步请求在途
  会拒绝执行（避免两个线程写同一个 socket）。

链路鲁棒性：
- 请求超时与连接超时分离（推理本身在香橙派上要 0.3~1s，请求超时须放宽）；
- 每次请求遇到连接类错误先尝试一次重连再重发，仍失败返回 None
  （上层 VisualPickAction 的重试机制会继续兜底）；
- 重连后是新 socket（四元组隔离），旧请求的迟到应答不会错位成新应答；
- RTT/最近错误对外可查，供 GUI 显示链路健康。
"""
import json
import socket
import threading
import time

# 单帧 JSON 上限（几十个目标的检测框远小于此）
_MAX_LINE_BYTES = 256 * 1024


class RemoteVisionService:
    """视觉服务客户端：detect → 目标列表；ping → RTT。"""

    def __init__(self, host: str, port: int,
                 connect_timeout_s: float = 3.0, request_timeout_s: float = 6.0):
        self.host = host
        self.port = port
        self.connect_timeout_s = connect_timeout_s
        self.request_timeout_s = request_timeout_s
        self.sock = None
        self.connected = False
        self.last_error = None
        self.last_rtt_ms = None    # 最近一次成功请求的往返耗时（ms）
        # 异步检测（行为树用）：后台线程 + 结果占位
        self._async_lock = threading.Lock()
        self._pending = None       # {"done": bool, "result": list|None} 或 None
        self._worker = None

    # ---------------- 连接管理 ----------------
    def connect(self) -> bool:
        """建立 TCP 连接（不阻塞任务：失败仅置状态，上层按需重试）。"""
        self.close()
        try:
            sock = socket.create_connection(
                (self.host, self.port), timeout=self.connect_timeout_s)
            sock.settimeout(self.request_timeout_s)
            # TCP_NODELAY：请求-应答小包场景关 Nagle，降交互延迟
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.sock = sock
            self.connected = True
            self.last_error = None
            return True
        except OSError as e:
            self.connected = False
            self.last_error = f"连接视觉节点 {self.host}:{self.port} 失败: {e}"
            return False

    def close(self):
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
        self.sock = None
        self.connected = False

    # ---------------- 请求 ----------------
    def _send_request(self, payload: dict):
        """一次「发送+收应答」。返回 dict 或抛连接/格式异常。"""
        if self.sock is None:
            raise ConnectionError("socket 未连接")
        line = (json.dumps(payload, ensure_ascii=False) + "\n").encode("utf-8")
        self.sock.sendall(line)
        buf = b""
        while len(buf) < _MAX_LINE_BYTES:
            chunk = self.sock.recv(65536)
            if not chunk:
                raise ConnectionError("视觉节点关闭了连接")
            buf += chunk
            if b"\n" in buf:
                break
        else:
            raise ConnectionError("应答超长（>256KB），链路异常")
        first_line = buf.split(b"\n", 1)[0]
        return json.loads(first_line.decode("utf-8"))

    def _request(self, payload: dict):
        """带「断线重连一次」的请求。返回 dict 或 None。

        注：重连是新 socket，旧请求的迟到应答不会与本请求应答混淆；
        代价仅是慢请求（>request_timeout_s）在服务端被多推理一次。
        """
        for attempt in (1, 2):
            if not self.connected and not self.connect():
                if attempt == 2:
                    return None
                continue
            t0 = time.perf_counter()
            try:
                resp = self._send_request(payload)
                self.last_rtt_ms = (time.perf_counter() - t0) * 1000.0
                return resp
            except (OSError, ConnectionError, json.JSONDecodeError,
                    UnicodeDecodeError, ValueError) as e:
                self.last_error = f"{type(e).__name__}: {e}"
                self.close()
                if attempt == 1:
                    time.sleep(0.05)
        return None

    def _async_inflight(self) -> bool:
        w = self._worker
        return w is not None and w.is_alive()

    # ---------------- 异步检测（行为树主路径） ----------------
    def detect_async(self, conf: float) -> bool:
        """后台线程发起 detect，立即返回 True；已有在途请求则返回 False。

        结果用 poll_detect() 收取。GUI 20ms 轮询/急停按钮不被阻塞。
        """
        with self._async_lock:
            if self._async_inflight():
                return False
            holder = {"done": False, "result": None}
            self._pending = holder

            def _work():
                try:
                    holder["result"] = self._detect_sync(conf)
                except Exception as e:      # 后台线程绝不向上抛
                    holder["result"] = None
                    self.last_error = f"{type(e).__name__}: {e}"
                finally:
                    holder["done"] = True

            self._worker = threading.Thread(target=_work, daemon=True)
            self._worker.start()
            return True

    def poll_detect(self):
        """收取异步检测结果。返回 (状态, 结果)：
        - ("idle", None)：无在途请求（调用方应 detect_async 发起一次）
        - ("busy", None)：请求在途（调用方下个 tick 再来）
        - ("done", dets)：完成；dets 为目标列表，None 表示失败（见 last_error）
        """
        p = self._pending
        if p is None:
            return ("idle", None)
        if not p["done"]:
            return ("busy", None)
        self._pending = None
        return ("done", p["result"])

    def cancel_async(self):
        """丢弃未收取的异步结果（行为树节点重置时调用，防串场）。"""
        with self._async_lock:
            self._pending = None

    # ---------------- 同步接口（GUI 测试按钮用） ----------------
    def _detect_sync(self, conf: float):
        resp = self._request({"cmd": "detect", "conf": conf})
        if resp is None or not resp.get("ok"):
            if resp is not None:
                self.last_error = resp.get("error", "视觉节点返回失败")
            return None
        return resp.get("dets", [])

    def detect(self, conf: float):
        """同步检测：返回目标列表；失败返回 None。异步在途时拒绝（返回 None）。"""
        if self._async_inflight():
            self.last_error = "异步检测进行中，请勿并发调用"
            return None
        return self._detect_sync(conf)

    def ping(self):
        """链路测试，返回 RTT 秒（恒 >0）；失败返回 None。异步在途时拒绝。"""
        if self._async_inflight():
            self.last_error = "异步检测进行中，请勿并发调用"
            return None
        # 用 perf_counter 计时：Windows 上 monotonic 粒度约 15.6ms，localhost
        # 往返不足一个 tick 会算出精确的 0.0，易被调用方误判为失败
        t0 = time.perf_counter()
        resp = self._request({"cmd": "ping"})
        if resp is None or not resp.get("ok"):
            return None
        return max(time.perf_counter() - t0, 1e-6)
