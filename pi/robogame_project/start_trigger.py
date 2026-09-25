# start_trigger.py
"""SSH 终端发车触发模块：纯命令行敲击回车发车，专为 SSH + tmux 比赛托管流程设计。

发车流程：
1. 操作手通过 SSH 连接树莓派并进入 tmux 会话（防止网络中断影响比赛）：
   bash launch_car.sh
2. 机器人完成底盘、机械臂与视觉初始化，打印就绪信息；
3. 操作手在赛道发车区将小车摆正就位；
4. 在 SSH 终端敲击【回车键 (Enter)】，小车立即开始全自主比赛任务；
5. 操作手可随时按下 Ctrl+B 然后按 D 脱离 tmux 会话，即使断网小车也会安全跑完全程。
"""
import sys
import threading
import time


def wait_for_start(timeout: float | None = None, poll_callback=None) -> bool:
    """等待 SSH 终端回车触发信号。

    :param timeout: 超时等待时间（秒，None 为无限等待用户敲回车）
    :param poll_callback: 等待期间周期性调用的轮询回调（如 chassis.poll），防止串口缓冲阻塞
    :return: True 表示收到发车信号，False 表示等待超时
    """
    trigger_event = threading.Event()

    def _listen_stdin():
        try:
            if sys.stdin and not sys.stdin.closed:
                line = sys.stdin.readline()
                trigger_event.set()
        except Exception:
            pass

    t = threading.Thread(target=_listen_stdin, daemon=True)
    t.start()

    print("=" * 60)
    print(" >>> 机器人全系统初始化完成，已进入待命状态！<<<")
    print("   请在赛道发车区摆正车身就位后：")
    print("   👉 在当前 SSH 终端敲击【回车键 (Enter)】立即发车 👈")
    print("=" * 60, flush=True)

    try:
        if poll_callback is None:
            triggered = trigger_event.wait(timeout=timeout)
        else:
            t_start = time.monotonic()
            while not trigger_event.is_set():
                if timeout is not None and (time.monotonic() - t_start > timeout):
                    break
                try:
                    poll_callback()
                except Exception:
                    pass
                trigger_event.wait(0.05)
            triggered = trigger_event.is_set()
    except KeyboardInterrupt:
        print("\n[Trigger] 操作手取消发车。")
        return False

    if triggered:
        print("\n[Trigger] >>> 收到发车信号 (Enter)！任务正式启动！<<<\n", flush=True)
    else:
        print("\n[Trigger] 等待发车超时，取消启动。\n", flush=True)

    return triggered


if __name__ == "__main__":
    print("=== 测试 SSH 发车触发模块 ===")
    wait_for_start(timeout=10.0)
