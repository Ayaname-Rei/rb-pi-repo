# main.py
"""工程唯一入口：启动 Tkinter 任务控制界面（参照 robo_control.py 交互风格）。"""
import tkinter as tk

from ui.mission_gui import MissionControlApp


def main():
    root = tk.Tk()
    MissionControlApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()





