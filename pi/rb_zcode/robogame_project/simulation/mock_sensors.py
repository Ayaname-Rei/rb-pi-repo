# simulation/mock_sensors.py

class BaseIrSensor:
    """红外传感器抽象接口"""
    def is_centered(self) -> bool:
        raise NotImplementedError
    def reset(self):
        pass

class MockIrSensor(BaseIrSensor):
    """用于本地电脑调试的虚拟红外传感器"""
    def __init__(self, trigger_tick: int = 3):
        self.trigger_tick = trigger_tick
        self.ticks = 0
        print("[Simulation] MockIrSensor 已加载：模拟红外关键点检测")

    def reset(self):
        self.ticks = 0

    def is_centered(self) -> bool:
        self.ticks += 1
        if self.ticks >= self.trigger_tick:
            print(f"[Simulation] Mock IR: 成功捕捉到关键点黑线！ (Tick: {self.ticks})")
            return True
        print(f"[Simulation] Mock IR: 正在寻找黑线... (Tick: {self.ticks})")
        return False

class RealHardwareIrSensor(BaseIrSensor):
    """真实硬件红外传感器驱动（上车真机时使用）"""
    def __init__(self, pin: int = 23):
        self.pin = pin
        print(f"[Hardware] 初始化真机红外传感器 (GPIO Pin: {pin})")
        # 实际开发中可在此处初始化 RPi.GPIO 或 gpiozero

    def is_centered(self) -> bool:
        # TODO: 读取真实硬件 GPIO 电平状态
        # 例如: return GPIO.input(self.pin) == GPIO.LOW
        return True