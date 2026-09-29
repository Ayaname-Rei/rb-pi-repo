# hardware/ir_sensor.py
"""红外巡线传感器驱动：统一接口 + 真机占位实现

业务行为树只依赖 BaseIrSensor 接口，不关心底层是真机 GPIO 还是本地仿真，
因此切换真机/仿真时上层代码无需任何改动。
"""

class BaseIrSensor:
    """红外传感器统一接口"""
    def is_centered(self) -> bool:
        raise NotImplementedError


class RealHardwareIrSensor(BaseIrSensor):
    """真机红外巡线传感器驱动（占位实现）

    TODO(上车部署): 根据实际 GPIO 接线补全 is_centered()，
    返回底盘是否对准关键点黑线。业务代码无需改动。
    """
    def __init__(self, pin: int = 23):
        self.pin = pin
        print(f"[Info] 已加载 RealHardwareIrSensor 占位驱动 (pin={pin})，待实现真机读取逻辑")

    def is_centered(self) -> bool:
        # TODO: 读取 GPIO 电平并判断是否居中；当前占位恒返回 False
        return False
