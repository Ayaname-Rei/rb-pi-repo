"""
RoboGame LeArm 智能机械臂 —— 阶段四：动作组数据模型与 XML 序列化器 (ActionGroupManager)
作者: Antigravity

本模块掌管上位机第三模块“真实出厂生产层”的动作帧列表数据结构与 XML 持久化存取：
1. ActionFrame: 包含空间坐标、6 轴脉宽、运行时间、插补模式与爪子开合状态的独立动作帧。
2. ActionGroupManager: 提供动作帧的高效内存管理（增、删、改、插入、上移、下移、重新编号），
   以及符合 V3.0 规范的 XML 格式双向导入导出（同时自适应反解 FK 坐标，向前向后完全兼容）。
"""

import os
import sys
import re
import xml.etree.ElementTree as ET
from xml.dom import minidom

# 保证在 Windows 控制台下支持 UTF-8 打印
if sys.platform.startswith("win"):
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:
        pass

# 导入基础运动学
try:
    from arm_kinematics import ArmKinematics
except ImportError:
    cur_dir = os.path.dirname(os.path.abspath(__file__))
    sys.path.append(cur_dir)
    from arm_kinematics import ArmKinematics


class ActionFrame:
    """标准化动作帧模型"""

    def __init__(self, index=1, name="", time_ms=1000, duties=None, 
                 x=None, y=None, z=None, pitch=None, roll=None, interp="LINEAR"):
        """
        :param index: 帧号 (1-based)
        :param name: 帧名称描述 (如 "接近紫色槽")
        :param time_ms: 该帧到达运行耗时 (ms)
        :param duties: 6 轴舵机目标脉宽字典 {1: d1, 2: d2, ..., 6: d6}
        :param x, y, z: 笛卡尔坐标 (cm)，可选 (若为 None 可在需要时通过 FK 反解)
        :param pitch: 夹爪空间俯仰角 (度)
        :param roll: 手腕翻滚角 (度)，若为 None 自动根据 2 号舵机脉宽推导
        :param interp: 插补模式: "LINEAR" (笛卡尔空间直线插补) 或 "JOINT" (关节空间插补)
        """
        self.index = int(index)
        self.name = str(name) if name else f"帧 {self.index}"
        self.time_ms = max(20, int(time_ms))
        
        # 默认中位姿态脉宽保底
        default_duties = {1: 281, 2: 509, 3: 500, 4: 500, 5: 500, 6: 496}
        if duties:
            self.duties = {int(k): int(v) for k, v in duties.items()}
            # 补齐未指定的轴
            for sid in range(1, 7):
                if sid not in self.duties:
                    self.duties[sid] = default_duties[sid]
        else:
            self.duties = default_duties

        self.x = float(x) if x is not None else None
        self.y = float(y) if y is not None else None
        self.z = float(z) if z is not None else None
        self.pitch = float(pitch) if pitch is not None else None

        # 翻滚角 (Roll)：若未显式指定，自动由 2 号舵机脉宽精准反推
        if roll is not None:
            self.roll = float(roll)
        else:
            d2 = self.duties.get(2, 509)
            k_angle = 1000.0 / 240.0
            self.roll = round((d2 - 509) / k_angle, 1)

        self.interp = "JOINT" if str(interp).upper() == "JOINT" else "LINEAR"

    def get_claw_state(self):
        """获取夹爪文本状态"""
        claw_duty = self.duties.get(1, 281)
        if claw_duty <= 300:
            return "OPEN"
        elif claw_duty >= 400:
            return "GRIP"
        return f"DUTY_{claw_duty}"

    def to_dict(self):
        """转换为字典"""
        return {
            "index": self.index,
            "name": self.name,
            "time_ms": self.time_ms,
            "duties": dict(self.duties),
            "x": self.x,
            "y": self.y,
            "z": self.z,
            "pitch": self.pitch,
            "roll": self.roll,
            "interp": self.interp,
            "claw_state": self.get_claw_state()
        }

    @classmethod
    def from_dict(cls, d):
        """从字典还原 ActionFrame"""
        return cls(
            index=d.get("index", 1),
            name=d.get("name", ""),
            time_ms=d.get("time_ms", 1000),
            duties=d.get("duties", {}),
            x=d.get("x", None),
            y=d.get("y", None),
            z=d.get("z", None),
            pitch=d.get("pitch", None),
            roll=d.get("roll", 0.0),
            interp=d.get("interp", "LINEAR")
        )

    def clone(self):
        """深拷贝当前帧"""
        return ActionFrame(
            index=self.index,
            name=self.name,
            time_ms=self.time_ms,
            duties=dict(self.duties),
            x=self.x,
            y=self.y,
            z=self.z,
            pitch=self.pitch,
            roll=self.roll,
            interp=self.interp
        )


class ActionGroupManager:
    """真实生产层动作组序列管理器与 XML 序列化器"""

    def __init__(self, name="New_Action_Group", kinematics=None, calib_file=None):
        self.name = name
        self.frames = []
        self.last_loaded_format = "DETAILED"
        if kinematics is not None:
            self.ik = kinematics
        else:
            self.ik = ArmKinematics(calib_file=calib_file)

    def __len__(self):
        return len(self.frames)

    def __getitem__(self, idx):
        return self.frames[idx]

    def clear(self):
        """清空当前动作组"""
        self.frames.clear()

    def reindex(self, update_default_names=True):
        """重新整理帧号，确保从 1 到 N 严格连续，并智能顺延默认命名的序号"""
        pattern = re.compile(r"^(动作|帧|Frame|等待位|Step|Task)([_#\s\-]+)(\d+)(.*)$", re.IGNORECASE)
        for i, frame in enumerate(self.frames):
            frame.index = i + 1
            if update_default_names and frame.name:
                m = pattern.match(frame.name.strip())
                if m:
                    prefix = m.group(1)
                    sep = m.group(2)
                    suffix = m.group(4)
                    frame.name = f"{prefix}{sep}{i + 1}{suffix}"

    def add_frame(self, frame: ActionFrame) -> int:
        """
        在动作组末尾追加一个动作帧
        :return: 插入后该帧在列表中的下标 (0-based)
        """
        new_frame = frame.clone()
        new_frame.index = len(self.frames) + 1
        # 若缺失笛卡尔坐标，自动通过正运动学 (FK) 补充反算
        self._ensure_cartesian(new_frame)
        self.frames.append(new_frame)
        self.reindex()
        return len(self.frames) - 1

    def insert_frame(self, index: int, frame: ActionFrame) -> bool:
        """在指定下标位置插入帧"""
        if index < 0 or index > len(self.frames):
            return False
        new_frame = frame.clone()
        self._ensure_cartesian(new_frame)
        self.frames.insert(index, new_frame)
        self.reindex()
        return True

    def update_frame(self, index: int, frame: ActionFrame) -> bool:
        """
        更新指定下标 (0-based) 的动作帧数据
        """
        if index < 0 or index >= len(self.frames):
            return False
        updated = frame.clone()
        updated.index = index + 1
        self._ensure_cartesian(updated)
        self.frames[index] = updated
        return True

    def delete_frame(self, index: int) -> bool:
        """
        删除指定下标 (0-based) 的动作帧
        """
        if index < 0 or index >= len(self.frames):
            return False
        self.frames.pop(index)
        self.reindex()
        return True

    def move_up(self, index: int) -> bool:
        """
        将指定下标 (0-based) 的动作帧上移一位
        """
        if index <= 0 or index >= len(self.frames):
            return False
        self.frames[index - 1], self.frames[index] = self.frames[index], self.frames[index - 1]
        self.reindex()
        return True

    def move_down(self, index: int) -> bool:
        """
        将指定下标 (0-based) 的动作帧下移一位
        """
        if index < 0 or index >= len(self.frames) - 1:
            return False
        self.frames[index], self.frames[index + 1] = self.frames[index + 1], self.frames[index]
        self.reindex()
        return True

    def _ensure_cartesian(self, frame: ActionFrame):
        """如果帧缺少坐标数据，使用运动学正解自动推导补齐"""
        if frame.x is None or frame.y is None or frame.z is None or frame.pitch is None:
            try:
                x, y, z, pitch, _ = self.ik.solve_fk(frame.duties)
                frame.x = round(x, 2)
                frame.y = round(y, 2)
                frame.z = round(z, 2)
                frame.pitch = round(pitch, 1)
            except Exception:
                pass
        # 始终同步手腕翻滚角 (Roll)
        d2 = frame.duties.get(2, self.ik.calib["wrist_roll_2"]["zero"])
        frame.roll = round((d2 - self.ik.calib["wrist_roll_2"]["zero"]) / self.ik.ANGLE_TO_PULSE, 1)

    def save_to_xml(self, filepath: str) -> bool:
        """
        将动作组保存为符合规范的标准 XML 文件
        """
        try:
            self.reindex()
            root = ET.Element("ActionGroup")
            root.set("name", self.name)
            root.set("total_frames", str(len(self.frames)))

            for frame in self.frames:
                self._ensure_cartesian(frame)
                frame_elem = ET.SubElement(root, "Frame")
                frame_elem.set("index", str(frame.index))
                frame_elem.set("name", frame.name)
                frame_elem.set("time", str(frame.time_ms))
                frame_elem.set("interp", frame.interp)

                # Cartesian 空间标签
                cart = ET.SubElement(frame_elem, "Cartesian")
                cart.set("X", f"{frame.x:.2f}" if frame.x is not None else "0.00")
                cart.set("Y", f"{frame.y:.2f}" if frame.y is not None else "0.00")
                cart.set("Z", f"{frame.z:.2f}" if frame.z is not None else "0.00")
                cart.set("Pitch", f"{frame.pitch:.1f}" if frame.pitch is not None else "0.0")
                cart.set("Roll", f"{frame.roll:.1f}" if frame.roll is not None else "0.0")

                # Servos 脉宽标签
                servos = ET.SubElement(frame_elem, "Servos")
                for sid in range(1, 7):
                    servos.set(f"S{sid}", str(frame.duties.get(sid, 500)))

                # Claw 爪子标签
                claw = ET.SubElement(frame_elem, "Claw")
                claw.set("state", frame.get_claw_state())

            # 格式化美化输出
            xml_str = ET.tostring(root, encoding="utf-8")
            dom = minidom.parseString(xml_str)
            pretty_xml = dom.toprettyxml(indent="  ", encoding="utf-8")

            # 确保保存目录存在
            os.makedirs(os.path.dirname(os.path.abspath(filepath)), exist_ok=True)
            with open(filepath, "wb") as f:
                f.write(pretty_xml)
            return True
        except Exception as e:
            print(f"[错误] 保存 XML 动作组失败: {e}")
            return False

    def load_from_xml(self, filepath: str, append: bool = False, insert_index: int = None) -> bool:
        """
        从 XML 文件加载动作组。
        自适应双向兼容支持：
        1. 本上位机详细标准格式 (<ActionGroup> + <Frame> + <Cartesian> + <Servos>)
        2. 商家官方上位机旧版格式 (<NewDataSet> + <Table> + <Move>#1 P... + <Time>T...)
        读取商家旧版格式后，将自动通过运动学正解 (FK) 补齐末端空间坐标 (X, Y, Z, Pitch)，
        并在后续保存时，无缝生成带笛卡尔插补的现代化详细 XML 文件！
        :param append: 若为 True，则保留现有动作帧，将新解析的帧追加/插入；若为 False，则清空现有帧覆盖加载
        :param insert_index: 若指定，则将新动作帧插入到现有序列的该位置 (0-based)；若为 None 且 append=True，则追加到末尾
        :return: bool 是否成功读取
        """
        if not os.path.exists(filepath):
            print(f"[错误] 文件不存在: {filepath}")
            return False

        try:
            # 容错解析 XML（支持 utf-8、gbk、gb2312 等各种编码的商家旧文件）
            root = None
            try:
                tree = ET.parse(filepath)
                root = tree.getroot()
            except Exception:
                for enc in ["utf-8", "gbk", "gb2312", "ansi"]:
                    try:
                        with open(filepath, "r", encoding=enc, errors="ignore") as f:
                            content = f.read()
                        root = ET.fromstring(content)
                        break
                    except Exception:
                        continue

            if root is None:
                print(f"[错误] XML 文件解析失败，无法解析根节点: {filepath}")
                return False

            parsed_frames = []

            # -------------------------------------------------------------
            # 分支 1: 本软件高级详细格式 (<Frame> 节点)
            # -------------------------------------------------------------
            frame_nodes = root.findall(".//Frame")
            if frame_nodes:
                self.last_loaded_format = "DETAILED"
                for elem in frame_nodes:
                    idx = int(elem.get("index", len(parsed_frames) + 1))
                    name = elem.get("name", f"动作_{idx}")
                    time_ms = int(elem.get("time", 1000))
                    interp = elem.get("interp", "LINEAR")

                    x, y, z, pitch, roll = None, None, None, None, None
                    cart_elem = elem.find("Cartesian")
                    if cart_elem is not None:
                        x = float(cart_elem.get("X", 0.0))
                        y = float(cart_elem.get("Y", 0.0))
                        z = float(cart_elem.get("Z", 0.0))
                        pitch = float(cart_elem.get("Pitch", 0.0))
                        if "Roll" in cart_elem.attrib:
                            roll = float(cart_elem.get("Roll", 0.0))

                    duties = {}
                    servos_elem = elem.find("Servos")
                    if servos_elem is not None:
                        for sid in range(1, 7):
                            val_str = servos_elem.get(f"S{sid}")
                            if val_str is not None:
                                duties[sid] = int(val_str)

                    frame = ActionFrame(
                        index=idx,
                        name=name,
                        time_ms=time_ms,
                        duties=duties,
                        x=x, y=y, z=z, pitch=pitch, roll=roll,
                        interp=interp
                    )
                    self._ensure_cartesian(frame)
                    parsed_frames.append(frame)

            # -------------------------------------------------------------
            # 分支 2: 商家官方上位机简略格式 (<NewDataSet> / <Move> / <Time>)
            # -------------------------------------------------------------
            else:
                self.last_loaded_format = "VENDOR"
                moves = root.findall(".//Move")
                times = root.findall(".//Time")
                ids = root.findall(".//ID")
                type_node = root.find(".//Type")
                type_text = type_node.text.strip() if type_node is not None and type_node.text else ""
                is_pwm = "PWM" in type_text.upper()

                if not moves:
                    print(f"[警告] XML 文件中未检测到有效动作帧数据 (<Frame> 或 <Move>): {filepath}")
                    return False

                count = len(moves)
                for i in range(count):
                    m_txt = moves[i].text.strip() if moves[i].text else ""
                    t_txt = times[i].text.strip() if i < len(times) and times[i].text else ""
                    id_txt = ids[i].text.strip() if i < len(ids) and ids[i].text else str(i + 1)

                    # 提取各个舵机脉宽: #1 P205 #2 P503 ...
                    pairs = re.findall(r"#(\d+)\s*P(\d+)", m_txt)
                    duties = {}
                    for sid_str, duty_str in pairs:
                        sid = int(sid_str)
                        duty = int(duty_str)
                        # 如果是 PWM 格式 (500~2500) 自动转换为总线脉宽 (0~1000)
                        if is_pwm or duty > 1000:
                            duty = max(0, min(1000, int((duty - 500) / 2)))
                        duties[sid] = duty

                    # 补齐可能缺失的舵机保底值
                    for sid in range(1, 7):
                        if sid not in duties:
                            duties[sid] = 500

                    # 提取运行时间: T500 -> 500
                    t_match = re.search(r"\d+", t_txt)
                    time_ms = int(t_match.group(0)) if t_match else 1000

                    # 提取帧序号
                    try:
                        frame_idx = int(id_txt)
                    except ValueError:
                        frame_idx = i + 1

                    frame = ActionFrame(
                        index=frame_idx,
                        name=f"动作_{frame_idx}",
                        time_ms=time_ms,
                        duties=duties,
                        interp="LINEAR"  # 官方文件导入后，默认赋予笛卡尔直线插补模式
                    )
                    # 关键一步：通过运动学正解 FK 自动反算并补齐 X, Y, Z, Pitch 空间坐标！
                    self._ensure_cartesian(frame)
                    parsed_frames.append(frame)

            if not parsed_frames:
                return False

            self.last_imported_count = len(parsed_frames)

            if not append:
                self.name = root.get("name", os.path.splitext(os.path.basename(filepath))[0])
                self.frames = parsed_frames
            else:
                if insert_index is not None and 0 <= insert_index <= len(self.frames):
                    self.frames[insert_index:insert_index] = parsed_frames
                else:
                    self.frames.extend(parsed_frames)

            self.reindex(update_default_names=True)
            return len(self.frames) > 0
        except Exception as e:
            print(f"[错误] 读取 XML 动作组失败: {e}")
            return False

    def export_to_vendor_xml(self, filepath: str, servo_type: str = "BUS Servo") -> bool:
        """
        将当前动作组导出为商家官方上位机兼容格式 (<NewDataSet>)，
        便于在官方调试软件或旧版工具中直接打开使用。
        """
        try:
            self.reindex()
            root = ET.Element("NewDataSet")
            type_elem = ET.SubElement(root, "Type")
            type_elem.text = servo_type

            table_elem = ET.SubElement(root, "Table")
            for frame in self.frames:
                id_elem = ET.SubElement(table_elem, "ID")
                id_elem.text = str(frame.index)

                move_elem = ET.SubElement(table_elem, "Move")
                parts = []
                for sid in range(1, 7):
                    parts.append(f"#{sid} P{frame.duties.get(sid, 500)}")
                move_elem.text = " ".join(parts)

                time_elem = ET.SubElement(table_elem, "Time")
                time_elem.text = f"T{frame.time_ms}"

            xml_str = ET.tostring(root, encoding="utf-8")
            dom = minidom.parseString(xml_str)
            pretty_xml = dom.toprettyxml(indent="  ", encoding="utf-8")

            os.makedirs(os.path.dirname(os.path.abspath(filepath)), exist_ok=True)
            with open(filepath, "wb") as f:
                f.write(pretty_xml)
            return True
        except Exception as e:
            print(f"[错误] 导出官方格式 XML 失败: {e}")
            return False


if __name__ == "__main__":
    print("=" * 65)
    print("🚀 【ActionGroupManager 动作组与 XML 序列化模型独立验证】")
    print("=" * 65)

    manager = ActionGroupManager(name="PickPurple_And_Store")

    # 模拟加入测试帧序列
    test_frames = [
        ActionFrame(1, "接近高台槽", 1200, {1: 281, 2: 509, 3: 589, 4: 446, 5: 753, 6: 496}, 44.70, 0.00, -3.00, -35.0),
        ActionFrame(2, "下插夹紧方块", 600,  {1: 453, 2: 509, 3: 589, 4: 446, 5: 753, 6: 496}, 44.70, 0.00, -8.78, -35.0),
        ActionFrame(3, "拔脱回程安全位", 800, {1: 453, 2: 509, 3: 589, 4: 446, 5: 680, 6: 496}, 30.00, 0.00,  2.00, -35.0),
        ActionFrame(4, "平直下放车身料框", 1000, {1: 281, 2: 509, 3: 320, 4: 360, 5: 480, 6: 496}, 16.00, 0.00, -12.00, -35.0),
    ]

    for f in test_frames:
        manager.add_frame(f)

    print(f"✅ 成功添加 {len(manager)} 个动作帧")

    # 测试帧操作：上移与下移
    print("👉 测试调序：将第 4 帧上移")
    manager.move_up(3)
    assert manager[2].name == "平直下放车身料框"
    print("👉 测试调序：再将第 3 帧下移复原")
    manager.move_down(2)
    assert manager[3].name == "平直下放车身料框"
    print("✅ 帧顺序调配逻辑自检完全正确")

    # 测试 XML 保存与读取
    test_xml_path = os.path.join(os.path.dirname(__file__), "test_action_group.xml")
    saved = manager.save_to_xml(test_xml_path)
    print(f"👉 动作组导出 XML: {saved}, 文件: {test_xml_path}")

    # 读取验证
    new_manager = ActionGroupManager()
    loaded = new_manager.load_from_xml(test_xml_path)
    print(f"👉 动作组从 XML 读回: {loaded}, 读回帧数: {len(new_manager)}")

    for f in new_manager:
        print(f"   [帧 {f.index}] {f.name:<12} | 时长: {f.time_ms:4d}ms | 坐标: ({f.x:5.1f}, {f.y:5.1f}, {f.z:6.2f}, P={f.pitch:4.1f}°) | 夹爪: {f.get_claw_state()}")

    # 清理临时测试 XML
    if os.path.exists(test_xml_path):
        os.remove(test_xml_path)
        print("🧹 临时测试 XML 文件已清理")

    print("\n✅ ActionGroupManager 所有功能验证完毕！")
