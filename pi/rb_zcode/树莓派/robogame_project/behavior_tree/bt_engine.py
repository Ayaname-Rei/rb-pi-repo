# behavior_tree/bt_engine.py
from enum import Enum
from typing import List

class NodeStatus(Enum):
    """行为树节点执行的三种状态标准"""
    SUCCESS = 1
    FAILURE = 2
    RUNNING = 3

class BTNode:
    """行为树节点抽象基类"""
    def tick(self) -> NodeStatus:
        raise NotImplementedError("每一个行为树节点必须实现 tick() 方法")

class Sequence(BTNode):
    """
    顺序节点 (Sequence)
    - 逻辑：按顺序依次执行子节点。
    - 规则：只要有一个子节点返回 FAILURE 或 RUNNING，整个 Sequence 就返回该状态；
      只有当所有子节点全部 SUCCESS 时，才返回 SUCCESS。
    """
    def __init__(self, children: List[BTNode]):
        self.children = children
        self.current_index = 0

    def tick(self) -> NodeStatus:
        while self.current_index < len(self.children):
            current_child = self.children[self.current_index]
            status = current_child.tick()
            
            if status == NodeStatus.RUNNING:
                return NodeStatus.RUNNING
            elif status == NodeStatus.FAILURE:
                self.current_index = 0  # 失败重置索引
                return NodeStatus.FAILURE
            
            # 如果当前子节点成功，推进到下一个子节点
            self.current_index += 1
            
        # 所有子节点执行完毕，重置索引并返回成功
        self.current_index = 0
        return NodeStatus.SUCCESS

class Selector(BTNode):
    """
    选择节点 (Selector / Fallback)
    - 逻辑：优选尝试执行子节点。
    - 规则：只要有一个子节点返回 SUCCESS 或 RUNNING，整个 Selector 就返回该状态；
      只有当所有子节点全部 FAILURE 时，才返回 FAILURE。
    """
    def __init__(self, children: List[BTNode]):
        self.children = children
        self.current_index = 0

    def tick(self) -> NodeStatus:
        while self.current_index < len(self.children):
            current_child = self.children[self.current_index]
            status = current_child.tick()
            
            if status == NodeStatus.RUNNING:
                return NodeStatus.RUNNING
            elif status == NodeStatus.SUCCESS:
                self.current_index = 0  # 成功重置索引
                return NodeStatus.SUCCESS
            
            # 如果当前子节点失败，尝试执行下一个备用子节点
            self.current_index += 1
            
        # 所有备用子节点均失败，重置索引并返回失败
        self.current_index = 0
        return NodeStatus.FAILURE