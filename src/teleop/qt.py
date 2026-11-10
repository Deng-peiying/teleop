"""Official Qt layout ported to teleop's new L20 SDK backend.

UI originally adapted from linker-bot/linkerhand-python-sdk's
example/gui_control/gui_control.py. Layout and gesture data are local copies;
the old SDK is not a runtime dependency. Gesture raw poses are not calibration.
"""
import sys
import time
from pathlib import Path
from types import SimpleNamespace
from typing import List
import PyQt5
from PyQt5.QtCore import Qt, pyqtSignal, QTimer, QObject, QEvent, QRect
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QSlider, QLabel, QPushButton, QGroupBox, QScrollArea, QTabWidget,
    QFrame, QSplitter, QMessageBox, QTextEdit
)
from PyQt5.QtGui import QFont, QPainter, QColor, QBrush
from linkerbot_hand import CommandSemantic
from .mapping import DISPLAY_NAMES
from .hand import open_hand, read_named, write_named

LOOP_TIME = 1000


LABELS = {channel: f"{label} ({DISPLAY_NAMES[channel]})" for channel, label in {
    "thumb_rotation": "拇指掌外摆动",
    "thumb_roll": "拇指掌面旋转",
    "thumb_root": "拇指根部",
    "thumb_tip": "拇指尖部",
    "index_abd": "食指侧摆",
    "index_root": "食指根部",
    "index_tip": "食指末端",
    "middle_abd": "中指侧摆",
    "middle_root": "中指根部",
    "middle_tip": "中指末端",
    "ring_abd": "无名指侧摆",
    "ring_root": "无名指根部",
    "ring_tip": "无名指末端",
    "pinky_abd": "小指侧摆",
    "pinky_root": "小指根部",
    "pinky_tip": "小指末端",
}.items()}

PRESETS = {
    '点赞': {'thumb_rotation': 210, 'thumb_roll': 255, 'thumb_root': 255, 'thumb_tip': 255, 'index_abd': 162, 'index_root': 0, 'index_tip': 0, 'middle_abd': 162, 'middle_root': 0, 'middle_tip': 0, 'ring_abd': 144, 'ring_root': 0, 'ring_tip': 0, 'pinky_abd': 100, 'pinky_root': 0, 'pinky_tip': 0},
    '握拳': {'thumb_rotation': 132, 'thumb_roll': 0, 'thumb_root': 96, 'thumb_tip': 144, 'index_abd': 193, 'index_root': 0, 'index_tip': 0, 'middle_abd': 158, 'middle_root': 0, 'middle_tip': 0, 'ring_abd': 128, 'ring_root': 0, 'ring_tip': 0, 'pinky_abd': 91, 'pinky_root': 0, 'pinky_tip': 0},
    '张开': {'thumb_rotation': 245, 'thumb_roll': 255, 'thumb_root': 255, 'thumb_tip': 255, 'index_abd': 193, 'index_root': 255, 'index_tip': 255, 'middle_abd': 148, 'middle_root': 255, 'middle_tip': 255, 'ring_abd': 105, 'ring_root': 255, 'ring_tip': 255, 'pinky_abd': 42, 'pinky_root': 255, 'pinky_tip': 255},
    'OK': {'thumb_rotation': 178, 'thumb_roll': 44, 'thumb_root': 148, 'thumb_tip': 94, 'index_abd': 164, 'index_root': 110, 'index_tip': 71, 'middle_abd': 100, 'middle_root': 255, 'middle_tip': 255, 'ring_abd': 114, 'ring_root': 255, 'ring_tip': 255, 'pinky_abd': 127, 'pinky_root': 255, 'pinky_tip': 255},
    '拇指对中指': {'thumb_rotation': 105, 'thumb_roll': 96, 'thumb_root': 191, 'thumb_tip': 94, 'index_abd': 95, 'index_root': 255, 'index_tip': 255, 'middle_abd': 100, 'middle_root': 55, 'middle_tip': 108, 'ring_abd': 114, 'ring_root': 255, 'ring_tip': 255, 'pinky_abd': 127, 'pinky_root': 255, 'pinky_tip': 255},
    '拇指对无名指': {'thumb_rotation': 60, 'thumb_roll': 115, 'thumb_root': 191, 'thumb_tip': 94, 'index_abd': 95, 'index_root': 255, 'index_tip': 255, 'middle_abd': 100, 'middle_root': 255, 'middle_tip': 255, 'ring_abd': 114, 'ring_root': 72, 'ring_tip': 97, 'pinky_abd': 127, 'pinky_root': 255, 'pinky_tip': 255},
    '拇指对小指': {'thumb_rotation': 70, 'thumb_roll': 0, 'thumb_root': 191, 'thumb_tip': 94, 'index_abd': 95, 'index_root': 255, 'index_tip': 255, 'middle_abd': 100, 'middle_root': 255, 'middle_tip': 255, 'ring_abd': 114, 'ring_root': 255, 'ring_tip': 255, 'pinky_abd': 121, 'pinky_root': 55, 'pinky_tip': 100},
    '准备1': {'thumb_rotation': 210, 'thumb_roll': 255, 'thumb_root': 255, 'thumb_tip': 255, 'index_abd': 162, 'index_root': 0, 'index_tip': 0, 'middle_abd': 162, 'middle_root': 0, 'middle_tip': 0, 'ring_abd': 144, 'ring_root': 0, 'ring_tip': 0, 'pinky_abd': 100, 'pinky_root': 0, 'pinky_tip': 0},
    '壹': {'thumb_rotation': 68, 'thumb_roll': 0, 'thumb_root': 96, 'thumb_tip': 144, 'index_abd': 190, 'index_root': 255, 'index_tip': 255, 'middle_abd': 161, 'middle_root': 0, 'middle_tip': 0, 'ring_abd': 127, 'ring_root': 0, 'ring_tip': 0, 'pinky_abd': 80, 'pinky_root': 0, 'pinky_tip': 0},
    '贰': {'thumb_rotation': 68, 'thumb_roll': 0, 'thumb_root': 96, 'thumb_tip': 144, 'index_abd': 190, 'index_root': 255, 'index_tip': 255, 'middle_abd': 66, 'middle_root': 255, 'middle_tip': 255, 'ring_abd': 127, 'ring_root': 0, 'ring_tip': 0, 'pinky_abd': 80, 'pinky_root': 0, 'pinky_tip': 0},
    '叁': {'thumb_rotation': 68, 'thumb_roll': 0, 'thumb_root': 96, 'thumb_tip': 144, 'index_abd': 200, 'index_root': 255, 'index_tip': 255, 'middle_abd': 132, 'middle_root': 255, 'middle_tip': 255, 'ring_abd': 76, 'ring_root': 255, 'ring_tip': 255, 'pinky_abd': 80, 'pinky_root': 0, 'pinky_tip': 0},
    '肆': {'thumb_rotation': 129, 'thumb_roll': 78, 'thumb_root': 80, 'thumb_tip': 64, 'index_abd': 200, 'index_root': 255, 'index_tip': 255, 'middle_abd': 132, 'middle_root': 255, 'middle_tip': 255, 'ring_abd': 114, 'ring_root': 255, 'ring_tip': 255, 'pinky_abd': 48, 'pinky_root': 255, 'pinky_tip': 255},
    '伍': {'thumb_rotation': 245, 'thumb_roll': 255, 'thumb_root': 255, 'thumb_tip': 255, 'index_abd': 193, 'index_root': 255, 'index_tip': 255, 'middle_abd': 148, 'middle_root': 255, 'middle_tip': 255, 'ring_abd': 105, 'ring_root': 255, 'ring_tip': 255, 'pinky_abd': 42, 'pinky_root': 255, 'pinky_tip': 255},
    '陆': {'thumb_rotation': 245, 'thumb_roll': 255, 'thumb_root': 255, 'thumb_tip': 255, 'index_abd': 156, 'index_root': 0, 'index_tip': 0, 'middle_abd': 126, 'middle_root': 0, 'middle_tip': 0, 'ring_abd': 125, 'ring_root': 0, 'ring_tip': 0, 'pinky_abd': 42, 'pinky_root': 255, 'pinky_tip': 255},
    '漆': {'thumb_rotation': 145, 'thumb_roll': 55, 'thumb_root': 38, 'thumb_tip': 255, 'index_abd': 156, 'index_root': 0, 'index_tip': 164, 'middle_abd': 126, 'middle_root': 0, 'middle_tip': 163, 'ring_abd': 125, 'ring_root': 0, 'ring_tip': 0, 'pinky_abd': 117, 'pinky_root': 0, 'pinky_tip': 0},
    '捌': {'thumb_rotation': 210, 'thumb_roll': 255, 'thumb_root': 255, 'thumb_tip': 255, 'index_abd': 162, 'index_root': 255, 'index_tip': 255, 'middle_abd': 162, 'middle_root': 0, 'middle_tip': 0, 'ring_abd': 144, 'ring_root': 0, 'ring_tip': 0, 'pinky_abd': 100, 'pinky_root': 0, 'pinky_tip': 0},
    '玖': {'thumb_rotation': 85, 'thumb_roll': 37, 'thumb_root': 67, 'thumb_tip': 169, 'index_abd': 162, 'index_root': 255, 'index_tip': 0, 'middle_abd': 162, 'middle_root': 0, 'middle_tip': 0, 'ring_abd': 144, 'ring_root': 0, 'ring_tip': 0, 'pinky_abd': 100, 'pinky_root': 0, 'pinky_tip': 0},
    '动作1': {'thumb_rotation': 245, 'thumb_roll': 255, 'thumb_root': 255, 'thumb_tip': 255, 'index_abd': 193, 'index_root': 255, 'index_tip': 255, 'middle_abd': 148, 'middle_root': 255, 'middle_tip': 255, 'ring_abd': 105, 'ring_root': 255, 'ring_tip': 255, 'pinky_abd': 42, 'pinky_root': 255, 'pinky_tip': 255},
    '动作2': {'thumb_rotation': 245, 'thumb_roll': 255, 'thumb_root': 255, 'thumb_tip': 255, 'index_abd': 125, 'index_root': 255, 'index_tip': 255, 'middle_abd': 129, 'middle_root': 255, 'middle_tip': 255, 'ring_abd': 125, 'ring_root': 255, 'ring_tip': 255, 'pinky_abd': 130, 'pinky_root': 255, 'pinky_tip': 255},
    '动作3': {'thumb_rotation': 245, 'thumb_roll': 255, 'thumb_root': 255, 'thumb_tip': 255, 'index_abd': 193, 'index_root': 255, 'index_tip': 255, 'middle_abd': 148, 'middle_root': 255, 'middle_tip': 255, 'ring_abd': 105, 'ring_root': 255, 'ring_tip': 255, 'pinky_abd': 42, 'pinky_root': 255, 'pinky_tip': 255},
    '动作4': {'thumb_rotation': 245, 'thumb_roll': 255, 'thumb_root': 255, 'thumb_tip': 255, 'index_abd': 125, 'index_root': 255, 'index_tip': 255, 'middle_abd': 129, 'middle_root': 255, 'middle_tip': 255, 'ring_abd': 125, 'ring_root': 255, 'ring_tip': 255, 'pinky_abd': 130, 'pinky_root': 255, 'pinky_tip': 255},
    '动作5': {'thumb_rotation': 210, 'thumb_roll': 255, 'thumb_root': 255, 'thumb_tip': 255, 'index_abd': 255, 'index_root': 255, 'index_tip': 255, 'middle_abd': 255, 'middle_root': 255, 'middle_tip': 255, 'ring_abd': 247, 'ring_root': 255, 'ring_tip': 255, 'pinky_abd': 255, 'pinky_root': 255, 'pinky_tip': 255},
    '动作6': {'thumb_rotation': 210, 'thumb_roll': 255, 'thumb_root': 255, 'thumb_tip': 255, 'index_abd': 0, 'index_root': 255, 'index_tip': 255, 'middle_abd': 0, 'middle_root': 255, 'middle_tip': 255, 'ring_abd': 0, 'ring_root': 255, 'ring_tip': 255, 'pinky_abd': 0, 'pinky_root': 255, 'pinky_tip': 255},
    '动作7': {'thumb_rotation': 210, 'thumb_roll': 255, 'thumb_root': 255, 'thumb_tip': 255, 'index_abd': 255, 'index_root': 255, 'index_tip': 255, 'middle_abd': 255, 'middle_root': 255, 'middle_tip': 255, 'ring_abd': 247, 'ring_root': 255, 'ring_tip': 255, 'pinky_abd': 255, 'pinky_root': 255, 'pinky_tip': 255},
    '动作8': {'thumb_rotation': 210, 'thumb_roll': 255, 'thumb_root': 255, 'thumb_tip': 255, 'index_abd': 0, 'index_root': 255, 'index_tip': 255, 'middle_abd': 0, 'middle_root': 255, 'middle_tip': 255, 'ring_abd': 0, 'ring_root': 255, 'ring_tip': 255, 'pinky_abd': 0, 'pinky_root': 255, 'pinky_tip': 255},
    '动作9': {'thumb_rotation': 210, 'thumb_roll': 255, 'thumb_root': 255, 'thumb_tip': 255, 'index_abd': 125, 'index_root': 255, 'index_tip': 255, 'middle_abd': 129, 'middle_root': 255, 'middle_tip': 255, 'ring_abd': 125, 'ring_root': 255, 'ring_tip': 255, 'pinky_abd': 130, 'pinky_root': 255, 'pinky_tip': 255},
    '根部1': {'thumb_rotation': 245, 'thumb_roll': 255, 'thumb_root': 0, 'thumb_tip': 255, 'index_abd': 125, 'index_root': 0, 'index_tip': 255, 'middle_abd': 129, 'middle_root': 0, 'middle_tip': 255, 'ring_abd': 125, 'ring_root': 0, 'ring_tip': 255, 'pinky_abd': 130, 'pinky_root': 0, 'pinky_tip': 255},
    '根部2': {'thumb_rotation': 245, 'thumb_roll': 255, 'thumb_root': 255, 'thumb_tip': 255, 'index_abd': 125, 'index_root': 255, 'index_tip': 255, 'middle_abd': 129, 'middle_root': 255, 'middle_tip': 255, 'ring_abd': 125, 'ring_root': 255, 'ring_tip': 255, 'pinky_abd': 130, 'pinky_root': 255, 'pinky_tip': 255},
    '末端1': {'thumb_rotation': 219, 'thumb_roll': 255, 'thumb_root': 6, 'thumb_tip': 125, 'index_abd': 125, 'index_root': 0, 'index_tip': 0, 'middle_abd': 129, 'middle_root': 0, 'middle_tip': 0, 'ring_abd': 125, 'ring_root': 0, 'ring_tip': 0, 'pinky_abd': 130, 'pinky_root': 0, 'pinky_tip': 0},
    '末端2': {'thumb_rotation': 219, 'thumb_roll': 255, 'thumb_root': 6, 'thumb_tip': 255, 'index_abd': 125, 'index_root': 0, 'index_tip': 255, 'middle_abd': 129, 'middle_root': 0, 'middle_tip': 255, 'ring_abd': 125, 'ring_root': 0, 'ring_tip': 255, 'pinky_abd': 130, 'pinky_root': 0, 'pinky_tip': 255},
    '末端3': {'thumb_rotation': 219, 'thumb_roll': 255, 'thumb_root': 6, 'thumb_tip': 125, 'index_abd': 125, 'index_root': 0, 'index_tip': 0, 'middle_abd': 129, 'middle_root': 0, 'middle_tip': 0, 'ring_abd': 125, 'ring_root': 0, 'ring_tip': 0, 'pinky_abd': 130, 'pinky_root': 0, 'pinky_tip': 0},
    '末端4': {'thumb_rotation': 219, 'thumb_roll': 255, 'thumb_root': 6, 'thumb_tip': 255, 'index_abd': 125, 'index_root': 0, 'index_tip': 255, 'middle_abd': 129, 'middle_root': 0, 'middle_tip': 255, 'ring_abd': 125, 'ring_root': 0, 'ring_tip': 255, 'pinky_abd': 130, 'pinky_root': 0, 'pinky_tip': 255},
    '默认': {'thumb_rotation': 245, 'thumb_roll': 255, 'thumb_root': 255, 'thumb_tip': 255, 'index_abd': 193, 'index_root': 255, 'index_tip': 255, 'middle_abd': 148, 'middle_root': 255, 'middle_tip': 255, 'ring_abd': 105, 'ring_root': 255, 'ring_tip': 255, 'pinky_abd': 42, 'pinky_root': 255, 'pinky_tip': 255},
}

class DotMatrixWidget(QWidget):
    """点阵显示部件 - 白色到深红色渐变版"""
    
    def __init__(self, parent=None, rows=12, cols=6):
        super().__init__(parent)
        self.rows = rows
        self.cols = cols
        self.dot_size = 6
        self.spacing = 3
        self.data = None
        
        # 计算部件大小 - 增加一些额外空间确保完全显示
        width = cols * (self.dot_size + self.spacing) + self.spacing + 2
        height = rows * (self.dot_size + self.spacing) + self.spacing + 2
        self.setMinimumSize(width, height)
        self.setMaximumSize(width, height)
        
    def set_data(self, data):
        """设置点阵数据"""
        if data is not None:
            try:
                # 如果数据是 numpy 数组，转换为列表
                if hasattr(data, 'tolist'):
                    self.data = data.tolist()
                else:
                    self.data = data
            except Exception as e:
                print(f"转换数据时出错: {e}")
                self.data = None
        else:
            self.data = None
        self.update()
        
    def paintEvent(self, event):
        """绘制点阵"""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        # 绘制背景
        painter.fillRect(self.rect(), QColor('white'))
        
        for row in range(self.rows):
            for col in range(self.cols):
                x = self.spacing + col * (self.dot_size + self.spacing)
                y = self.spacing + row * (self.dot_size + self.spacing)
                
                if self.data is not None:
                    try:
                        # 处理 numpy 数组或普通列表
                        if hasattr(self.data, 'flatten'):
                            flat_data = self.data.flatten()
                        else:
                            flat_data = self.data
                        
                        index = row * self.cols + col
                        if index < len(flat_data):
                            value = flat_data[index]
                            # 确保 value 是标量数值
                            if hasattr(value, 'item'):
                                value = value.item()
                                
                            if value > 0:
                                # 白色到深红色渐变逻辑
                                intensity = min(255, max(0, int(value)))
                                
                                if intensity < 128:
                                    # 白色到浅红色渐变
                                    red = 255
                                    green = 255 - (intensity * 55 // 128)
                                    blue = 255 - (intensity * 55 // 128)
                                    color = QColor(red, green, blue)
                                else:
                                    # 浅红色到深红色渐变
                                    red = 255
                                    green = 200 - ((intensity - 128) * 200 // 127)
                                    blue = 200 - ((intensity - 128) * 200 // 127)
                                    color = QColor(red, green, blue)
                            else:
                                color = QColor('#c8c8c8')  # 默认灰色
                        else:
                            color = QColor('#c8c8c8')  # 默认灰色
                    except Exception as e:
                        print(f"绘制点阵时出错: {e}")
                        color = QColor('#c8c8c8')  # 默认灰色
                else:
                    color = QColor('#c8c8c8')  # 默认灰色
                
                # 绘制圆点
                painter.setBrush(QBrush(color))
                painter.setPen(QColor('#666666'))
                painter.drawEllipse(x, y, self.dot_size, self.dot_size)

class MatrixDisplayWidget(QWidget):
    """矩阵显示部件，包含五个手指的点阵 - 新的排列方式"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.finger_matrices = {}
        self.init_ui()
        
    def init_ui(self):
        """初始化UI - 新的排列方式"""
        main_layout = QVBoxLayout(self)
        main_layout.setSpacing(10)
        main_layout.setContentsMargins(5, 5, 5, 5)
        
        # 第一行：拇指、食指、中指
        first_row_layout = QHBoxLayout()
        first_row_layout.setSpacing(15)
        
        # 拇指
        thumb_frame = self.create_finger_frame("拇指", "thumb_matrix")
        first_row_layout.addWidget(thumb_frame)
        
        # 食指
        index_frame = self.create_finger_frame("食指", "index_matrix")
        first_row_layout.addWidget(index_frame)
        
        # 中指
        middle_frame = self.create_finger_frame("中指", "middle_matrix")
        first_row_layout.addWidget(middle_frame)
        
        first_row_layout.addStretch()
        main_layout.addLayout(first_row_layout)
        
        # 第二行：无名指、小指
        second_row_layout = QHBoxLayout()
        second_row_layout.setSpacing(15)
        
        # 无名指
        ring_frame = self.create_finger_frame("无名指", "ring_matrix")
        second_row_layout.addWidget(ring_frame)
        
        # 小指
        little_frame = self.create_finger_frame("小指", "little_matrix")
        second_row_layout.addWidget(little_frame)
        
        second_row_layout.addStretch()
        main_layout.addLayout(second_row_layout)
        
        main_layout.addStretch()
        
    def create_finger_frame(self, display_name, finger_name):
        """创建单个手指的框架"""
        finger_frame = QWidget()
        finger_layout = QVBoxLayout(finger_frame)
        finger_layout.setSpacing(5)
        finger_layout.setContentsMargins(5, 5, 5, 5)
        
        # 手指标签 - 居中显示
        label = QLabel(display_name)
        label.setAlignment(Qt.AlignCenter)
        label.setStyleSheet("font-weight: bold;")
        finger_layout.addWidget(label)
        
        # 创建点阵部件
        matrix = DotMatrixWidget()
        finger_layout.addWidget(matrix, 0, Qt.AlignCenter)
        
        self.finger_matrices[finger_name] = matrix
        return finger_frame
        
    def update_matrix_data(self, finger_name, data):
        """更新指定手指的点阵数据"""
        if finger_name in self.finger_matrices:
            # 处理 numpy 数组或普通列表
            if data is not None:
                try:
                    # 如果数据是 numpy 数组，先转换为列表
                    if hasattr(data, 'tolist'):
                        data = data.tolist()
                    
                    # 展平数据（如果数据是二维列表或二维数组）
                    if data and (isinstance(data[0], list) or 
                            (hasattr(data, 'ndim') and data.ndim > 1)):
                        # 如果是二维结构，展平为一维
                        flattened = []
                        for item in data:
                            if isinstance(item, (list, tuple)) or (hasattr(item, '__iter__') and not isinstance(item, str)):
                                flattened.extend(item)
                            else:
                                flattened.append(item)
                    else:
                        # 如果已经是一维，直接使用
                        flattened = data
                        
                    self.finger_matrices[finger_name].set_data(flattened)
                except Exception as e:
                    print(f"处理矩阵数据时出错: {e}")
                    # 出错时使用默认灰色点阵
                    default_data = [0] * 72
                    self.finger_matrices[finger_name].set_data(default_data)
            else:
                # 数据为空时使用默认灰色点阵
                default_data = [0] * 72
                self.finger_matrices[finger_name].set_data(default_data)
            
    def initialize_default_matrices(self):
        """初始化显示默认点阵（所有点为灰色）"""
        default_data = [0] * 72  # 12x6的全0矩阵
        for finger_name in self.finger_matrices.keys():
            self.finger_matrices[finger_name].set_data(default_data)

class HandApiManager(QObject):
    """Adapter from the original UI callbacks to the new official SDK."""
    status_updated = pyqtSignal(str, str)
    matrix_data_updated = pyqtSignal(dict)

    def __init__(self):
        super().__init__()
        self._connection = open_hand()
        self.api = self._connection.__enter__()
        try:
            capabilities = self.api.get_capabilities()
            self.names = capabilities.joint_names
            pose = read_named(self.api)
            self.hand_joint = "L20"
            self.hand_type = "right"
            self.ranges = {j.name: j.raw_angle_range for j in capabilities.joints}
            self.speed_range = capabilities.command(CommandSemantic.SPEEDS).raw_ranges[0]
            self.current_range = capabilities.command(CommandSemantic.CURRENT_LIMITS).raw_ranges[0]
            self.hand_config = SimpleNamespace(
                joint_names=[LABELS[name] for name in self.names],
                init_pos=[pose[name] for name in self.names],
                preset_actions={label:[target[name] for name in self.names]
                                for label,target in PRESETS.items()})
            self.matrix_timer = QTimer(self)
            self.matrix_timer.timeout.connect(self.update_matrix_data)
            self.matrix_timer.start(500)
        except BaseException:
            self._connection.__exit__(*sys.exc_info())
            self.api = None
            raise

    def update_matrix_data(self):
        if self.api is None:
            return
        try:
            sample = self.api.get_force(timeout=0.5)
            matrices = {}
            for matrix in sample.tactile:
                if matrix.online is False:
                    continue
                finger = "little" if matrix.name == "pinky" else matrix.name
                matrices[f"{finger}_matrix"] = [v for row in matrix.raw for v in row]
            self.matrix_data_updated.emit(matrices)
        except Exception as exc:
            self.status_updated.emit("error", f"触觉读取失败: {exc}")

    def publish_joint_state(self, positions: List[int]):
        try:
            write_named(self.api, dict(zip(self.names, positions, strict=True)))
            self.status_updated.emit("info", "关节状态已发送")
        except Exception as exc:
            self.status_updated.emit("error", f"发送失败: {exc}")

    def publish_speed(self, value: int):
        try:
            self.api.set_speeds([value]*len(self.names)).wait(timeout=1.0)
            self.status_updated.emit("info", f"速度 raw 已设为 {value}")
        except Exception as exc:
            self.status_updated.emit("error", f"速度设置失败: {exc}")

    def publish_torque(self, value: int):
        # The old button said torque; the new L20 SDK exposes raw current limits.
        try:
            self.api.set_current_limits([value]*len(self.names)).wait(timeout=1.0)
            self.status_updated.emit("info", f"电流限幅 raw 已设为 {value}")
        except Exception as exc:
            self.status_updated.emit("error", f"电流限幅设置失败: {exc}")

    def shutdown(self):
        self.matrix_timer.stop()
        if self.api is not None:
            self.api = None
            self._connection.__exit__(None, None, None)


class HandControlGUI(QWidget):
    """灵巧手控制界面"""
    status_updated = pyqtSignal(str, str)  # 状态类型, 消息内容

    def __init__(self, api_manager: HandApiManager):
        super().__init__()
        
        # 循环控制变量
        self.cycle_timer = None  # 循环定时器
        self.current_action_index = -1  # 当前动作索引
        self.preset_buttons = []  # 存储预设动作按钮引用
        
        # 设置API管理器
        self.api_manager = api_manager
        self.api_manager.status_updated.connect(self.update_status)
        self.api_manager.matrix_data_updated.connect(self.update_matrix_display)
        
        # 获取手部配置
        self.hand_joint = self.api_manager.hand_joint
        self.hand_type = self.api_manager.hand_type
        self.hand_config = self.api_manager.hand_config
        
        # 初始化UI
        self.init_ui()
        
        # 设置定时器发布关节状态
        self.publish_timer = QTimer(self)
        self.publish_timer.setInterval(30)  # 10Hz发布频率
        self.publish_timer.timeout.connect(self.publish_joint_state)
        self.publish_timer.start()

    def init_ui(self):
        """初始化用户界面"""
        # 设置窗口属性
        self.setWindowTitle(f'灵巧手控制界面 - {self.hand_type} {self.hand_joint} · 新版 SDK')
        self.setMinimumSize(1200, 900)
        
        # 设置样式
        self.setStyleSheet("""
            QWidget {
                font-family: 'Microsoft YaHei', 'SimHei', sans-serif;
                font-size: 12px;
            }
            QGroupBox {
                border: 1px solid #CCCCCC;
                border-radius: 6px;
                margin-top: 6px;
                padding: 10px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px 0 5px;
                color: #165DFF;
                font-weight: bold;
            }
            QSlider::groove:horizontal {
                border: 1px solid #999999;
                height: 8px;
                border-radius: 4px;
                background: #CCCCCC;
                margin: 2px 0;
            }
            QSlider::handle:horizontal {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #165DFF, stop:1 #0E42D2);
                border: 1px solid #5C8AFF;
                width: 18px;
                margin: -5px 0;
                border-radius: 9px;
            }
            QPushButton {
                background-color: #E0E0E0;
                border: 1px solid #CCCCCC;
                border-radius: 4px;
                padding: 5px 10px;
                min-width: 80px;
            }
            QPushButton:hover {
                background-color: #F0F0F0;
            }
            QPushButton:pressed {
                background-color: #D0D0D0;
            }
            QPushButton[category="preset"] {
                background-color: #E6F7FF;
                color: #1890FF;
                border-color: #91D5FF;
            }
            QPushButton[category="preset"]:hover {
                background-color: #B3E0FF;
            }
            QPushButton[category="action"] {
                background-color: #FFF7E6;
                color: #FA8C16;
                border-color: #FFD591;
            }
            QPushButton[category="action"]:hover {
                background-color: #FFE6B3;
            }
            QPushButton[category="danger"] {
                background-color: #FFF1F0;
                color: #F5222D;
                border-color: #FFCCC7;
            }
            QPushButton[category="danger"]:hover {
                background-color: #FFE8E6;
            }
            QLabel#StatusLabel {
                padding: 5px;
                border-radius: 4px;
            }
            QLabel#StatusInfo {
                background-color: #F0F7FF;
                color: #0066CC;
            }
            QLabel#StatusError {
                background-color: #FFF0F0;
                color: #CC0000;
            }
            /* 数值显示面板样式 */
            QTextEdit#ValueDisplay {
                background-color: #F8F8F8;
                border: 1px solid #CCCCCC;
                border-radius: 4px;
                padding: 10px;
                font-family: Consolas, monospace;
                font-size: 12px;
            }
        """)
        
        # 创建主垂直布局
        main_layout = QVBoxLayout(self)
        
        # 创建水平分割器（原有三个面板）
        splitter = QSplitter(Qt.Horizontal)
        
        # 创建左侧关节控制面板
        self.joint_control_panel = self.create_joint_control_panel()
        splitter.addWidget(self.joint_control_panel)
        
        # 创建中间预设动作面板
        self.preset_actions_panel = self.create_preset_actions_panel()
        splitter.addWidget(self.preset_actions_panel)
        
        # 创建右侧状态监控面板
        self.status_monitor_panel = self.create_status_monitor_panel()
        splitter.addWidget(self.status_monitor_panel)
        
        # 设置分割器比例
        splitter.setSizes([500, 300, 400])
        
        # 添加分割器到主布局，并设置拉伸因子为1（可伸缩）
        main_layout.addWidget(splitter, stretch=1)
        
        # 创建并添加数值显示面板，设置拉伸因子为0（不可伸缩）
        self.value_display_panel = self.create_value_display_panel()
        main_layout.addWidget(self.value_display_panel, stretch=0)
        
        # 初始更新数值显示
        self.update_value_display()

    def create_joint_control_panel(self):
        """创建关节控制面板"""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        
        # 创建标题
        title_label = QLabel(f"关节控制 - {self.hand_joint}")
        title_label.setFont(QFont("Microsoft YaHei", 14, QFont.Bold))
        layout.addWidget(title_label)

        # 创建滑动条滚动区域
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setFrameShape(QFrame.NoFrame)
        
        scroll_content = QWidget()
        self.sliders_layout = QGridLayout(scroll_content)
        self.sliders_layout.setSpacing(10)
        
        # 创建滑动条
        self.create_joint_sliders()
        
        scroll_area.setWidget(scroll_content)
        layout.addWidget(scroll_area)
        
        return panel

    def create_joint_sliders(self):
        """创建关节滑动条"""
        # 清除现有滑动条
        for i in reversed(range(self.sliders_layout.count())):
            item = self.sliders_layout.itemAt(i)
            if item.widget():
                item.widget().deleteLater()
        
        # 创建新滑动条
        self.sliders = []
        self.slider_labels = []
        
        for i, (name, value) in enumerate(zip(
            self.hand_config.joint_names, self.hand_config.init_pos
        )):
            # 创建标签
            label = QLabel(f"{name}: {value}")
            label.setMinimumWidth(120)
            
            # 创建滑动条
            slider = QSlider(Qt.Horizontal)
            limits = self.api_manager.ranges[self.api_manager.names[i]]
            slider.setRange(int(limits.minimum), int(limits.maximum))
            slider.setValue(value)
            slider.valueChanged.connect(
                lambda val, idx=i: self.on_slider_value_changed(idx, val)
            )
            
            # 添加到布局
            row, col = divmod(i, 1)
            self.sliders_layout.addWidget(label, row, 0)
            self.sliders_layout.addWidget(slider, row, 1)
            
            self.sliders.append(slider)
            self.slider_labels.append(label)

    def create_preset_actions_panel(self):
        """创建预设动作面板"""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        
        # 系统预设动作
        sys_preset_group = QGroupBox("系统预设")
        sys_preset_layout = QGridLayout(sys_preset_group)
        sys_preset_layout.setSpacing(8)
        
        # 添加系统预设动作按钮
        self.create_system_preset_buttons(sys_preset_layout)
        layout.addWidget(sys_preset_group)
        
        # 添加动作按钮
        actions_layout = QHBoxLayout()
        
        # 添加循环运行按钮
        self.cycle_button = QPushButton("循环预设动作")
        self.cycle_button.setProperty("category", "action")
        self.cycle_button.clicked.connect(self.on_cycle_clicked)
        actions_layout.addWidget(self.cycle_button)
        
        self.home_button = QPushButton("回到初始位置")
        self.home_button.setProperty("category", "action")
        self.home_button.clicked.connect(self.on_home_clicked)
        actions_layout.addWidget(self.home_button)
        
        self.stop_button = QPushButton("停止所有动作")
        self.stop_button.setProperty("category", "danger")
        self.stop_button.clicked.connect(self.on_stop_clicked)
        actions_layout.addWidget(self.stop_button)
        
        layout.addLayout(actions_layout)
        
        return panel

    def create_system_preset_buttons(self, parent_layout):
        """创建系统预设动作按钮"""
        self.preset_buttons = []  # 清空按钮列表
        if self.hand_config.preset_actions:
            buttons = []
            for idx, (name, positions) in enumerate(self.hand_config.preset_actions.items()):
                button = QPushButton(name)
                button.setProperty("category", "preset")
                button.clicked.connect(
                    lambda checked, pos=positions: self.on_preset_action_clicked(pos)
                )
                buttons.append(button)
                self.preset_buttons.append(button)  # 保存按钮引用
                
            # 添加到网格布局
            cols = 2
            for i, button in enumerate(buttons):
                row, col = divmod(i, cols)
                parent_layout.addWidget(button, row, col)

    def create_status_monitor_panel(self):
        """创建状态监控面板"""
        panel = QWidget()
        layout = QVBoxLayout(panel)

        # 标题
        title_label = QLabel("状态监控")
        title_label.setFont(QFont("Microsoft YaHei", 14, QFont.Bold))
        layout.addWidget(title_label)

        # 快速设置
        quick_set_gb = QGroupBox("快速设置")
        qv_layout = QVBoxLayout(quick_set_gb)

        # 速度行
        speed_hbox = QHBoxLayout()
        speed_hbox.addWidget(QLabel("速度:"))
        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setRange(int(self.api_manager.speed_range.minimum),
                                   int(self.api_manager.speed_range.maximum))
        self.speed_slider.setValue(int(self.api_manager.speed_range.maximum))
        self.speed_slider.setMinimumWidth(150)
        speed_hbox.addWidget(self.speed_slider)
        self.speed_val_lbl = QLabel("255")
        self.speed_val_lbl.setMinimumWidth(30)
        speed_hbox.addWidget(self.speed_val_lbl)
        self.speed_btn = QPushButton("设置速度")
        self.speed_btn.clicked.connect(
            lambda: self.api_manager.publish_speed(self.speed_slider.value()))
        speed_hbox.addWidget(self.speed_btn)
        speed_hbox.addStretch()
        qv_layout.addLayout(speed_hbox)

        # 电流限幅行
        torque_hbox = QHBoxLayout()
        torque_hbox.addWidget(QLabel("电流限幅:"))
        self.torque_slider = QSlider(Qt.Horizontal)
        self.torque_slider.setRange(int(self.api_manager.current_range.minimum),
                                    int(self.api_manager.current_range.maximum))
        self.torque_slider.setValue(int(self.api_manager.current_range.maximum))
        self.torque_slider.setMinimumWidth(150)
        torque_hbox.addWidget(self.torque_slider)
        self.torque_val_lbl = QLabel("255")
        self.torque_val_lbl.setMinimumWidth(30)
        torque_hbox.addWidget(self.torque_val_lbl)
        self.torque_btn = QPushButton("设置电流限幅")
        self.torque_btn.clicked.connect(
            lambda: self.api_manager.publish_torque(self.torque_slider.value()))
        torque_hbox.addWidget(self.torque_btn)
        torque_hbox.addStretch()
        qv_layout.addLayout(torque_hbox)

        layout.addWidget(quick_set_gb)

        # 标签页部分
        tab_widget = QTabWidget()

        # 系统信息标签页
        sys_info_widget = QWidget()
        sys_info_layout = QVBoxLayout(sys_info_widget)

        conn_group = QGroupBox("连接状态")
        conn_layout = QVBoxLayout(conn_group)
        self.connection_status = QLabel("手部API已连接")
        self.connection_status.setObjectName("StatusLabel")
        self.connection_status.setObjectName("StatusInfo")
        conn_layout.addWidget(self.connection_status)

        hand_info_group = QGroupBox("手部信息")
        hand_info_layout = QVBoxLayout(hand_info_group)
        info_text = f"""手部类型: {self.hand_type}
关节型号: {self.hand_joint}
关节数量: {len(self.hand_config.joint_names)}"""
        self.hand_info_label = QLabel(info_text)
        self.hand_info_label.setWordWrap(True)
        hand_info_layout.addWidget(self.hand_info_label)

        sys_info_layout.addWidget(conn_group)
        sys_info_layout.addWidget(hand_info_group)
        
        # 添加矩阵热力图显示
        matrix_group = QGroupBox("手指矩阵热力图")
        matrix_layout = QVBoxLayout(matrix_group)
        self.matrix_display = MatrixDisplayWidget()
        matrix_layout.addWidget(self.matrix_display)
        sys_info_layout.addWidget(matrix_group)

        sys_info_layout.addStretch()
        tab_widget.addTab(sys_info_widget, "系统信息")

        # 状态日志标签页
        log_widget = QWidget()
        log_layout = QVBoxLayout(log_widget)
        self.status_log = QLabel("等待系统启动...")
        self.status_log.setObjectName("StatusLabel")
        self.status_log.setObjectName("StatusInfo")
        self.status_log.setWordWrap(True)
        self.status_log.setMinimumHeight(300)
        log_layout.addWidget(self.status_log)
        clear_log_btn = QPushButton("清除日志")
        clear_log_btn.clicked.connect(self.clear_status_log)
        log_layout.addWidget(clear_log_btn)
        tab_widget.addTab(log_widget, "状态日志")

        layout.addWidget(tab_widget)

        # 实时更新滑块值
        self.speed_slider.valueChanged.connect(
            lambda v: self.speed_val_lbl.setText(str(v)))
        self.torque_slider.valueChanged.connect(
            lambda v: self.torque_val_lbl.setText(str(v)))
        return panel

    def create_value_display_panel(self):
        """创建滑动条数值显示面板"""
        panel = QGroupBox("关节数值列表")
        layout = QVBoxLayout(panel)
        
        # 设置布局上下间隔为20像素
        layout.setContentsMargins(10, 20, 10, 20)
        
        self.value_display = QTextEdit()
        self.value_display.setObjectName("ValueDisplay")
        self.value_display.setReadOnly(True)
        self.value_display.setMinimumHeight(60)
        self.value_display.setMaximumHeight(80)
        self.value_display.setText("[]")
        
        layout.addWidget(self.value_display)
        
        return panel

    def update_matrix_display(self, matrix_data):
        """更新矩阵显示"""
        for finger_name, data in matrix_data.items():
            self.matrix_display.update_matrix_data(finger_name, data)

    def on_slider_value_changed(self, index: int, value: int):
        """滑动条值改变事件处理"""
        if 0 <= index < len(self.slider_labels):
            joint_name = self.hand_config.joint_names[index]
            self.slider_labels[index].setText(f"{joint_name}: {value}")
            
        # 更新数值显示
        self.update_value_display()

    def update_value_display(self):
        """更新数值显示面板内容"""
        values = [slider.value() for slider in self.sliders]
        self.value_display.setText(f"{values}")

    def on_preset_action_clicked(self, positions: List[int]):
        """预设动作按钮点击事件处理"""
        if len(positions) != len(self.sliders):
            QMessageBox.warning(
                self, "动作不匹配", 
                f"预设动作关节数量({len(positions)})与当前关节数量({len(self.sliders)})不匹配"
            )
            return
            
        # 更新滑动条
        for i, (slider, pos) in enumerate(zip(self.sliders, positions)):
            slider.setValue(pos)
            self.on_slider_value_changed(i, pos)
            
        # 发布关节状态
        self.publish_joint_state()

    def on_home_clicked(self):
        """回到初始位置按钮点击事件处理"""
        for slider, pos in zip(self.sliders, self.hand_config.init_pos):
            slider.setValue(pos)
            
        self.publish_joint_state()
        self.status_updated.emit("info", "回到初始位置")
        
        # 更新数值显示
        self.update_value_display()

    def on_stop_clicked(self):
        """停止所有动作按钮点击事件处理"""
        # 停止循环定时器
        if self.cycle_timer and self.cycle_timer.isActive():
            self.cycle_timer.stop()
            self.cycle_timer = None
            self.cycle_button.setText("循环预设动作")
            self.reset_preset_buttons_color()
            
        self.status_updated.emit("warning", "已停止所有动作")

    def on_cycle_clicked(self):
        """循环运行预设动作按钮点击事件处理"""
        if not self.hand_config.preset_actions:
            QMessageBox.warning(self, "无预设动作", "当前手部型号没有预设动作可循环运行")
            return
            
        if self.cycle_timer and self.cycle_timer.isActive():
            # 停止循环
            self.cycle_timer.stop()
            self.cycle_timer = None
            self.cycle_button.setText("循环预设动作")
            self.reset_preset_buttons_color()
            self.status_updated.emit("info", "已停止循环运行预设动作")
        else:
            # 开始循环
            self.current_action_index = -1
            self.cycle_timer = QTimer(self)
            self.cycle_timer.timeout.connect(self.run_next_action)
            self.cycle_timer.start(LOOP_TIME)
            self.cycle_button.setText("停止循环运行")
            self.status_updated.emit("info", "开始循环运行预设动作")
            self.run_next_action()

    def run_next_action(self):
        """运行下一个预设动作"""
        if not self.hand_config.preset_actions:
            return
            
        # 重置所有按钮颜色
        self.reset_preset_buttons_color()
        
        # 计算下一个动作索引
        self.current_action_index = (self.current_action_index + 1) % len(self.hand_config.preset_actions)
        
        # 获取下一个动作
        action_names = list(self.hand_config.preset_actions.keys())
        action_name = action_names[self.current_action_index]
        action_positions = self.hand_config.preset_actions[action_name]
        
        # 执行动作
        self.on_preset_action_clicked(action_positions)
        
        # 高亮当前动作按钮
        if 0 <= self.current_action_index < len(self.preset_buttons):
            button = self.preset_buttons[self.current_action_index]
            button.setStyleSheet("background-color: green; color: white; border-color: #91D5FF;")
            
        self.status_updated.emit("info", f"运行预设动作: {action_name}")

    def reset_preset_buttons_color(self):
        """重置所有预设按钮颜色"""
        for button in self.preset_buttons:
            button.setStyleSheet("")
            button.setProperty("category", "preset")
            button.style().unpolish(button)
            button.style().polish(button)

    def publish_joint_state(self):
        """发布当前关节状态"""
        positions = [slider.value() for slider in self.sliders]
        self.api_manager.publish_joint_state(positions)

    def update_status(self, status_type: str, message: str):
        """更新状态显示"""
        # 更新连接状态
        if status_type == "info" and "API初始化成功" in message:
            self.connection_status.setText("手部API已连接")
            self.connection_status.setObjectName("StatusLabel")
            self.connection_status.setObjectName("StatusInfo")
            
        # 更新日志
        current_time = time.strftime("%H:%M:%S")
        log_entry = f"[{current_time}] {message}\n"
        current_log = self.status_log.text()
        
        if len(current_log) > 10000:
            current_log = current_log[-10000:]
            
        self.status_log.setText(log_entry + current_log)
        
        # 设置日志样式
        self.status_log.setObjectName("StatusLabel")
        if status_type == "error":
            self.status_log.setObjectName("StatusError")
        else:
            self.status_log.setObjectName("StatusInfo")

    def clear_status_log(self):
        """清除状态日志"""
        self.status_log.setText("日志已清除")
        self.status_log.setObjectName("StatusLabel")
        self.status_log.setObjectName("StatusInfo")

    def closeEvent(self, event):
        """窗口关闭事件处理"""
        if self.cycle_timer and self.cycle_timer.isActive():
            self.cycle_timer.stop()
        if self.publish_timer and self.publish_timer.isActive():
            self.publish_timer.stop()
        self.api_manager.shutdown()
        super().closeEvent(event)

def main():
    # Resolve plugins explicitly: Qt's relocated prefix can corrupt Chinese paths.
    QApplication.addLibraryPath(str(Path(PyQt5.__file__).parent / "Qt5/plugins"))
    app = QApplication(sys.argv)
    manager = HandApiManager()
    try:
        window = HandControlGUI(manager)
        window.status_updated = manager.status_updated
        app.aboutToQuit.connect(manager.shutdown)
        window.show()
        return app.exec_()
    finally:
        manager.shutdown()


if __name__ == "__main__":
    sys.exit(main())

