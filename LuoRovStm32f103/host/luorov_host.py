#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LuoRov 八推进器 ROV 地面站上位机 (PySide6 · Flutter Material 3 风格)

完整功能:
  1. 三工作台架构:
     - 「🎮 遥控驾驶舱」: 5 种运动模式 (纯手动 / 姿态自稳 / 定深遥控 / 定航自稳 / 全闭环悬停)、
       手柄+键盘(WASD/QE/Space/C) 实时 25Hz 操控、单电机直通点动调试、人工地平仪、FinsROV 推进器拓扑图
     - 「🎛️ PID 与混控调参」: 实时双轨迹示波器 (目标 vs 实际 + PID输出 + 阶跃测试)、
       7 路串级 PID 参数读写、FinsROV V33 / LuoRov 8推进器分配映射与正反桨切换、STM32 Flash 掉电固化
     - 「⚡ 电机测试」: 电机选择、-100.0%~+100.0% 百分比精确输出(含实测 ±10.5% 死区记录)、
       百分比-推力(g)实时曲线绘图与 CSV 导入导出
  2. 自定义矢量 ROV 应用图标 + 无边框圆角 AppBar + 「深海蓝 / 翡翠绿」双主题实时切换

依赖: pyserial, PySide6, pygame (可选，用于手柄)
运行: python luorov_host.py
"""

from collections import deque
import math
import queue
import struct
import sys
import threading
import time

import serial
import serial.tools.list_ports

from PySide6.QtCore import Qt, QTimer, QPointF, QRectF, Signal, QObject
from PySide6.QtGui import (
    QColor,
    QFont,
    QIcon,
    QKeySequence,
    QLinearGradient,
    QPainter,
    QPainterPath,
    QPen,
    QPixmap,
    QPolygonF,
    QRadialGradient,
    QShortcut,
)
from PySide6.QtWidgets import (
    QApplication,
    QComboBox,
    QDoubleSpinBox,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSizeGrip,
    QSlider,
    QSpinBox,
    QStackedWidget,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from luorov_motor_test import MotorTestPage

# 手柄读取 (pygame 惰性导入: 未安装时不崩溃，不影响串口与键盘操控)
try:
    from luorov_gamepad import GamepadReader, AXIS_INDEX, DEFAULT_FACE
except Exception:  # noqa: BLE001
    GamepadReader = None
    AXIS_INDEX = {"LX": 0, "LY": 1, "RX": 2, "RY": 3}
    DEFAULT_FACE = {"A": 0, "B": 1, "X": 2, "Y": 3}


# =============================================================================
# 通信协议常量 (与固件 protocol.h / control.h 严格一致)
# =============================================================================
SYNC1 = 0xAA
SYNC2 = 0x55

CMD_PING = 0x01
CMD_SET_MOTOR = 0x02
CMD_SET_ALL = 0x03
CMD_STOP = 0x04
CMD_ARM = 0x05
CMD_ZERO_DEPTH = 0x06
CMD_SET_CONTROL = 0x07
CMD_SET_TARGETS = 0x08
CMD_SET_PID = 0x09
CMD_GET_PARAMS = 0x0A
CMD_SET_MIXER = 0x0B
CMD_FLASH_OP = 0x0C

TELE_STATUS = 0x80
TELE_ACK = 0x81
TELE_PARAMS = 0x82

# 68 字节扩展遥测结构:
#   52B 基础字段 + <BBhihhhh (ctrl_mode, sys_flags, target_yaw_cdeg, target_depth_mm, 4x pid_out)
TELE_FMT_68 = "<IiihhhhhhhhhhHHHHHHHHBBBBBBhihhhh"
TELE_FMT_52 = "<IiihhhhhhhhhhHHHHHHHHBBBB"

MODE_INFO = [
    (0, "纯手动", "MANUAL · 四轴开环直通混控"),
    (1, "姿态自稳", "STABILIZE · 横滚/俯仰串级自稳"),
    (2, "定深遥控", "DEPTH HOLD · 定深 + 横滚/俯仰自稳"),
    (3, "定航自稳", "HEADING · 横滚/俯仰/偏航三轴自稳"),
    (4, "全闭环悬停", "FULL HOVER · 定深 + 三轴姿态全闭环"),
]

PID_LOOPS_META = [
    (0, "深度位置环", "Depth (cm → 推力)", (10.0, 1.2, 4.0, 180.0, 500.0)),
    (1, "横滚外环·角度", "Roll Angle (° → dps)", (3.5, 0.15, 0.0, 25.0, 120.0)),
    (2, "横滚内环·速率", "Roll Rate (dps → 推力)", (2.6, 0.0, 0.04, 80.0, 380.0)),
    (3, "俯仰外环·角度", "Pitch Angle (° → dps)", (3.5, 0.15, 0.0, 25.0, 120.0)),
    (4, "俯仰内环·速率", "Pitch Rate (dps → 推力)", (2.6, 0.0, 0.04, 80.0, 380.0)),
    (5, "偏航外环·角度", "Yaw Angle (° → dps)", (2.5, 0.05, 0.0, 20.0, 90.0)),
    (6, "偏航内环·速率", "Yaw Rate (dps → 推力)", (3.0, 0.0, 0.05, 80.0, 380.0)),
]

SLOT_LABELS = [
    "外·左前 H_FL",
    "外·左后 H_RL",
    "外·右前 H_FR",
    "外·右后 H_RR",
    "内·左前 V_FL",
    "内·左后 V_RL",
    "内·右前 V_FR",
    "内·右后 V_RR",
]

MOTOR_NAMES = [
    "M1 通道0 (PA6)",
    "M2 通道1 (PA7)",
    "M3 通道2 (PB0)",
    "M4 通道3 (PB1)",
    "M5 通道4 (PB6)",
    "M6 通道5 (PB7)",
    "M7 通道6 (PB8)",
    "M8 通道7 (PB9)",
]


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def build_frame(type_: int, payload: bytes = b"") -> bytes:
    frame = bytes([SYNC1, SYNC2, len(payload), type_]) + payload
    crc = crc16_modbus(frame[2:])
    return frame + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


class FrameParser:
    """字节流 -> 完整帧解析 (与固件状态机一致)。"""

    def __init__(self):
        self.buf = bytearray()

    def feed(self, data: bytes):
        self.buf.extend(data)
        out = []
        while True:
            frame = self._try_extract()
            if frame is None:
                break
            out.append(frame)
        return out

    def _try_extract(self):
        buf = self.buf
        i = 0
        n = len(buf)
        while i < n - 1:
            if buf[i] == SYNC1 and buf[i + 1] == SYNC2:
                break
            i += 1
        if i > 0:
            del buf[:i]
        if len(buf) < 4:
            return None
        ln = buf[2]
        total = 6 + ln
        if len(buf) < total:
            return None
        frame = bytes(buf[:total])
        del buf[:total]
        crc = frame[-1] << 8 | frame[-2]
        if crc16_modbus(frame[2:-2]) != crc:
            return None
        return frame[3], frame[4 : 4 + ln]


# =============================================================================
# Flutter Material 3 双主题定义 (蓝色主题 & 绿色主题)
# =============================================================================
THEMES = {
    "blue": {
        "name": "深海蓝",
        "bg_root": "#0C1320",
        "bg_appbar": "#101B2D",
        "surface": "#142136",
        "surface_high": "#1A2B45",
        "surface_inset": "#0F192A",
        "outline": "#243957",
        "outline_soft": "#1C2E48",
        "primary": "#38BDF8",
        "primary_hover": "#7DD3FC",
        "primary_container": "#123859",
        "on_primary": "#062033",
        "secondary": "#818CF8",
        "tertiary": "#2DD4BF",
        "sky_top": "#0F294A",
        "sky_bot": "#1D4E89",
        "sea_top": "#0E5A75",
        "sea_bot": "#072230",
        "icon_g1": "#0284C7",
        "icon_g2": "#38BDF8",
        "text_primary": "#EDF4FC",
        "text_secondary": "#94ADC9",
        "text_muted": "#5E7796",
        "ok": "#34D399",
        "warn": "#FBBF24",
        "danger": "#F87171",
        "danger_bg": "#991B1B",
        "danger_hover": "#DC2626",
    },
    "green": {
        "name": "翡翠绿",
        "bg_root": "#0A1512",
        "bg_appbar": "#0E1D19",
        "surface": "#122520",
        "surface_high": "#19332B",
        "surface_inset": "#0D1C18",
        "outline": "#22463B",
        "outline_soft": "#1A372E",
        "primary": "#34D399",
        "primary_hover": "#6EE7B7",
        "primary_container": "#104233",
        "on_primary": "#04261B",
        "secondary": "#2DD4BF",
        "tertiary": "#A3E635",
        "sky_top": "#0E342D",
        "sky_bot": "#17594A",
        "sea_top": "#0F4C45",
        "sea_bot": "#06211C",
        "icon_g1": "#059669",
        "icon_g2": "#34D399",
        "text_primary": "#ECFDF5",
        "text_secondary": "#8FB8A9",
        "text_muted": "#567D70",
        "ok": "#4ADE80",
        "warn": "#FBBF24",
        "danger": "#F87171",
        "danger_bg": "#991B1B",
        "danger_hover": "#DC2626",
    },
}


def build_qss(t: dict) -> str:
    return f"""
    QMainWindow {{
        background: transparent;
    }}

    QFrame#WindowShell {{
        background-color: {t['bg_root']};
        border: 1px solid {t['outline']};
        border-radius: 14px;
        color: {t['text_primary']};
        font-family: "Microsoft YaHei UI", "Segoe UI", sans-serif;
        font-size: 12px;
    }}

    QFrame#AppBar {{
        background-color: {t['bg_appbar']};
        border-top-left-radius: 13px;
        border-top-right-radius: 13px;
        border-bottom: 1px solid {t['outline_soft']};
    }}

    QFrame.M3Card {{
        background-color: {t['surface']};
        border: 1px solid {t['outline_soft']};
        border-radius: 16px;
    }}

    QFrame.M3Tile {{
        background-color: {t['surface_inset']};
        border: 1px solid {t['outline_soft']};
        border-radius: 12px;
    }}

    QLabel {{
        color: {t['text_primary']};
        background: transparent;
    }}

    QLabel.CardTitle {{
        color: {t['text_primary']};
        font-size: 13px;
        font-weight: 700;
    }}

    QLabel.CardSub {{
        color: {t['text_muted']};
        font-size: 10px;
        font-weight: 600;
        letter-spacing: 0.8px;
    }}

    QLabel.Muted {{
        color: {t['text_muted']};
        font-size: 11px;
    }}

    QLabel.MonoValue {{
        font-family: "Cascadia Code", "Consolas", monospace;
        color: {t['text_primary']};
        font-weight: 700;
    }}

    QComboBox, QDoubleSpinBox, QSpinBox {{
        background-color: {t['surface_inset']};
        color: {t['text_primary']};
        border: 1px solid {t['outline']};
        border-radius: 9px;
        padding: 3px 8px;
        min-height: 22px;
        font-family: "Cascadia Code", "Microsoft YaHei UI", sans-serif;
    }}
    QComboBox:hover, QDoubleSpinBox:hover, QSpinBox:hover {{
        border-color: {t['primary']};
    }}
    QComboBox::drop-down {{
        border: none;
        width: 18px;
    }}
    QComboBox QAbstractItemView {{
        background-color: {t['surface']};
        color: {t['text_primary']};
        border: 1px solid {t['outline']};
        selection-background-color: {t['primary_container']};
    }}

    QPushButton {{
        background-color: {t['surface_high']};
        color: {t['text_primary']};
        border: 1px solid {t['outline']};
        border-radius: 11px;
        padding: 5px 12px;
        font-weight: 600;
    }}
    QPushButton:hover {{
        background-color: {t['primary_container']};
        border-color: {t['primary']};
    }}
    QPushButton:pressed {{
        background-color: {t['surface_inset']};
    }}

    QPushButton.FilledBtn {{
        background-color: {t['primary']};
        color: {t['on_primary']};
        border: none;
        border-radius: 12px;
        padding: 6px 14px;
        font-weight: 700;
    }}
    QPushButton.FilledBtn:hover {{
        background-color: {t['primary_hover']};
    }}

    QPushButton#ConnectBtn {{
        background-color: {t['primary']};
        color: {t['on_primary']};
        border: none;
        border-radius: 14px;
        padding: 6px 16px;
        font-weight: 700;
    }}
    QPushButton#ConnectBtn:hover {{
        background-color: {t['primary_hover']};
    }}
    QPushButton#ConnectBtn[connected="true"] {{
        background-color: {t['surface_high']};
        border: 1px solid {t['warn']};
        color: {t['warn']};
    }}

    /* Flutter Segmented Pill */
    QFrame.SegmentBox {{
        background-color: {t['surface_inset']};
        border: 1px solid {t['outline']};
        border-radius: 14px;
    }}
    QPushButton.SegBtn {{
        background: transparent;
        border: none;
        border-radius: 11px;
        padding: 4px 11px;
        font-size: 11px;
        font-weight: 600;
        color: {t['text_secondary']};
    }}
    QPushButton.SegBtn:hover {{
        color: {t['text_primary']};
    }}
    QPushButton.SegBtn[active="true"] {{
        background-color: {t['primary']};
        color: {t['on_primary']};
        font-weight: 700;
    }}

    /* 正反桨极性切换按钮 */
    QPushButton.DirBtn[neg="false"] {{
        background-color: {t['primary_container']};
        border: 1px solid {t['primary']};
        color: {t['primary']};
        border-radius: 9px;
        padding: 3px 8px;
        font-family: "Cascadia Code", "Microsoft YaHei UI";
        font-size: 11px;
        font-weight: 700;
    }}
    QPushButton.DirBtn[neg="true"] {{
        background-color: #45290A;
        border: 1px solid {t['warn']};
        color: {t['warn']};
        border-radius: 9px;
        padding: 3px 8px;
        font-family: "Cascadia Code", "Microsoft YaHei UI";
        font-size: 11px;
        font-weight: 700;
    }}

    QPushButton.WinBtn {{
        background: transparent;
        border: none;
        border-radius: 10px;
        color: {t['text_secondary']};
        font-size: 13px;
        font-weight: 600;
        padding: 0px;
    }}
    QPushButton.WinBtn:hover {{
        background-color: {t['surface_high']};
        color: {t['text_primary']};
    }}
    QPushButton#CloseBtn:hover {{
        background-color: #DC2626;
        color: #FFFFFF;
    }}

    QPushButton#ArmBtn {{
        background-color: {t['primary_container']};
        border: 1px solid {t['primary']};
        color: {t['primary']};
        border-radius: 13px;
        padding: 8px 12px;
        font-size: 12px;
        font-weight: 700;
    }}
    QPushButton#ArmBtn:hover {{
        background-color: {t['primary']};
        color: {t['on_primary']};
    }}
    QPushButton#ArmBtn[armed="true"] {{
        background-color: #45290A;
        border: 1px solid {t['warn']};
        color: {t['warn']};
    }}

    QPushButton#EStopBtn {{
        background-color: {t['danger_bg']};
        border: 1px solid {t['danger']};
        color: #FFFFFF;
        border-radius: 13px;
        padding: 8px 14px;
        font-size: 12px;
        font-weight: 700;
    }}
    QPushButton#EStopBtn:hover {{
        background-color: {t['danger_hover']};
    }}

    QSlider::groove:horizontal {{
        border: none;
        height: 6px;
        background: {t['surface_high']};
        border-radius: 3px;
    }}
    QSlider::handle:horizontal {{
        background: {t['primary']};
        border: 2px solid {t['bg_root']};
        width: 15px;
        height: 15px;
        margin: -5px 0;
        border-radius: 8px;
    }}

    QTextEdit#LogConsole {{
        background-color: {t['surface_inset']};
        color: {t['text_primary']};
        border: 1px solid {t['outline_soft']};
        border-radius: 12px;
        font-family: "Cascadia Code", "Consolas", monospace;
        font-size: 11px;
        padding: 6px;
    }}

    QCheckBox {{
        color: {t['text_secondary']};
        spacing: 6px;
        font-size: 11px;
        font-weight: 600;
    }}
    QCheckBox::indicator {{
        width: 15px;
        height: 15px;
        border-radius: 4px;
        border: 1px solid {t['outline']};
        background-color: {t['surface_inset']};
    }}
    QCheckBox::indicator:checked {{
        background-color: {t['primary']};
        border-color: {t['primary']};
    }}

    QPushButton.MotorSelBtn {{
        background-color: {t['surface_inset']};
        color: {t['text_secondary']};
        border: 1px solid {t['outline']};
        border-radius: 11px;
        padding: 4px 6px;
        font-family: "Cascadia Code", "Microsoft YaHei UI", sans-serif;
        font-size: 11px;
        font-weight: 700;
    }}
    QPushButton.MotorSelBtn:hover {{
        border-color: {t['primary']};
        color: {t['text_primary']};
    }}
    QPushButton.MotorSelBtn[selected="true"] {{
        background-color: {t['primary_container']};
        border: 1.5px solid {t['primary']};
        color: {t['primary']};
    }}

    QPushButton.SmallPillBtn {{
        background-color: {t['surface_inset']};
        color: {t['text_secondary']};
        border: 1px solid {t['outline']};
        border-radius: 8px;
        padding: 2px 4px;
        font-family: "Cascadia Code", "Microsoft YaHei UI", sans-serif;
        font-size: 10px;
        font-weight: 700;
    }}
    QPushButton.SmallPillBtn:hover {{
        background-color: {t['primary_container']};
        border-color: {t['primary']};
        color: {t['text_primary']};
    }}

    QTableWidget {{
        background-color: {t['surface_inset']};
        color: {t['text_primary']};
        border: 1px solid {t['outline_soft']};
        border-radius: 10px;
        gridline-color: {t['outline_soft']};
        font-family: "Cascadia Code", "Consolas", monospace;
        font-size: 11px;
        selection-background-color: {t['primary_container']};
        selection-color: {t['text_primary']};
    }}
    QHeaderView::section {{
        background-color: {t['surface_high']};
        color: {t['text_secondary']};
        border: none;
        border-bottom: 1px solid {t['outline']};
        padding: 5px 4px;
        font-family: "Microsoft YaHei UI", "Segoe UI", sans-serif;
        font-size: 11px;
        font-weight: 700;
    }}
    QScrollBar:vertical {{
        background: {t['surface_inset']};
        width: 8px;
        border-radius: 4px;
    }}
    QScrollBar::handle:vertical {{
        background: {t['outline']};
        min-height: 24px;
        border-radius: 4px;
    }}
    QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{
        height: 0px;
    }}
    """


# =============================================================================
# 矢量 ROV 图标
# =============================================================================
def create_rov_pixmap(theme: dict, size: int = 64) -> QPixmap:
    pm = QPixmap(size, size)
    pm.fill(Qt.transparent)
    p = QPainter(pm)
    p.setRenderHint(QPainter.Antialiasing, True)

    s = float(size)
    m = s * 0.04
    rect = QRectF(m, m, s - 2 * m, s - 2 * m)

    grad = QLinearGradient(0, 0, s, s)
    grad.setColorAt(0.0, QColor(theme["icon_g1"]))
    grad.setColorAt(1.0, QColor(theme["icon_g2"]))
    p.setPen(Qt.NoPen)
    p.setBrush(grad)
    p.drawRoundedRect(rect, s * 0.28, s * 0.28)

    p.setPen(QPen(QColor(255, 255, 255, 55), max(1.0, s * 0.025)))
    p.setBrush(Qt.NoBrush)
    p.drawRoundedRect(rect.adjusted(s * 0.03, s * 0.03, -s * 0.03, -s * 0.03), s * 0.25, s * 0.25)

    cx, cy = s / 2.0, s / 2.0
    p.setPen(QPen(QColor(255, 255, 255, 165), max(1.2, s * 0.035), Qt.SolidLine, Qt.RoundCap))
    p.drawArc(QRectF(cx - s * 0.22, cy - s * 0.36, s * 0.44, s * 0.28), 35 * 16, 110 * 16)

    hull = QRectF(cx - s * 0.22, cy - s * 0.14, s * 0.44, s * 0.32)
    p.setPen(QPen(QColor("#FFFFFF"), max(1.5, s * 0.045)))
    p.setBrush(QColor(255, 255, 255, 38))
    p.drawRoundedRect(hull, s * 0.09, s * 0.09)

    p.setPen(Qt.NoPen)
    p.setBrush(QColor("#FFFFFF"))
    p.drawEllipse(QPointF(cx, cy + s * 0.02), s * 0.075, s * 0.075)
    p.setBrush(QColor(theme["icon_g1"]))
    p.drawEllipse(QPointF(cx, cy + s * 0.02), s * 0.032, s * 0.032)

    p.setPen(QPen(QColor("#FFFFFF"), max(1.5, s * 0.05), Qt.SolidLine, Qt.RoundCap))
    for dx, dy in ((-0.26, -0.15), (0.26, -0.15), (-0.26, 0.18), (0.26, 0.18)):
        p.drawLine(
            QPointF(cx + dx * s * 0.85, cy + dy * s * 0.85),
            QPointF(cx + dx * s * 1.15, cy + dy * s * 1.15),
        )
    p.end()
    return pm


# =============================================================================
# 自定义高颜值控件
# =============================================================================
class StatusChip(QLabel):
    def __init__(self, text: str, color: str):
        super().__init__(text)
        self.setAlignment(Qt.AlignCenter)
        self.set_state(text, color)

    def set_state(self, text: str, color: str):
        self.setText(f"● {text}")
        c = QColor(color)
        bg = f"rgba({c.red()}, {c.green()}, {c.blue()}, 30)"
        bd = f"rgba({c.red()}, {c.green()}, {c.blue()}, 115)"
        self.setStyleSheet(
            f"background-color: {bg}; color: {color}; border: 1px solid {bd};"
            "border-radius: 11px; padding: 3px 7px; font-size: 11px; font-weight: 700;"
        )


class AttitudeHUD(QWidget):
    """人工地平仪 (Roll + Pitch) + 顶部动态航向罗盘条 (Yaw)。"""

    def __init__(self, theme: dict, parent=None):
        super().__init__(parent)
        self.setMinimumSize(260, 250)
        self._theme = theme
        self._roll = self._pitch = self._yaw = 0.0
        self._target_roll = self._target_pitch = self._target_yaw = 0.0
        self._imu_ok = False

        self._anim = QTimer(self)
        self._anim.timeout.connect(self._step_anim)
        self._anim.start(20)

    def set_theme(self, theme: dict):
        self._theme = theme
        self.update()

    def set_attitude(self, roll: float, pitch: float, yaw: float, ok: bool):
        self._target_roll = roll
        self._target_pitch = pitch
        self._target_yaw = yaw
        self._imu_ok = ok

    def _step_anim(self):
        alpha = 0.28
        changed = False
        if abs(self._target_roll - self._roll) > 0.02:
            self._roll += (self._target_roll - self._roll) * alpha
            changed = True
        if abs(self._target_pitch - self._pitch) > 0.02:
            self._pitch += (self._target_pitch - self._pitch) * alpha
            changed = True
        if abs(self._target_yaw - self._yaw) > 0.02:
            self._yaw += (self._target_yaw - self._yaw) * alpha
            changed = True
        if changed:
            self.update()

    def paintEvent(self, event):  # noqa: N802
        t = self._theme
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, True)
        p.setRenderHint(QPainter.TextAntialiasing, True)

        w, h = self.width(), self.height()
        tape_h = 32
        self._draw_heading_tape(p, QRectF(4, 2, w - 8, tape_h), t)

        avail_h = h - tape_h - 12
        radius = max(56.0, min((w - 18) / 2.0, avail_h / 2.0))
        cx = w / 2.0
        cy = tape_h + 8 + avail_h / 2.0
        self._draw_horizon(p, cx, cy, radius, t)
        p.end()

    def _draw_heading_tape(self, p: QPainter, rect: QRectF, t: dict):
        p.save()
        p.setPen(QPen(QColor(t["outline_soft"]), 1))
        p.setBrush(QColor(t["surface_inset"]))
        p.drawRoundedRect(rect, 10, 10)

        clip = QPainterPath()
        clip.addRoundedRect(rect.adjusted(2, 1, -2, -1), 9, 9)
        p.setClipPath(clip)

        cx = rect.center().x()
        px_per_deg = 2.5
        yaw = self._yaw % 360.0
        cardinals = {0: "N", 45: "NE", 90: "E", 135: "SE", 180: "S", 225: "SW", 270: "W", 315: "NW"}
        font_sm = QFont("Cascadia Code", 8)
        font_card = QFont("Cascadia Code", 9, QFont.Bold)

        start_deg = int(math.floor(yaw - rect.width() / (2 * px_per_deg)))
        end_deg = int(math.ceil(yaw + rect.width() / (2 * px_per_deg)))

        for d in range(start_deg - (start_deg % 5), end_deg + 5, 5):
            x = cx + (d - yaw) * px_per_deg
            norm = d % 360
            if norm % 15 == 0:
                p.setPen(QPen(QColor(t["text_muted"]), 1.2))
                p.drawLine(QPointF(x, rect.bottom() - 8), QPointF(x, rect.bottom() - 2))
                label = cardinals.get(norm, f"{norm:03d}")
                if norm in (0, 90, 180, 270):
                    p.setFont(font_card)
                    p.setPen(QColor(t["primary"]))
                else:
                    p.setFont(font_sm)
                    p.setPen(QColor(t["text_secondary"]))
                p.drawText(QRectF(x - 18, rect.top() + 2, 36, 15), Qt.AlignCenter, label)
            else:
                p.setPen(QPen(QColor(t["outline"]), 1))
                p.drawLine(QPointF(x, rect.bottom() - 5), QPointF(x, rect.bottom() - 2))

        box = QRectF(cx - 28, rect.top() + 3, 56, 19)
        p.setPen(QPen(QColor(t["primary"]), 1.3))
        p.setBrush(QColor(t["bg_root"]))
        p.drawRoundedRect(box, 6, 6)
        p.setFont(QFont("Cascadia Code", 9, QFont.Bold))
        p.setPen(QColor(t["text_primary"]))
        p.drawText(box, Qt.AlignCenter, f"{yaw:05.1f}°")
        p.restore()

    def _draw_horizon(self, p: QPainter, cx: float, cy: float, r: float, t: dict):
        p.save()
        accent = QColor(t["primary"])

        glow = QRadialGradient(cx, cy, r + 6)
        glow.setColorAt(0.88, QColor(t["surface_inset"]))
        glow.setColorAt(1.0, QColor(accent.red(), accent.green(), accent.blue(), 48 if self._imu_ok else 16))
        p.setPen(Qt.NoPen)
        p.setBrush(glow)
        p.drawEllipse(QPointF(cx, cy), r + 5, r + 5)

        clip = QPainterPath()
        clip.addEllipse(QPointF(cx, cy), r, r)
        p.setClipPath(clip)

        p.save()
        p.translate(cx, cy)
        p.rotate(-self._roll)
        px_per_deg = r / 42.0
        p.translate(0, max(-55.0, min(55.0, self._pitch)) * px_per_deg)

        big = r * 3.0
        sky_grad = QLinearGradient(0, -big, 0, 0)
        sky_grad.setColorAt(0.0, QColor(t["sky_top"]))
        sky_grad.setColorAt(1.0, QColor(t["sky_bot"]))
        p.fillRect(QRectF(-big, -big, big * 2, big), sky_grad)

        sea_grad = QLinearGradient(0, 0, 0, big)
        sea_grad.setColorAt(0.0, QColor(t["sea_top"]))
        sea_grad.setColorAt(1.0, QColor(t["sea_bot"]))
        p.fillRect(QRectF(-big, 0, big * 2, big), sea_grad)

        p.setPen(QPen(accent, 2.0))
        p.drawLine(QPointF(-big, 0), QPointF(big, 0))

        p.setFont(QFont("Cascadia Code", 8))
        for deg in (-30, -20, -10, 10, 20, 30):
            y = -deg * px_per_deg
            half_w = r * (0.28 if deg % 20 == 0 else 0.16)
            p.setPen(QPen(QColor(240, 248, 255, 190), 1.2))
            p.drawLine(QPointF(-half_w, y), QPointF(half_w, y))
            txt = f"{deg:+d}"
            p.drawText(QRectF(half_w + 4, y - 7, 28, 14), Qt.AlignLeft | Qt.AlignVCenter, txt)
            p.drawText(QRectF(-half_w - 32, y - 7, 28, 14), Qt.AlignRight | Qt.AlignVCenter, txt)
        p.restore()

        p.save()
        p.translate(cx, cy)
        arc_r = r - 7
        p.setPen(QPen(QColor(255, 255, 255, 90), 1.2))
        for ang in (-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60):
            rad = math.radians(ang - 90)
            tick = 7 if ang % 30 == 0 else 4
            p.drawLine(
                QPointF(math.cos(rad) * (arc_r - tick), math.sin(rad) * (arc_r - tick)),
                QPointF(math.cos(rad) * arc_r, math.sin(rad) * arc_r),
            )
        p.rotate(-self._roll)
        ptr = QPolygonF([
            QPointF(0, -(arc_r - 8)),
            QPointF(-5, -(arc_r - 15)),
            QPointF(5, -(arc_r - 15)),
        ])
        p.setPen(Qt.NoPen)
        p.setBrush(QColor(t["warn"]))
        p.drawPolygon(ptr)
        p.restore()

        p.setPen(QPen(accent, 2.4, Qt.SolidLine, Qt.RoundCap))
        p.drawLine(QPointF(cx - 32, cy), QPointF(cx - 11, cy))
        p.drawLine(QPointF(cx + 11, cy), QPointF(cx + 32, cy))
        p.drawLine(QPointF(cx - 11, cy), QPointF(cx - 11, cy + 6))
        p.drawLine(QPointF(cx + 11, cy), QPointF(cx + 11, cy + 6))
        p.setBrush(accent)
        p.drawEllipse(QPointF(cx, cy), 2.8, 2.8)

        if not self._imu_ok:
            badge = QRectF(cx - 46, cy + r * 0.46, 92, 19)
            p.setPen(Qt.NoPen)
            p.setBrush(QColor(11, 17, 26, 185))
            p.drawRoundedRect(badge, 9, 9)
            p.setPen(QColor(t["text_muted"]))
            p.setFont(QFont("Microsoft YaHei UI", 8, QFont.Bold))
            p.drawText(badge, Qt.AlignCenter, "等待 IMU 数据")

        p.restore()
        p.setPen(QPen(QColor(t["primary"] if self._imu_ok else t["outline"]), 2.0))
        p.setBrush(Qt.NoBrush)
        p.drawEllipse(QPointF(cx, cy), r, r)


class RovTopView(QWidget):
    """对标 FinsROV 实车俯视结构: 四角外置 ±45° 水平推进器 + 两侧内置 4 垂直推进器。"""

    def __init__(self, theme: dict, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(176)
        self._theme = theme
        self._motor_us = [1500] * 8
        self._slot_map = [0, 2, 1, 3, 4, 6, 5, 7]  # [H_FL, H_RL, H_FR, H_RR, V_FL, V_RL, V_FR, V_RR]
        self._armed = False
        self._mode_name = "MANUAL"

    def set_theme(self, theme: dict):
        self._theme = theme
        self.update()

    def set_slot_map(self, slot_map: list[int]):
        if len(slot_map) == 8:
            self._slot_map = list(slot_map)
            self.update()

    def set_motors(self, motor_us: list[int], armed: bool, mode_name: str = ""):
        self._motor_us = list(motor_us[:8])
        self._armed = armed
        if mode_name:
            self._mode_name = mode_name
        self.update()

    def paintEvent(self, event):  # noqa: N802
        t = self._theme
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, True)
        p.setRenderHint(QPainter.TextAntialiasing, True)

        w, h = self.width(), self.height()
        cx, cy = w / 2.0, h / 2.0 + 5

        p.setPen(Qt.NoPen)
        p.setBrush(QColor(t["outline_soft"]))
        for x in range(16, w - 10, 22):
            for y in range(12, h - 10, 22):
                p.drawEllipse(QPointF(x, y), 1.1, 1.1)

        # FinsROV 圆柱耐压主舱 + 两侧侧翼框架
        frame_w, frame_h = min(230.0, w * 0.58), min(126.0, h * 0.70)
        frame = QRectF(cx - frame_w / 2, cy - frame_h / 2, frame_w, frame_h)
        cabin_w = frame_w * 0.44
        cabin = QRectF(cx - cabin_w / 2, frame.top() + 4, cabin_w, frame_h - 8)

        # 前视透明半球罩
        dome = QRectF(cx - cabin_w * 0.36, cabin.top() - 12, cabin_w * 0.72, 24)
        p.setPen(QPen(QColor(t["primary"]), 1.4))
        p.setBrush(QColor(t["primary_container"]))
        p.drawChord(dome, 0, 180 * 16)

        p.setPen(QColor(t["text_muted"]))
        p.setFont(QFont("Cascadia Code", 8, QFont.Bold))
        p.drawText(QRectF(cx - 40, dome.top() - 15, 80, 14), Qt.AlignCenter, "▲ FWD")

        # 外框架与中央耐压舱
        p.setPen(QPen(QColor(t["warn"] if self._armed else t["outline"]), 1.6))
        p.setBrush(QColor(t["surface_inset"]))
        p.drawRoundedRect(frame, 18, 18)

        p.setPen(QPen(QColor(t["outline"]), 1.4))
        p.setBrush(QColor(t["surface"]))
        p.drawRoundedRect(cabin, 12, 12)

        p.setPen(QColor(t["warn"] if self._armed else t["text_secondary"]))
        p.setFont(QFont("Cascadia Code", 8, QFont.Bold))
        p.drawText(cabin, Qt.AlignCenter, self._mode_name if self._armed else "SAFE")

        # 8 个逻辑槽位几何位置 (与 FinsROV 实车俯视图完全一致):
        # 0..3: [H_FL, H_RL, H_FR, H_RR] 四角 ±45° 水平推进器
        # 4..7: [V_FL, V_RL, V_FR, V_RR] 舱体左右两侧中段 4 个圆形垂直推进器
        slots = [
            (frame.left() + 4, frame.top() + 6, -45, False),      # 0: H_FL
            (frame.left() + 4, frame.bottom() - 6, 45, False),    # 1: H_RL
            (frame.right() - 4, frame.top() + 6, 45, False),      # 2: H_FR
            (frame.right() - 4, frame.bottom() - 6, -45, False),  # 3: H_RR
            (cabin.left() - 22, cy - frame_h * 0.20, 0, True),    # 4: V_FL
            (cabin.left() - 22, cy + frame_h * 0.20, 0, True),    # 5: V_RL
            (cabin.right() + 22, cy - frame_h * 0.20, 0, True),   # 6: V_FR
            (cabin.right() + 22, cy + frame_h * 0.20, 0, True),   # 7: V_RR
        ]

        prim = QColor(t["primary"])
        warn = QColor(t["warn"])
        for slot_i, (mx, my, angle, is_vert) in enumerate(slots):
            ch = self._slot_map[slot_i] & 0x07
            us = self._motor_us[ch] if ch < len(self._motor_us) else 1500
            delta = (us - 1500) / 500.0
            mag = min(1.0, abs(delta))

            if mag < 0.04:
                color = QColor(t["outline"])
                fill = QColor(t["surface_high"])
            elif delta > 0:
                color = prim
                fill = QColor(prim.red(), prim.green(), prim.blue(), int(45 + 150 * mag))
            else:
                color = warn
                fill = QColor(warn.red(), warn.green(), warn.blue(), int(45 + 150 * mag))

            p.save()
            p.translate(mx, my)
            if is_vert:
                p.setPen(QPen(color, 1.8))
                p.setBrush(fill)
                p.drawEllipse(QPointF(0, 0), 13.5, 13.5)
            else:
                p.rotate(angle)
                p.setPen(QPen(color, 1.8))
                p.setBrush(fill)
                p.drawRoundedRect(QRectF(-15, -9, 30, 18), 6, 6)
            p.restore()

            p.setFont(QFont("Cascadia Code", 8, QFont.Bold))
            p.setPen(QColor(t["text_primary"] if mag >= 0.04 else t["text_secondary"]))
            p.drawText(QRectF(mx - 15, my - 8, 30, 16), Qt.AlignCenter, f"M{ch + 1}")

        p.end()


class PidScopeWidget(QWidget):
    """实时双轨迹阶跃响应示波器 (目标值 vs 测量值 + PID 输出曲线)。"""

    def __init__(self, theme: dict, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(210)
        self._theme = theme
        self._max_pts = 220
        self._channel = "roll"  # roll | pitch | yaw | depth
        self._paused = False
        self._hist = {
            "roll": deque(maxlen=self._max_pts),
            "pitch": deque(maxlen=self._max_pts),
            "yaw": deque(maxlen=self._max_pts),
            "depth": deque(maxlen=self._max_pts),
        }

    def set_theme(self, theme: dict):
        self._theme = theme
        self.update()

    def set_channel(self, ch: str):
        if ch in self._hist:
            self._channel = ch
            self.update()

    def set_paused(self, paused: bool):
        self._paused = paused

    def clear_data(self):
        for dq in self._hist.values():
            dq.clear()
        self.update()

    def push_sample(self, roll_pair, pitch_pair, yaw_pair, depth_pair):
        """每项为 (target, actual, pid_out_pct)。"""
        if self._paused:
            return
        self._hist["roll"].append(roll_pair)
        self._hist["pitch"].append(pitch_pair)
        self._hist["yaw"].append(yaw_pair)
        self._hist["depth"].append(depth_pair)
        if self.isVisible():
            self.update()

    def paintEvent(self, event):  # noqa: N802
        t = self._theme
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, True)
        p.setRenderHint(QPainter.TextAntialiasing, True)

        w, h = self.width(), self.height()
        plot = QRectF(46, 26, max(40, w - 58), max(40, h - 42))

        p.setPen(QPen(QColor(t["outline_soft"]), 1))
        p.setBrush(QColor(t["surface_inset"]))
        p.drawRoundedRect(plot, 10, 10)

        data = list(self._hist[self._channel])
        units = {"roll": "°", "pitch": "°", "yaw": "°", "depth": "m"}[self._channel]
        default_span = 25.0 if self._channel != "depth" else 1.0

        if data:
            vals = [x[0] for x in data] + [x[1] for x in data]
            vmin, vmax = min(vals), max(vals)
            mid = 0.5 * (vmin + vmax)
            half = max(default_span * 0.5, (vmax - vmin) * 0.62)
            ylo, yhi = mid - half, mid + half
        else:
            ylo, yhi = -default_span * 0.5, default_span * 0.5

        # 水平参考网格线
        p.setFont(QFont("Cascadia Code", 8))
        for i in range(5):
            frac = i / 4.0
            y = plot.top() + frac * plot.height()
            val = yhi - frac * (yhi - ylo)
            p.setPen(QPen(QColor(t["outline_soft"]), 1, Qt.DotLine))
            p.drawLine(QPointF(plot.left(), y), QPointF(plot.right(), y))
            p.setPen(QColor(t["text_muted"]))
            fmt = f"{val:+.2f}" if self._channel == "depth" else f"{val:+.0f}"
            p.drawText(QRectF(2, y - 7, 41, 14), Qt.AlignRight | Qt.AlignVCenter, fmt)

        if len(data) >= 2:
            clip = QPainterPath()
            clip.addRoundedRect(plot.adjusted(1, 1, -1, -1), 9, 9)
            p.save()
            p.setClipPath(clip)

            def map_pt(idx_pt, val_y):
                x = plot.left() + (idx_pt / max(1, self._max_pts - 1)) * plot.width()
                norm = (val_y - ylo) / max(1e-6, yhi - ylo)
                y = plot.bottom() - norm * plot.height()
                return QPointF(x, y)

            # 1) PID 输出百分比微弱曲线 (-100..+100 映射到上下边界)
            out_path = QPainterPath()
            for i, (_, _, out_pct) in enumerate(data):
                x = plot.left() + (i / max(1, self._max_pts - 1)) * plot.width()
                y = plot.center().y() - (max(-100.0, min(100.0, out_pct)) / 100.0) * (plot.height() * 0.45)
                if i == 0:
                    out_path.moveTo(x, y)
                else:
                    out_path.lineTo(x, y)
            sec = QColor(t["secondary"])
            sec.setAlpha(120)
            p.setPen(QPen(sec, 1.3))
            p.drawPath(out_path)

            # 2) 目标值虚线
            tgt_path = QPainterPath()
            for i, (tgt_v, _, _) in enumerate(data):
                pt = map_pt(i, tgt_v)
                if i == 0:
                    tgt_path.moveTo(pt)
                else:
                    tgt_path.lineTo(pt)
            p.setPen(QPen(QColor(t["warn"]), 1.7, Qt.DashLine))
            p.drawPath(tgt_path)

            # 3) 实际反馈实线
            act_path = QPainterPath()
            for i, (_, act_v, _) in enumerate(data):
                pt = map_pt(i, act_v)
                if i == 0:
                    act_path.moveTo(pt)
                else:
                    act_path.lineTo(pt)
            p.setPen(QPen(QColor(t["primary"]), 2.2))
            p.drawPath(act_path)
            p.restore()

            tgt_now, act_now, out_now = data[-1]
            err_now = tgt_now - act_now
            legend = (
                f"── 实际: {act_now:+.2f}{units}    "
                f"-- 目标: {tgt_now:+.2f}{units}    "
                f"误差: {err_now:+.2f}{units}    "
                f"PID输出: {out_now:+.1f}%"
            )
        else:
            legend = "连接串口后自动绘制实时目标/反馈/PID输出波形"

        p.setFont(QFont("Cascadia Code", 9, QFont.Bold))
        p.setPen(QColor(t["text_secondary"]))
        p.drawText(QRectF(plot.left(), 4, plot.width(), 18), Qt.AlignLeft | Qt.AlignVCenter, legend)
        p.end()


class DualStickCanvas(QWidget):
    """反锯齿双摇杆雷达控件。"""

    def __init__(self, theme: dict, parent=None):
        super().__init__(parent)
        self.setMinimumSize(230, 120)
        self._theme = theme
        self._lx = self._ly = self._rx = self._ry = None

    def set_theme(self, theme: dict):
        self._theme = theme
        self.update()

    def set_sticks(self, lx, ly, rx, ry):
        self._lx, self._ly, self._rx, self._ry = lx, ly, rx, ry
        self.update()

    def paintEvent(self, event):  # noqa: N802
        t = self._theme
        prim = QColor(t["primary"])
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, True)
        p.setRenderHint(QPainter.TextAntialiasing, True)

        w, h = self.width(), self.height()
        r = min(w / 4.8, (h - 26) / 2.0)
        cy = (h - 14) / 2.0
        centers = [(w * 0.27, "左摇杆 (平移)", self._lx, self._ly), (w * 0.73, "右摇杆 (转向/升降)", self._rx, self._ry)]

        for cx, label, x, y in centers:
            p.setPen(QPen(QColor(t["outline"]), 1.4))
            p.setBrush(QColor(t["surface_inset"]))
            p.drawEllipse(QPointF(cx, cy), r, r)

            p.setPen(QPen(QColor(t["outline_soft"]), 1, Qt.DashLine))
            p.drawEllipse(QPointF(cx, cy), r * 0.5, r * 0.5)
            p.setPen(QPen(QColor(t["outline_soft"]), 1))
            p.drawLine(QPointF(cx - r, cy), QPointF(cx + r, cy))
            p.drawLine(QPointF(cx, cy - r), QPointF(cx, cy + r))

            if x is not None and y is not None:
                px = cx + max(-1.0, min(1.0, x)) * (r - 5)
                py = cy + max(-1.0, min(1.0, y)) * (r - 5)
                p.setPen(QPen(QColor(prim.red(), prim.green(), prim.blue(), 145), 2.0))
                p.drawLine(QPointF(cx, cy), QPointF(px, py))

                glow = QRadialGradient(px, py, 12)
                glow.setColorAt(0.0, QColor(prim.red(), prim.green(), prim.blue(), 220))
                glow.setColorAt(1.0, QColor(prim.red(), prim.green(), prim.blue(), 0))
                p.setPen(Qt.NoPen)
                p.setBrush(glow)
                p.drawEllipse(QPointF(px, py), 12, 12)

                p.setPen(QPen(QColor("#FFFFFF"), 1.5))
                p.setBrush(prim)
                p.drawEllipse(QPointF(px, py), 5.0, 5.0)

            p.setPen(QColor(t["text_secondary"]))
            p.setFont(QFont("Microsoft YaHei UI", 8))
            p.drawText(QRectF(cx - 56, cy + r + 3, 112, 15), Qt.AlignCenter, label)
        p.end()


class MotorCard(QFrame):
    """单个推进器直通调试与脉宽回显卡片。"""

    valueChanged = Signal(int, int)

    def __init__(self, idx: int, name: str, theme: dict, parent=None):
        super().__init__(parent)
        self.idx = idx
        self._theme = theme
        self._last_pct = 0
        self._last_us = 1500
        self.setProperty("class", "M3Tile")

        lay = QVBoxLayout(self)
        lay.setContentsMargins(10, 7, 10, 7)
        lay.setSpacing(4)

        top = QHBoxLayout()
        self.name_lbl = QLabel(name)
        self.pct_lbl = QLabel("0%")
        top.addWidget(self.name_lbl)
        top.addStretch()
        top.addWidget(self.pct_lbl)
        lay.addLayout(top)

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(-100, 100)
        self.slider.setValue(0)
        self.slider.valueChanged.connect(self._on_changed)
        lay.addWidget(self.slider)

        bot = QHBoxLayout()
        self.us_lbl = QLabel("1500 µs")
        self.zero_btn = QPushButton("归零")
        self.zero_btn.setFixedHeight(20)
        self.zero_btn.setStyleSheet("font-size: 10px; padding: 1px 7px; border-radius: 7px;")
        self.zero_btn.clicked.connect(lambda: self.slider.setValue(0))
        bot.addWidget(self.us_lbl)
        bot.addStretch()
        bot.addWidget(self.zero_btn)
        lay.addLayout(bot)

        self.apply_theme(theme)

    def apply_theme(self, theme: dict):
        self._theme = theme
        self.name_lbl.setStyleSheet(f"color: {theme['text_secondary']}; font-size: 11px; font-weight: 700;")
        self._refresh_pct_style(self._last_pct)
        self.set_feedback_us(self._last_us)

    def _refresh_pct_style(self, v: int):
        t = self._theme
        color = t["primary"] if v > 0 else (t["warn"] if v < 0 else t["text_secondary"])
        self.pct_lbl.setText(f"{v:+d}%" if v != 0 else "0%")
        self.pct_lbl.setStyleSheet(
            f"font-family: 'Cascadia Code','Consolas',monospace; color: {color}; font-size: 11px; font-weight: 700;"
        )

    def _on_changed(self, v: int):
        self._last_pct = v
        self._refresh_pct_style(v)
        self.valueChanged.emit(self.idx, v)

    def set_slider_silent(self, v: int):
        self._last_pct = v
        self.slider.blockSignals(True)
        self.slider.setValue(v)
        self.slider.blockSignals(False)
        self._refresh_pct_style(v)

    def set_feedback_us(self, us: int):
        self._last_us = us
        t = self._theme
        color = t["primary"] if us > 1520 else (t["warn"] if us < 1480 else t["text_muted"])
        self.us_lbl.setText(f"{us} µs")
        self.us_lbl.setStyleSheet(
            f"font-family: 'Cascadia Code','Consolas',monospace; color: {color}; font-size: 10px;"
        )


class DraggableAppBar(QFrame):
    def __init__(self, win: QMainWindow, parent=None):
        super().__init__(parent)
        self._win = win
        self._drag_offset = None
        self.setObjectName("AppBar")
        self.setFixedHeight(60)

    def mousePressEvent(self, event):  # noqa: N802
        if event.button() == Qt.LeftButton:
            handle = self._win.windowHandle()
            if handle is not None and handle.startSystemMove():
                event.accept()
                return
            self._drag_offset = event.globalPosition().toPoint() - self._win.frameGeometry().topLeft()
            event.accept()
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):  # noqa: N802
        if self._drag_offset is not None and (event.buttons() & Qt.LeftButton):
            self._win.move(event.globalPosition().toPoint() - self._drag_offset)
            event.accept()
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):  # noqa: N802
        self._drag_offset = None
        super().mouseReleaseEvent(event)

    def mouseDoubleClickEvent(self, event):  # noqa: N802
        if event.button() == Qt.LeftButton:
            self._win._toggle_max()
            event.accept()
        super().mouseDoubleClickEvent(event)


class SerialBridge(QObject):
    connect_ok = Signal(str, int, object)
    connect_failed = Signal(str, str)
    status_received = Signal(tuple)
    ack_received = Signal(int, int)
    params_received = Signal(int, bytes)
    disconnected = Signal(str)


# =============================================================================
# 主窗口
# =============================================================================
class LuoRovWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("LuoRov · 八推进器 ROV 地面站")
        self.setWindowFlags(
            Qt.FramelessWindowHint
            | Qt.WindowSystemMenuHint
            | Qt.WindowMinimizeButtonHint
            | Qt.WindowMaximizeButtonHint
        )
        self.setAttribute(Qt.WA_TranslucentBackground, True)
        self.resize(1360, 860)
        self.setMinimumSize(1180, 750)
        self.setFocusPolicy(Qt.StrongFocus)

        self.theme_key = "blue"
        self.theme = THEMES[self.theme_key]

        self.ser = None
        self.parser = FrameParser()
        self.rx_thread = None
        self.tx_thread = None
        self.tx_q: queue.Queue[bytes] = queue.Queue(maxsize=32)
        self.running = False
        self.connecting = False
        self.armed = False
        self.last_motor_send = [0.0] * 8

        # 多模式控制状态
        self.ctrl_mode = 0              # 0..4
        self.rc_drive_enabled = True    # True=手柄/键盘混控遥控, False=单电机滑块直通
        self.speed_scale = 0.6          # 默认 60% 遥控限速档位
        self.keys_held: set[int] = set()
        self.gp_axes_norm = (0.0, 0.0, 0.0, 0.0)  # lx, ly, rx, ry
        self.step_roll_deg = 0.0
        self.step_pitch_deg = 0.0
        self.cur_depth_m = 0.0
        self.cur_yaw_deg = 0.0

        self.bridge = SerialBridge()
        self.bridge.connect_ok.connect(self._on_connect_ok)
        self.bridge.connect_failed.connect(self._on_connect_failed)
        self.bridge.status_received.connect(self._update_status)
        self.bridge.ack_received.connect(self._on_ack)
        self.bridge.params_received.connect(self._on_params_pack)
        self.bridge.disconnected.connect(self._on_rx_disconnected)

        self.face_map = dict(DEFAULT_FACE)
        self.calib = None
        self.gamepad_reader = None
        self._face_active = {k: False for k in ("A", "B", "X", "Y")}

        self._build_ui()
        self.apply_theme("blue")
        self._refresh_ports()

        if GamepadReader is not None:
            self.gamepad_reader = GamepadReader()
            self.gamepad_reader.start()
        else:
            self.gp_status_lbl.setText("手柄不可用 (未安装 pygame，仍可用键盘 WASD/QE 操控)")

        # 周期任务: 100ms 遥测轮询 + 40ms(25Hz) 遥控与手柄刷新
        self.ping_timer = QTimer(self)
        self.ping_timer.timeout.connect(self._send_ping)
        self.ping_timer.start(100)

        self.ctrl_timer = QTimer(self)
        self.ctrl_timer.timeout.connect(self._tick_control_and_gamepad)
        self.ctrl_timer.start(40)

        QShortcut(QKeySequence(Qt.Key_Escape), self, activated=self._stop)
        self._log("地面站已就绪：支持纯手动 / 自稳 / 定深 / 定航 / 全悬停 及在线 PID 调参", "info")

    # ---------------- 键盘操控事件 (WASD + QE + Space/C) ----------------
    def keyPressEvent(self, event):  # noqa: N802
        if not event.isAutoRepeat():
            self.keys_held.add(event.key())
        super().keyPressEvent(event)

    def keyReleaseEvent(self, event):  # noqa: N802
        if not event.isAutoRepeat():
            self.keys_held.discard(event.key())
        super().keyReleaseEvent(event)

    # ---------------- UI 构建 ----------------
    def _build_ui(self):
        outer = QWidget()
        outer_lay = QVBoxLayout(outer)
        outer_lay.setContentsMargins(6, 6, 6, 6)
        self.setCentralWidget(outer)

        self.shell = QFrame()
        self.shell.setObjectName("WindowShell")
        outer_lay.addWidget(self.shell)

        root = QVBoxLayout(self.shell)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        root.addWidget(self._build_appbar())

        self.pages = QStackedWidget()
        self.pages.addWidget(self._build_cockpit_page())
        self.pages.addWidget(self._build_tuning_page())
        self.motor_test_page = MotorTestPage(self.theme)
        self.motor_test_page.motorCommandRequested.connect(self._on_motor_test_cmd)
        self.motor_test_page.allMotorsZeroRequested.connect(self._on_motor_test_all_zero)
        self.motor_test_page.armToggleRequested.connect(self._toggle_arm)
        self.motor_test_page.estopRequested.connect(self._stop)
        self.motor_test_page.logMessage.connect(self._log)
        self.pages.addWidget(self.motor_test_page)
        root.addWidget(self.pages, 1)

        foot = QHBoxLayout()
        foot.setContentsMargins(16, 0, 6, 4)
        self.foot_lbl = QLabel(
            "快捷键: WASD 平移 · Q/E 左右旋转 · Space/C 上浮下潜 · Esc 紧急停桨"
        )
        self.foot_lbl.setProperty("class", "Muted")
        foot.addWidget(self.foot_lbl)
        foot.addStretch()
        grip = QSizeGrip(self.shell)
        grip.setFixedSize(14, 14)
        foot.addWidget(grip, 0, Qt.AlignBottom | Qt.AlignRight)
        root.addLayout(foot)

    def _build_appbar(self) -> QFrame:
        bar = DraggableAppBar(self)
        lay = QHBoxLayout(bar)
        lay.setContentsMargins(12, 8, 10, 8)
        lay.setSpacing(6)

        self.logo_lbl = QLabel()
        self.logo_lbl.setFixedSize(34, 34)
        lay.addWidget(self.logo_lbl)

        brand_box = QVBoxLayout()
        brand_box.setSpacing(0)
        self.brand_title = QLabel("LuoRov")
        self.brand_sub = QLabel("FinsROV 8推进器地面站")
        brand_box.addWidget(self.brand_title)
        brand_box.addWidget(self.brand_sub)
        lay.addLayout(brand_box)

        # 页面导航分段胶囊 (驾驶舱 / 调参台 / 电机测试)
        nav_seg = QFrame()
        nav_seg.setProperty("class", "SegmentBox")
        nav_lay = QHBoxLayout(nav_seg)
        nav_lay.setContentsMargins(3, 3, 3, 3)
        nav_lay.setSpacing(2)

        self.nav_cockpit_btn = QPushButton("🎮 驾驶舱")
        self.nav_cockpit_btn.setProperty("class", "SegBtn")
        self.nav_cockpit_btn.setProperty("active", "true")
        self.nav_cockpit_btn.setCursor(Qt.PointingHandCursor)
        self.nav_cockpit_btn.clicked.connect(lambda: self._switch_page(0))

        self.nav_tune_btn = QPushButton("🎛️ 调参台")
        self.nav_tune_btn.setProperty("class", "SegBtn")
        self.nav_tune_btn.setProperty("active", "false")
        self.nav_tune_btn.setCursor(Qt.PointingHandCursor)
        self.nav_tune_btn.clicked.connect(lambda: self._switch_page(1))

        self.nav_motor_btn = QPushButton("⚡ 电机测试")
        self.nav_motor_btn.setProperty("class", "SegBtn")
        self.nav_motor_btn.setProperty("active", "false")
        self.nav_motor_btn.setCursor(Qt.PointingHandCursor)
        self.nav_motor_btn.clicked.connect(lambda: self._switch_page(2))

        nav_lay.addWidget(self.nav_cockpit_btn)
        nav_lay.addWidget(self.nav_tune_btn)
        nav_lay.addWidget(self.nav_motor_btn)
        lay.addWidget(nav_seg)

        # 状态胶囊组
        self.pill_link = StatusChip("未连接", self.theme["text_muted"])
        self.pill_mode = StatusChip("纯手动", self.theme["primary"])
        self.pill_imu = StatusChip("IMU --", self.theme["text_muted"])
        self.pill_depth = StatusChip("深度 --", self.theme["text_muted"])
        self.pill_arm = StatusChip("未解锁", self.theme["text_secondary"])
        self.uptime_lbl = QLabel("0s")
        self.uptime_lbl.setProperty("class", "Muted")

        for w in (self.pill_link, self.pill_mode, self.pill_imu, self.pill_depth, self.pill_arm, self.uptime_lbl):
            lay.addWidget(w)

        lay.addStretch()

        # 蓝 / 绿 主题切换器
        theme_seg = QFrame()
        theme_seg.setProperty("class", "SegmentBox")
        t_lay = QHBoxLayout(theme_seg)
        t_lay.setContentsMargins(3, 3, 3, 3)
        t_lay.setSpacing(2)

        self.btn_blue = QPushButton("蓝")
        self.btn_blue.setProperty("class", "SegBtn")
        self.btn_blue.setToolTip("深海蓝主题")
        self.btn_blue.setCursor(Qt.PointingHandCursor)
        self.btn_blue.clicked.connect(lambda: self.apply_theme("blue"))

        self.btn_green = QPushButton("绿")
        self.btn_green.setProperty("class", "SegBtn")
        self.btn_green.setToolTip("翡翠绿主题")
        self.btn_green.setCursor(Qt.PointingHandCursor)
        self.btn_green.clicked.connect(lambda: self.apply_theme("green"))

        t_lay.addWidget(self.btn_blue)
        t_lay.addWidget(self.btn_green)
        lay.addWidget(theme_seg)

        # 串口连接
        self.port_cb = QComboBox()
        self.port_cb.setMinimumWidth(84)
        self.port_cb.setMaximumWidth(150)
        self.baud_cb = QComboBox()
        self.baud_cb.addItems(["115200", "57600", "9600"])
        self.baud_cb.setCurrentText("115200")
        self.baud_cb.setFixedWidth(86)

        self.refresh_btn = QPushButton("刷新")
        self.refresh_btn.clicked.connect(self._refresh_ports)

        self.connect_btn = QPushButton("连接")
        self.connect_btn.setObjectName("ConnectBtn")
        self.connect_btn.clicked.connect(self._toggle_connect)

        for w in (self.port_cb, self.baud_cb, self.refresh_btn, self.connect_btn):
            lay.addWidget(w)

        self.top_div = QFrame()
        self.top_div.setFixedSize(1, 22)
        lay.addWidget(self.top_div)

        min_btn = QPushButton("─")
        min_btn.setProperty("class", "WinBtn")
        min_btn.setFixedSize(28, 28)
        min_btn.clicked.connect(self.showMinimized)

        self.max_btn = QPushButton("▢")
        self.max_btn.setProperty("class", "WinBtn")
        self.max_btn.setFixedSize(28, 28)
        self.max_btn.clicked.connect(self._toggle_max)

        close_btn = QPushButton("✕")
        close_btn.setObjectName("CloseBtn")
        close_btn.setProperty("class", "WinBtn")
        close_btn.setFixedSize(28, 28)
        close_btn.clicked.connect(self.close)

        for w in (min_btn, self.max_btn, close_btn):
            lay.addWidget(w)

        return bar

    def _switch_page(self, idx: int):
        prev = self.pages.currentIndex()
        if prev == 2 and idx != 2:
            self.motor_test_page.on_page_leave()
        self.pages.setCurrentIndex(idx)
        self.nav_cockpit_btn.setProperty("active", "true" if idx == 0 else "false")
        self.nav_tune_btn.setProperty("active", "true" if idx == 1 else "false")
        self.nav_motor_btn.setProperty("active", "true" if idx == 2 else "false")
        for b in (self.nav_cockpit_btn, self.nav_tune_btn, self.nav_motor_btn):
            b.style().unpolish(b)
            b.style().polish(b)
        if idx == 1 and self.ser and self.ser.is_open:
            self._request_all_params()
        elif idx == 2:
            self._set_drive_source(False)

    def _toggle_max(self):
        if self.isMaximized():
            self.showNormal()
            self.max_btn.setText("▢")
        else:
            self.showMaximized()
            self.max_btn.setText("❐")

    def _make_card(self, title: str, sub: str, right_widget: QWidget | None = None):
        card = QFrame()
        card.setProperty("class", "M3Card")
        lay = QVBoxLayout(card)
        lay.setContentsMargins(14, 12, 14, 12)
        lay.setSpacing(9)

        hdr = QHBoxLayout()
        tbox = QVBoxLayout()
        tbox.setSpacing(1)
        t_lbl = QLabel(title)
        t_lbl.setProperty("class", "CardTitle")
        s_lbl = QLabel(sub)
        s_lbl.setProperty("class", "CardSub")
        tbox.addWidget(t_lbl)
        tbox.addWidget(s_lbl)
        hdr.addLayout(tbox)
        hdr.addStretch()
        if right_widget is not None:
            hdr.addWidget(right_widget)
        lay.addLayout(hdr)
        return card, lay

    def _make_stat_tile(self, title: str, init_val: str):
        tile = QFrame()
        tile.setProperty("class", "M3Tile")
        lay = QVBoxLayout(tile)
        lay.setContentsMargins(10, 7, 10, 7)
        lay.setSpacing(2)
        lbl_t = QLabel(title)
        lbl_t.setProperty("class", "Muted")
        lbl_v = QLabel(init_val)
        lbl_v.setProperty("class", "MonoValue")
        lay.addWidget(lbl_t)
        lay.addWidget(lbl_v)
        return tile, lbl_v

    # =========================================================================
    # 页面 0: 遥控驾驶舱 (Flight Cockpit)
    # =========================================================================
    def _build_cockpit_page(self) -> QWidget:
        page = QWidget()
        lay = QHBoxLayout(page)
        lay.setContentsMargins(14, 12, 14, 10)
        lay.setSpacing(12)

        lay.addWidget(self._build_left_telemetry_col(), 32)
        lay.addWidget(self._build_center_control_col(), 42)
        lay.addWidget(self._build_right_gamepad_col(), 26)
        return page

    def _build_left_telemetry_col(self) -> QWidget:
        col = QWidget()
        lay = QVBoxLayout(col)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(12)

        cal_imu_btn = QPushButton("🧭 水平/零偏校准")
        cal_imu_btn.setFixedHeight(26)
        cal_imu_btn.setToolTip("上锁静止状态下点击，校准陀螺仪零偏并将当前姿态设为水平 0°")
        cal_imu_btn.clicked.connect(self._calibrate_imu)

        imu_card, imu_lay = self._make_card("姿态与陀螺仪", "ATTITUDE · 200HZ AHRS", cal_imu_btn)
        self.hud = AttitudeHUD(self.theme)
        imu_lay.addWidget(self.hud, 1)

        euler_row = QHBoxLayout()
        euler_row.setSpacing(8)
        t_r, self.roll_lbl = self._make_stat_tile("横滚 ROLL", "--.-°")
        t_p, self.pitch_lbl = self._make_stat_tile("俯仰 PITCH", "--.-°")
        t_y, self.yaw_lbl = self._make_stat_tile("偏航 YAW", "--.-°")
        euler_row.addWidget(t_r)
        euler_row.addWidget(t_p)
        euler_row.addWidget(t_y)
        imu_lay.addLayout(euler_row)

        vec_frame = QFrame()
        vec_frame.setProperty("class", "M3Tile")
        vec_grid = QGridLayout(vec_frame)
        vec_grid.setContentsMargins(10, 7, 10, 7)
        vec_grid.setHorizontalSpacing(10)
        vec_grid.setVerticalSpacing(3)

        lbl_g = QLabel("角速度 (dps)")
        lbl_g.setProperty("class", "Muted")
        self.gyro_lbl = QLabel("X:  --.-   Y:  --.-   Z:  --.-")
        self.gyro_lbl.setProperty("class", "MonoValue")

        lbl_a = QLabel("加速度 (g)")
        lbl_a.setProperty("class", "Muted")
        self.acc_lbl = QLabel("X: -.---   Y: -.---   Z: -.---")
        self.acc_lbl.setProperty("class", "MonoValue")

        vec_grid.addWidget(lbl_g, 0, 0)
        vec_grid.addWidget(self.gyro_lbl, 0, 1)
        vec_grid.addWidget(lbl_a, 1, 0)
        vec_grid.addWidget(self.acc_lbl, 1, 1)
        imu_lay.addWidget(vec_frame)
        lay.addWidget(imu_card, 1)

        zero_btn = QPushButton("水面置零")
        zero_btn.setFixedHeight(26)
        zero_btn.clicked.connect(self._zero_depth)
        dep_card, dep_lay = self._make_card("深度与环境", "DEPTH & PRESSURE", zero_btn)
        dep_card.setFixedHeight(118)

        dep_row = QHBoxLayout()
        dep_row.setSpacing(8)
        td, self.depth_lbl = self._make_stat_tile("实际深度", "--.-- m")
        ttgt, self.tgt_depth_lbl = self._make_stat_tile("目标深度", "0.00 m")
        tp, self.press_lbl = self._make_stat_tile("绝对压力", "--.- kPa")
        dep_row.addWidget(td)
        dep_row.addWidget(ttgt)
        dep_row.addWidget(tp)
        dep_lay.addLayout(dep_row)
        lay.addWidget(dep_card, 0)

        return col

    def _build_center_control_col(self) -> QWidget:
        col = QWidget()
        lay = QVBoxLayout(col)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(12)

        # 1) 模式选择 + 推进器拓扑 + 解锁急停
        self.mode_desc_lbl = QLabel(MODE_INFO[0][2])
        self.mode_desc_lbl.setProperty("class", "Muted")
        rov_card, rov_lay = self._make_card("运动模式与推进器分配", "FLIGHT MODES · FINSROV MIXER", self.mode_desc_lbl)

        # 5 模式胶囊切换栏
        mode_seg = QFrame()
        mode_seg.setProperty("class", "SegmentBox")
        m_seg_lay = QHBoxLayout(mode_seg)
        m_seg_lay.setContentsMargins(3, 3, 3, 3)
        m_seg_lay.setSpacing(2)
        self.mode_btns: list[QPushButton] = []
        for m_id, m_name, m_desc in MODE_INFO:
            b = QPushButton(m_name)
            b.setProperty("class", "SegBtn")
            b.setProperty("active", "true" if m_id == 0 else "false")
            b.setToolTip(m_desc)
            b.setCursor(Qt.PointingHandCursor)
            b.clicked.connect(lambda _=False, mid=m_id: self._select_mode(mid))
            m_seg_lay.addWidget(b, 1)
            self.mode_btns.append(b)
        rov_lay.addWidget(mode_seg)

        # 定深/定航目标快捷设定条
        tgt_bar = QHBoxLayout()
        tgt_bar.setSpacing(6)
        lbl_td = QLabel("设定水深(m):")
        lbl_td.setProperty("class", "Muted")
        self.sp_target_depth = QDoubleSpinBox()
        self.sp_target_depth.setRange(0.0, 20.0)
        self.sp_target_depth.setSingleStep(0.1)
        self.sp_target_depth.setDecimals(2)
        self.sp_target_depth.setFixedWidth(74)

        lbl_ty = QLabel("目标航向(°):")
        lbl_ty.setProperty("class", "Muted")
        self.sp_target_yaw = QDoubleSpinBox()
        self.sp_target_yaw.setRange(-180.0, 180.0)
        self.sp_target_yaw.setSingleStep(5.0)
        self.sp_target_yaw.setDecimals(1)
        self.sp_target_yaw.setFixedWidth(74)

        btn_apply_tgt = QPushButton("发送目标")
        btn_apply_tgt.setFixedHeight(26)
        btn_apply_tgt.clicked.connect(self._send_targets_cmd)

        btn_latch = QPushButton("锁定当前深/航")
        btn_latch.setFixedHeight(26)
        btn_latch.clicked.connect(self._latch_current_targets)

        for w in (lbl_td, self.sp_target_depth, lbl_ty, self.sp_target_yaw, btn_apply_tgt, btn_latch):
            tgt_bar.addWidget(w)
        rov_lay.addLayout(tgt_bar)

        self.rov_view = RovTopView(self.theme)
        rov_lay.addWidget(self.rov_view)

        btn_row = QHBoxLayout()
        btn_row.setSpacing(8)
        self.arm_btn = QPushButton("🔓 解锁 (ARM)")
        self.arm_btn.setObjectName("ArmBtn")
        self.arm_btn.clicked.connect(self._toggle_arm)

        self.all_stop_btn = QPushButton("全部回中 (1500µs)")
        self.all_stop_btn.setMinimumHeight(36)
        self.all_stop_btn.clicked.connect(self._all_stop)

        self.stop_btn = QPushButton("🛑 急停 (Esc)")
        self.stop_btn.setObjectName("EStopBtn")
        self.stop_btn.clicked.connect(self._stop)

        btn_row.addWidget(self.arm_btn, 2)
        btn_row.addWidget(self.all_stop_btn, 2)
        btn_row.addWidget(self.stop_btn, 2)
        rov_lay.addLayout(btn_row)
        lay.addWidget(rov_card, 0)

        # 2) 下部: 输入源切换 (手柄/键盘混控 vs 单电机直通调试) + 8 电机实时脉宽卡片
        src_seg = QFrame()
        src_seg.setProperty("class", "SegmentBox")
        src_lay = QHBoxLayout(src_seg)
        src_lay.setContentsMargins(2, 2, 2, 2)
        src_lay.setSpacing(2)

        self.btn_src_rc = QPushButton("🕹️ 手柄/键盘遥控")
        self.btn_src_rc.setProperty("class", "SegBtn")
        self.btn_src_rc.setProperty("active", "true")
        self.btn_src_rc.clicked.connect(lambda: self._set_drive_source(True))

        self.btn_src_direct = QPushButton("🔧 单电机直通点动")
        self.btn_src_direct.setProperty("class", "SegBtn")
        self.btn_src_direct.setProperty("active", "false")
        self.btn_src_direct.clicked.connect(lambda: self._set_drive_source(False))

        src_lay.addWidget(self.btn_src_rc)
        src_lay.addWidget(self.btn_src_direct)

        m_card, m_lay = self._make_card("推进器输出与点动调试", "THRUSTERS · PWM FEEDBACK", src_seg)

        grid = QGridLayout()
        grid.setSpacing(8)
        self.motor_cards: list[MotorCard] = []
        for i in range(8):
            mc = MotorCard(i, MOTOR_NAMES[i], self.theme)
            mc.valueChanged.connect(self._on_motor_slider)
            grid.addWidget(mc, i // 2, i % 2)
            self.motor_cards.append(mc)
        m_lay.addLayout(grid, 1)
        lay.addWidget(m_card, 1)

        return col

    def _build_right_gamepad_col(self) -> QWidget:
        col = QWidget()
        lay = QVBoxLayout(col)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(12)

        calib_btn = QPushButton("标定 A/B/X/Y")
        calib_btn.setFixedHeight(26)
        calib_btn.clicked.connect(self._start_calib)
        gp_card, gp_lay = self._make_card("遥控输入与手柄", "RC INPUT · GAMEPAD & KEYBOARD", calib_btn)

        self.gp_status_lbl = QLabel("正在检测手柄…")
        self.gp_status_lbl.setProperty("class", "Muted")
        gp_lay.addWidget(self.gp_status_lbl)

        # 动力限幅档位选择
        spd_row = QHBoxLayout()
        lbl_spd = QLabel("遥控动力档位:")
        lbl_spd.setProperty("class", "Muted")
        self.spd_cb = QComboBox()
        self.spd_cb.addItems(["30% 精细慢速", "60% 标准巡航", "85% 机动增强", "100% 全功率"])
        self.spd_cb.setCurrentIndex(1)
        self.spd_cb.currentIndexChanged.connect(
            lambda idx: setattr(self, "speed_scale", [0.30, 0.60, 0.85, 1.00][idx])
        )
        spd_row.addWidget(lbl_spd)
        spd_row.addWidget(self.spd_cb, 1)
        gp_lay.addLayout(spd_row)

        self.stick_canvas = DualStickCanvas(self.theme)
        gp_lay.addWidget(self.stick_canvas)

        self.rc_cmd_lbl = QLabel("发送指令: Vx:   0  Vy:   0  Vz:   0  Yaw:   0")
        self.rc_cmd_lbl.setProperty("class", "MonoValue")
        gp_lay.addWidget(self.rc_cmd_lbl)

        face_row = QHBoxLayout()
        face_row.setSpacing(6)
        lbl_f = QLabel("手柄键:")
        lbl_f.setProperty("class", "Muted")
        face_row.addWidget(lbl_f)
        self.gp_face: dict[str, QLabel] = {}
        for key in ("A", "B", "X", "Y"):
            lbl = QLabel(key)
            lbl.setAlignment(Qt.AlignCenter)
            lbl.setFixedSize(28, 28)
            face_row.addWidget(lbl)
            self.gp_face[key] = lbl
        face_row.addStretch()
        gp_lay.addLayout(face_row)

        self.gp_extra_lbl = QLabel("键盘支持: W/S 前后 · A/D 横移 · Q/E 转向 · Space/C 升降")
        self.gp_extra_lbl.setProperty("class", "Muted")
        gp_lay.addWidget(self.gp_extra_lbl)
        lay.addWidget(gp_card, 3)

        clear_btn = QPushButton("清空")
        clear_btn.setFixedHeight(24)
        log_card, log_lay = self._make_card("通信与状态日志", "SYSTEM CONSOLE", clear_btn)
        self.log_console = QTextEdit()
        self.log_console.setObjectName("LogConsole")
        self.log_console.setReadOnly(True)
        clear_btn.clicked.connect(self.log_console.clear)
        log_lay.addWidget(self.log_console)
        lay.addWidget(log_card, 2)

        return col

    # =========================================================================
    # 页面 1: PID 调参与推力分配工作台 (PID & Mixer Studio)
    # =========================================================================
    def _build_tuning_page(self) -> QWidget:
        page = QWidget()
        lay = QHBoxLayout(page)
        lay.setContentsMargins(14, 12, 14, 10)
        lay.setSpacing(12)

        # 左半部 (64%): 上方实时响应示波器 + 下方 7 路串级 PID 参数整定表
        left = QWidget()
        left_lay = QVBoxLayout(left)
        left_lay.setContentsMargins(0, 0, 0, 0)
        left_lay.setSpacing(12)

        # 1) 实时波形示波器卡片
        scope_seg = QFrame()
        scope_seg.setProperty("class", "SegmentBox")
        sc_lay = QHBoxLayout(scope_seg)
        sc_lay.setContentsMargins(2, 2, 2, 2)
        sc_lay.setSpacing(2)
        self.scope_btns: dict[str, QPushButton] = {}
        for ch_key, ch_title in (("roll", "横滚 Roll"), ("pitch", "俯仰 Pitch"), ("yaw", "偏航 Yaw"), ("depth", "深度 Depth")):
            b = QPushButton(ch_title)
            b.setProperty("class", "SegBtn")
            b.setProperty("active", "true" if ch_key == "roll" else "false")
            b.clicked.connect(lambda _=False, ck=ch_key: self._select_scope_ch(ck))
            sc_lay.addWidget(b)
            self.scope_btns[ch_key] = b

        scope_card, scope_card_lay = self._make_card(
            "闭环阶跃响应示波器", "LIVE OSCILLOSCOPE · TARGET VS ACTUAL", scope_seg
        )
        self.pid_scope = PidScopeWidget(self.theme)
        scope_card_lay.addWidget(self.pid_scope, 1)

        # 阶跃激励快捷按钮条
        step_row = QHBoxLayout()
        step_row.setSpacing(6)
        lbl_st = QLabel("阶跃测试激励:")
        lbl_st.setProperty("class", "Muted")
        step_row.addWidget(lbl_st)

        for txt, fn in (
            ("Roll +10°", lambda: self._set_step_angle(10.0, 0.0, "roll")),
            ("Roll -10°", lambda: self._set_step_angle(-10.0, 0.0, "roll")),
            ("Pitch +10°", lambda: self._set_step_angle(0.0, 10.0, "pitch")),
            ("姿态归 0°", lambda: self._set_step_angle(0.0, 0.0, None)),
            ("深度 +0.2m", lambda: self._step_depth(+0.20)),
            ("深度 -0.2m", lambda: self._step_depth(-0.20)),
        ):
            b = QPushButton(txt)
            b.setFixedHeight(25)
            b.clicked.connect(fn)
            step_row.addWidget(b)

        step_row.addStretch()
        self.pause_scope_btn = QPushButton("暂停波形")
        self.pause_scope_btn.setFixedHeight(25)
        self.pause_scope_btn.clicked.connect(self._toggle_scope_pause)
        clr_scope_btn = QPushButton("清空波形")
        clr_scope_btn.setFixedHeight(25)
        clr_scope_btn.clicked.connect(lambda: self.pid_scope.clear_data())
        step_row.addWidget(self.pause_scope_btn)
        step_row.addWidget(clr_scope_btn)
        scope_card_lay.addLayout(step_row)

        left_lay.addWidget(scope_card, 4)

        # 2) 7 路串级 PID 参数整定卡片
        pid_card, pid_lay = self._make_card(
            "7 路串级 PID 参数整定", "CASCADE PID · DEPTH & ROLL/PITCH/YAW DUAL-LOOP"
        )

        pgrid = QGridLayout()
        pgrid.setHorizontalSpacing(8)
        pgrid.setVerticalSpacing(6)

        headers = ["控制环 (外环角度 → 内环角速度)", "Kp", "Ki", "Kd", "积分限幅 I_max", "输出限幅 Out_max", "单路操作"]
        for col_i, h_text in enumerate(headers):
            hl = QLabel(h_text)
            hl.setProperty("class", "Muted")
            pgrid.addWidget(hl, 0, col_i)

        self.pid_spin_rows: list[tuple[QDoubleSpinBox, ...]] = []
        for row_i, (loop_id, cn_name, sub_unit, defaults) in enumerate(PID_LOOPS_META, start=1):
            name_lbl = QLabel(f"{cn_name}  ·  {sub_unit}")
            name_lbl.setStyleSheet("font-weight: 600; font-size: 11px;")
            pgrid.addWidget(name_lbl, row_i, 0)

            spins = []
            specs = [
                (0.0, 200.0, 0.1, 3, defaults[0]),
                (0.0, 50.0, 0.01, 3, defaults[1]),
                (0.0, 100.0, 0.01, 3, defaults[2]),
                (0.0, 1000.0, 5.0, 1, defaults[3]),
                (0.0, 1000.0, 10.0, 1, defaults[4]),
            ]
            for c_idx, (lo, hi, step, dec, dval) in enumerate(specs, start=1):
                sp = QDoubleSpinBox()
                sp.setRange(lo, hi)
                sp.setSingleStep(step)
                sp.setDecimals(dec)
                sp.setValue(dval)
                pgrid.addWidget(sp, row_i, c_idx)
                spins.append(sp)

            self.pid_spin_rows.append(tuple(spins))
            apply_one = QPushButton("下发")
            apply_one.setFixedHeight(25)
            apply_one.clicked.connect(lambda _=False, lid=loop_id: self._send_single_pid(lid))
            pgrid.addWidget(apply_one, row_i, 6)

        pid_lay.addLayout(pgrid)

        act_row = QHBoxLayout()
        act_row.setSpacing(8)

        btn_read = QPushButton("📥 从 STM32 读取当前参数")
        btn_read.clicked.connect(self._request_all_params)

        btn_send_all = QPushButton("📤 一键下发全部 PID")
        btn_send_all.setProperty("class", "FilledBtn")
        btn_send_all.clicked.connect(self._send_all_pids)

        btn_flash = QPushButton("💾 固化保存到 STM32 Flash")
        btn_flash.setToolTip("需先上锁停桨，保存后断电重启仍生效")
        btn_flash.clicked.connect(self._save_params_to_flash)

        btn_reset_def = QPushButton("🔄 恢复默认参数")
        btn_reset_def.clicked.connect(self._restore_factory_params)

        for b in (btn_read, btn_send_all, btn_flash, btn_reset_def):
            act_row.addWidget(b)
        pid_lay.addLayout(act_row)

        left_lay.addWidget(pid_card, 5)
        lay.addWidget(left, 64)

        # 右半部 (36%): FinsROV 8推进器通道映射、正反桨极性与功率补偿
        right_card, r_lay = self._make_card(
            "8 推进器分配映射与正反桨", "THRUSTER ALLOCATION · FINSROV V33 / LUOROV"
        )

        # 一键预设按钮
        pre_row = QHBoxLayout()
        pre_row.setSpacing(6)
        btn_pre_fins = QPushButton("⚡ 载入 FinsROV V33 原版预设")
        btn_pre_fins.setToolTip("OutID={0,3,7,4}, InID={1,2,6,5}, Sign={+1,-1,+1,+1,-1,-1,+1,-1}")
        btn_pre_fins.clicked.connect(self._apply_preset_finsrov)

        btn_pre_luo = QPushButton("⚡ 载入 LuoRov 顺序预设")
        btn_pre_luo.setToolTip("外圈水平 M1~M4, 内圈垂直 M5~M8, 全部正桨 +1")
        btn_pre_luo.clicked.connect(self._apply_preset_luorov)

        pre_row.addWidget(btn_pre_fins)
        pre_row.addWidget(btn_pre_luo)
        r_lay.addLayout(pre_row)

        # 8 逻辑安装位 -> 物理电调通道映射
        map_tile = QFrame()
        map_tile.setProperty("class", "M3Tile")
        map_grid = QGridLayout(map_tile)
        map_grid.setContentsMargins(10, 8, 10, 8)
        map_grid.setHorizontalSpacing(10)
        map_grid.setVerticalSpacing(5)

        default_map = [0, 2, 1, 3, 4, 6, 5, 7]
        self.slot_combos: list[QComboBox] = []
        for s_idx, s_label in enumerate(SLOT_LABELS):
            row_r = s_idx % 4
            col_base = 0 if s_idx < 4 else 2
            lbl = QLabel(s_label)
            lbl.setProperty("class", "Muted")
            cb = QComboBox()
            cb.addItems([f"M{k + 1} (CH{k})" for k in range(8)])
            cb.setCurrentIndex(default_map[s_idx])
            map_grid.addWidget(lbl, row_r, col_base)
            map_grid.addWidget(cb, row_r, col_base + 1)
            self.slot_combos.append(cb)
        r_lay.addWidget(map_tile)

        # 8 物理电机 M1..M8 正反桨方向切换
        dir_hdr = QLabel("物理电机正反桨方向 (点击切换 正桨+1 / 反桨-1):")
        dir_hdr.setProperty("class", "Muted")
        r_lay.addWidget(dir_hdr)

        dir_tile = QFrame()
        dir_tile.setProperty("class", "M3Tile")
        dir_grid = QGridLayout(dir_tile)
        dir_grid.setContentsMargins(10, 8, 10, 8)
        dir_grid.setSpacing(6)

        self.dir_btns: list[QPushButton] = []
        self.motor_dirs = [1] * 8
        for m_idx in range(8):
            b = QPushButton(f"M{m_idx + 1}: 正桨 (+1)")
            b.setProperty("class", "DirBtn")
            b.setProperty("neg", "false")
            b.setCursor(Qt.PointingHandCursor)
            b.clicked.connect(lambda _=False, mi=m_idx: self._toggle_motor_dir(mi))
            dir_grid.addWidget(b, m_idx // 2, m_idx % 2)
            self.dir_btns.append(b)
        r_lay.addWidget(dir_tile)

        # 死区补偿 / 浮力前馈 / 总功率限幅
        comp_tile = QFrame()
        comp_tile.setProperty("class", "M3Tile")
        comp_grid = QGridLayout(comp_tile)
        comp_grid.setContentsMargins(10, 8, 10, 8)
        comp_grid.setSpacing(6)

        l1 = QLabel("电调死区补偿 (0~200):")
        l1.setProperty("class", "Muted")
        self.sp_deadzone = QSpinBox()
        self.sp_deadzone.setRange(0, 200)
        self.sp_deadzone.setValue(40)

        l2 = QLabel("垂直净浮力补偿 (推力 ‰):")
        l2.setProperty("class", "Muted")
        self.sp_hover_trim = QSpinBox()
        self.sp_hover_trim.setRange(-500, 500)
        self.sp_hover_trim.setSuffix(" ‰")
        self.sp_hover_trim.setToolTip("推力千分比 (10‰ = 1% 推力 = 5µs；正浮力填负值向下压水，负浮力填正值)")
        self.sp_hover_trim.setValue(0)

        l3 = QLabel("推进总功率上限 (%):")
        l3.setProperty("class", "Muted")
        self.sp_max_power = QSpinBox()
        self.sp_max_power.setRange(15, 100)
        self.sp_max_power.setSuffix(" %")
        self.sp_max_power.setValue(100)

        comp_grid.addWidget(l1, 0, 0)
        comp_grid.addWidget(self.sp_deadzone, 0, 1)
        comp_grid.addWidget(l2, 1, 0)
        comp_grid.addWidget(self.sp_hover_trim, 1, 1)
        comp_grid.addWidget(l3, 2, 0)
        comp_grid.addWidget(self.sp_max_power, 2, 1)
        r_lay.addWidget(comp_tile)

        r_lay.addStretch()

        btn_apply_mix = QPushButton("📤 下发推进器混控与正反桨配置")
        btn_apply_mix.setProperty("class", "FilledBtn")
        btn_apply_mix.setMinimumHeight(36)
        btn_apply_mix.clicked.connect(self._send_mixer_cfg)
        r_lay.addWidget(btn_apply_mix)

        lay.addWidget(right_card, 36)
        return page

    # =========================================================================
    # 主题切换
    # =========================================================================
    def apply_theme(self, key: str):
        if key not in THEMES:
            return
        self.theme_key = key
        t = THEMES[key]
        self.theme = t

        QApplication.instance().setStyleSheet(build_qss(t))

        icon_pm = create_rov_pixmap(t, 64)
        self.logo_lbl.setPixmap(icon_pm.scaled(34, 34, Qt.KeepAspectRatio, Qt.SmoothTransformation))
        app_icon = QIcon(icon_pm)
        self.setWindowIcon(app_icon)
        QApplication.instance().setWindowIcon(app_icon)

        self.brand_title.setStyleSheet(
            f"font-size: 15px; font-weight: 800; color: {t['primary']}; letter-spacing: 0.6px;"
        )
        self.brand_sub.setStyleSheet(f"font-size: 10px; color: {t['text_muted']};")
        self.top_div.setStyleSheet(f"background-color: {t['outline']};")

        self.btn_blue.setProperty("active", "true" if key == "blue" else "false")
        self.btn_green.setProperty("active", "true" if key == "green" else "false")
        for b in (self.btn_blue, self.btn_green):
            b.style().unpolish(b)
            b.style().polish(b)

        mono_big = "font-family: 'Cascadia Code','Consolas',monospace; font-size: 15px; font-weight: 700;"
        self.roll_lbl.setStyleSheet(f"{mono_big} color: {t['primary']};")
        self.pitch_lbl.setStyleSheet(f"{mono_big} color: {t['tertiary']};")
        self.yaw_lbl.setStyleSheet(f"{mono_big} color: {t['secondary']};")
        self.depth_lbl.setStyleSheet(f"{mono_big} color: {t['primary']};")
        self.tgt_depth_lbl.setStyleSheet(f"{mono_big} color: {t['warn']};")
        self.press_lbl.setStyleSheet(f"{mono_big} color: {t['text_primary']};")

        self.hud.set_theme(t)
        self.rov_view.set_theme(t)
        self.stick_canvas.set_theme(t)
        self.pid_scope.set_theme(t)
        self.motor_test_page.apply_theme(t)
        for mc in self.motor_cards:
            mc.apply_theme(t)
        for k, lbl in self.gp_face.items():
            self._style_face_btn(lbl, self._face_active.get(k, False))

    def _style_face_btn(self, lbl: QLabel, active: bool):
        t = self.theme
        if active:
            lbl.setStyleSheet(
                f"background-color: {t['primary']}; color: {t['on_primary']}; "
                f"border: 1px solid {t['primary_hover']}; border-radius: 14px; font-weight: 800; font-size: 11px;"
            )
        else:
            lbl.setStyleSheet(
                f"background-color: {t['surface_inset']}; color: {t['text_muted']}; "
                f"border: 1px solid {t['outline_soft']}; border-radius: 14px; font-weight: 700; font-size: 11px;"
            )

    def _log(self, msg: str, level: str = "info"):
        t = self.theme
        colors = {"info": t["text_secondary"], "ok": t["ok"], "warn": t["warn"], "err": t["danger"]}
        c = colors.get(level, t["text_secondary"])
        ts = time.strftime("%H:%M:%S")
        self.log_console.append(
            f'<span style="color:{t["text_muted"]};">[{ts}]</span> <span style="color:{c};">{msg}</span>'
        )

    # =========================================================================
    # 串口收发与协议交互 (全异步非阻塞，彻底避免 COM1 / 蓝牙串口卡死界面)
    # =========================================================================
    def _refresh_ports(self):
        cur_dev = self.port_cb.currentData() or self.port_cb.currentText().split(" ")[0].strip()
        raw_ports = list(serial.tools.list_ports.comports())

        # 优先将 USB-TTL / CH340 / CP210x / FTDI / STLink 排在前面，避免默认误选 COM1
        def _port_priority(p):
            desc = f"{p.description} {p.hwid}".upper()
            is_usb = any(k in desc for k in ("USB", "CH340", "CP210", "FTDI", "STLINK", "SERIAL", "UART"))
            is_com1 = p.device.upper() == "COM1"
            return (0 if (is_usb and not is_com1) else (2 if is_com1 else 1), p.device)

        raw_ports.sort(key=_port_priority)
        self.port_cb.clear()
        self.port_cb.view().setMinimumWidth(260)

        target_idx = 0
        for idx, p in enumerate(raw_ports):
            desc = (p.description or "").replace(f"({p.device})", "").strip()
            label = f"{p.device} · {desc}" if desc else p.device
            self.port_cb.addItem(label, userData=p.device)
            if p.device == cur_dev:
                target_idx = idx
        if raw_ports:
            self.port_cb.setCurrentIndex(target_idx)

    def _toggle_connect(self):
        if self.connecting:
            return
        if self.ser and self.ser.is_open:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        port = self.port_cb.currentData() or self.port_cb.currentText().split(" ")[0].strip()
        baud = int(self.baud_cb.currentText())
        if not port:
            QMessageBox.warning(self, "提示", "请先选择一个可用串口 (COM)")
            return

        self.connecting = True
        self.connect_btn.setEnabled(False)
        self.connect_btn.setText("连接中…")
        self.pill_link.set_state(f"连接 {port}…", self.theme["warn"])
        self._log(f"正在打开串口 {port} @ {baud}…", "info")

        # 在后台线程打开串口，避免 COM1 / 蓝牙串口阻塞 Qt 主界面
        threading.Thread(target=self._open_worker, args=(port, baud), daemon=True).start()

    def _open_worker(self, port: str, baud: int):
        try:
            ser = serial.Serial(
                port=port,
                baudrate=baud,
                timeout=0.08,
                write_timeout=0.15,
                rtscts=False,
                dsrdtr=False,
            )
            self.bridge.connect_ok.emit(port, baud, ser)
        except Exception as e:
            self.bridge.connect_failed.emit(port, str(e))

    def _on_connect_ok(self, port: str, baud: int, ser):
        self.connecting = False
        self.connect_btn.setEnabled(True)
        self.ser = ser
        self.parser = FrameParser()
        while not self.tx_q.empty():
            try:
                self.tx_q.get_nowait()
            except queue.Empty:
                break

        self.running = True
        self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.tx_thread = threading.Thread(target=self._tx_loop, daemon=True)
        self.rx_thread.start()
        self.tx_thread.start()

        self.connect_btn.setText("断开")
        self.connect_btn.setProperty("connected", "true")
        self.connect_btn.style().unpolish(self.connect_btn)
        self.connect_btn.style().polish(self.connect_btn)

        self.pill_link.set_state(f"{port}", self.theme["ok"])
        self.motor_test_page.update_telemetry([1500] * 8, self.armed, True)
        self._log(f"已连接 {port} @ {baud}，正在同步 STM32 参数…", "ok")
        QTimer.singleShot(150, self._request_all_params)

    def _on_connect_failed(self, port: str, err: str):
        self.connecting = False
        self.connect_btn.setEnabled(True)
        self.connect_btn.setText("连接")
        self.pill_link.set_state("未连接", self.theme["text_muted"])
        self._log(f"打开 {port} 失败: {err}", "err")
        QMessageBox.critical(self, "串口打开失败", f"无法打开 {port}:\n{err}")

    def _disconnect(self):
        self.running = False
        ser = self.ser
        self.ser = None

        # 立即唤醒阻塞在队列或读写上的后台线程，防止关闭时卡顿
        try:
            self.tx_q.put_nowait(b"")
        except queue.Full:
            pass

        if ser is not None:
            def _close_bg(s):
                for fn in ("cancel_read", "cancel_write", "close"):
                    try:
                        getattr(s, fn)()
                    except Exception:
                        pass
            threading.Thread(target=_close_bg, args=(ser,), daemon=True).start()

        self.rx_thread = None
        self.tx_thread = None

        self.connect_btn.setEnabled(True)
        self.connect_btn.setText("连接")
        self.connect_btn.setProperty("connected", "false")
        self.connect_btn.style().unpolish(self.connect_btn)
        self.connect_btn.style().polish(self.connect_btn)

        self.pill_link.set_state("未连接", self.theme["text_muted"])
        self.pill_imu.set_state("IMU --", self.theme["text_muted"])
        self.pill_depth.set_state("深度 --", self.theme["text_muted"])
        self.hud.set_attitude(0.0, 0.0, 0.0, False)
        self.armed = False
        self.motor_test_page.on_emergency_or_stop()
        self.motor_test_page.update_telemetry([1500] * 8, False, False)
        self._log("串口已断开", "warn")

    def _send(self, data: bytes):
        if not (self.running and self.ser and self.ser.is_open):
            return
        try:
            self.tx_q.put_nowait(data)
        except queue.Full:
            # 队列满时丢弃最旧帧，保证最新控制帧不积压且绝不阻塞 UI
            try:
                self.tx_q.get_nowait()
                self.tx_q.put_nowait(data)
            except Exception:
                pass

    def _tx_loop(self):
        timeout_streak = 0
        while self.running:
            try:
                data = self.tx_q.get(timeout=0.1)
            except queue.Empty:
                continue
            if not self.running or not data:
                continue
            ser = self.ser
            if ser is None or not ser.is_open:
                break
            try:
                ser.write(data)
                timeout_streak = 0
            except serial.SerialTimeoutException:
                timeout_streak += 1
                if timeout_streak >= 6 and self.running:
                    self.bridge.disconnected.emit("串口写入连续超时 (可能选错了端口如 COM1，或硬件无响应)")
                    break
            except Exception as e:
                if self.running:
                    self.bridge.disconnected.emit(f"写入错误: {e}")
                break

    def _rx_loop(self):
        reason = ""
        while self.running:
            ser = self.ser
            if ser is None or not ser.is_open:
                break
            try:
                data = ser.read(256)
                if not data:
                    continue
            except Exception as e:
                reason = str(e)
                break
            for typ, payload in self.parser.feed(data):
                if typ == TELE_STATUS:
                    if len(payload) >= 68:
                        try:
                            t = struct.unpack(TELE_FMT_68, payload[:68])
                            self.bridge.status_received.emit(t)
                        except struct.error:
                            pass
                    elif len(payload) >= 52:
                        try:
                            t52 = struct.unpack(TELE_FMT_52, payload[:52])
                            self.bridge.status_received.emit(t52 + (0, 0, 0, 0, 0, 0, 0, 0))
                        except struct.error:
                            pass
                elif typ == TELE_ACK and len(payload) >= 2:
                    self.bridge.ack_received.emit(payload[0], payload[1])
                elif typ == TELE_PARAMS and len(payload) >= 1:
                    self.bridge.params_received.emit(payload[0], payload[1:])
        if self.running:
            self.bridge.disconnected.emit(reason)

    def _on_rx_disconnected(self, reason: str):
        self._disconnect()
        if reason:
            self._log(f"串口异常断开: {reason}", "err")

    def _on_ack(self, cmd: int, result: int):
        cmd_names = {
            CMD_ARM: "解锁/上锁",
            CMD_STOP: "紧急停止",
            CMD_SET_MOTOR: "单电机点动",
            CMD_SET_ALL: "全电机回中",
            CMD_ZERO_DEPTH: "水面置零",
            CMD_SET_TARGETS: "设定目标值",
            CMD_SET_PID: "PID参数更新",
            CMD_SET_MIXER: "混控分配更新",
            CMD_FLASH_OP: "Flash/校准操作",
        }
        cname = cmd_names.get(cmd, f"0x{cmd:02X}")
        if result == 0:
            self._log(f"{cname} -> 成功", "ok")
        else:
            self._log(f"{cname} -> 被拒绝 (请确认解锁/上锁状态要求)", "warn")

    def _send_ping(self):
        if self.ser and self.ser.is_open:
            self._send(build_frame(CMD_PING))

    # =========================================================================
    # 遥测刷新与示波器推流
    # =========================================================================
    def _update_status(self, t_data):
        (
            uptime, depth_mm, press_pa, temp_c100,
            roll, pitch, yaw, gx, gy, gz, ax, ay, az,
            m0, m1, m2, m3, m4, m5, m6, m7,
            armed, imu_ok, depth_ok, leak,
            fw_mode, sys_flags, tgt_yaw_cdeg, tgt_depth_mm,
            pid_d, pid_r, pid_p, pid_y,
        ) = t_data

        roll_deg = roll / 100.0
        pitch_deg = pitch / 100.0
        yaw_deg = yaw / 100.0
        tgt_yaw_deg = tgt_yaw_cdeg / 100.0
        depth_m = depth_mm / 1000.0
        tgt_depth_m = tgt_depth_mm / 1000.0

        self.cur_depth_m = depth_m
        self.cur_yaw_deg = yaw_deg

        self.hud.set_attitude(roll_deg, pitch_deg, yaw_deg, bool(imu_ok))
        self.roll_lbl.setText(f"{roll_deg:+6.1f}°")
        self.pitch_lbl.setText(f"{pitch_deg:+6.1f}°")
        self.yaw_lbl.setText(f"{yaw_deg % 360.0:05.1f}°")
        self.gyro_lbl.setText(f"X:{gx / 100.0:+6.1f}  Y:{gy / 100.0:+6.1f}  Z:{gz / 100.0:+6.1f}")
        self.acc_lbl.setText(f"X:{ax / 1000.0:+.3f}  Y:{ay / 1000.0:+.3f}  Z:{az / 1000.0:+.3f}")

        self.tgt_depth_lbl.setText(f"{tgt_depth_m:.2f} m")
        if depth_ok:
            self.depth_lbl.setText(f"{depth_m:.2f} m")
            self.press_lbl.setText(f"{press_pa / 1000.0:.2f} kPa")
        else:
            self.depth_lbl.setText("--.-- m")
            self.press_lbl.setText("--.- kPa")

        th = self.theme
        cal_str = "IMU OK·已校" if (imu_ok and (sys_flags & 1)) else ("IMU OK" if imu_ok else "IMU --")
        self.pill_imu.set_state(cal_str, th["ok"] if imu_ok else th["text_muted"])
        self.pill_depth.set_state("深度 OK" if depth_ok else "深度 --", th["ok"] if depth_ok else th["text_muted"])
        self.uptime_lbl.setText(f"{uptime / 1000.0:.0f}s")

        if fw_mode == 0xFF:
            mode_short = "直通点动"
            self.pill_mode.set_state("单电机点动", th["warn"])
        elif 0 <= fw_mode < len(MODE_INFO):
            mode_short = MODE_INFO[fw_mode][1]
            self.pill_mode.set_state(mode_short, th["primary"])
        else:
            mode_short = "MANUAL"

        self.armed = bool(armed)
        if self.armed:
            self.pill_arm.set_state("已解锁", th["warn"])
            self.arm_btn.setText("🔒 上锁 (DISARM)")
            self.arm_btn.setProperty("armed", "true")
        else:
            self.pill_arm.set_state("未解锁", th["text_secondary"])
            self.arm_btn.setText("🔓 解锁 (ARM)")
            self.arm_btn.setProperty("armed", "false")
        self.arm_btn.style().unpolish(self.arm_btn)
        self.arm_btn.style().polish(self.arm_btn)

        motors = [m0, m1, m2, m3, m4, m5, m6, m7]
        self.rov_view.set_motors(motors, self.armed, mode_short)
        for i, us in enumerate(motors):
            self.motor_cards[i].set_feedback_us(us)
        self.motor_test_page.update_telemetry(motors, self.armed, bool(self.ser and self.ser.is_open))

        # 推送到实时调参示波器
        self.pid_scope.push_sample(
            (self.step_roll_deg, roll_deg, pid_r / 10.0),
            (self.step_pitch_deg, pitch_deg, pid_p / 10.0),
            (tgt_yaw_deg, yaw_deg, pid_y / 10.0),
            (tgt_depth_m, depth_m if depth_ok else 0.0, pid_d / 10.0),
        )

    # =========================================================================
    # 模式切换、遥控循环与单电机点动
    # =========================================================================
    def _select_mode(self, mode_id: int):
        self.ctrl_mode = mode_id
        for i, b in enumerate(self.mode_btns):
            b.setProperty("active", "true" if i == mode_id else "false")
            b.style().unpolish(b)
            b.style().polish(b)
        self.mode_desc_lbl.setText(MODE_INFO[mode_id][2])
        # 切换模式时自动进入混控遥控态并通知固件锁存当前基准
        self._set_drive_source(True)
        self._send_control_frame(latch=True)
        self._log(f"切换模式 -> {MODE_INFO[mode_id][1]} ({MODE_INFO[mode_id][2]})", "info")

    def _set_drive_source(self, rc_enabled: bool):
        self.rc_drive_enabled = rc_enabled
        self.btn_src_rc.setProperty("active", "true" if rc_enabled else "false")
        self.btn_src_direct.setProperty("active", "false" if rc_enabled else "true")
        for b in (self.btn_src_rc, self.btn_src_direct):
            b.style().unpolish(b)
            b.style().polish(b)

    def _send_control_frame(self, vx=0, vy=0, vz=0, yaw=0, latch=False):
        flags = 0x01 | (0x02 if latch else 0x00)
        roll_cdeg = int(round(self.step_roll_deg * 100.0))
        pitch_cdeg = int(round(self.step_pitch_deg * 100.0))
        payload = struct.pack(
            "<BBhhhhhh",
            self.ctrl_mode,
            flags,
            int(vx),
            int(vy),
            int(vz),
            int(yaw),
            roll_cdeg,
            pitch_cdeg,
        )
        self._send(build_frame(CMD_SET_CONTROL, payload))

    def _send_targets_cmd(self):
        d_mm = int(round(self.sp_target_depth.value() * 1000.0))
        y_cdeg = int(round(self.sp_target_yaw.value() * 100.0))
        trim = int(self.sp_hover_trim.value())
        payload = struct.pack("<ihh", d_mm, y_cdeg, trim)
        self._send(build_frame(CMD_SET_TARGETS, payload))

    def _latch_current_targets(self):
        self.sp_target_depth.setValue(max(0.0, self.cur_depth_m))
        self.sp_target_yaw.setValue(self.cur_yaw_deg)
        self._send_control_frame(latch=True)
        self._log("已将当前深度与航向锁存为闭环目标值", "ok")

    def _calibrate_imu(self):
        if self.armed:
            QMessageBox.information(self, "安全提示", "请先点击「上锁 (DISARM)」并保持艇体静止后再执行校准。")
            return
        self._send(build_frame(CMD_FLASH_OP, bytes([3])))
        self._log("正在校准陀螺零偏与水平基准，请保持静止约 0.2 秒…", "info")

    def _toggle_arm(self):
        target = 0 if self.armed else 1
        self._send(build_frame(CMD_ARM, bytes([target])))

    def _stop(self):
        self.step_roll_deg = 0.0
        self.step_pitch_deg = 0.0
        self._send(build_frame(CMD_STOP))
        for mc in self.motor_cards:
            mc.set_slider_silent(0)
        self.motor_test_page.on_emergency_or_stop()
        self._log("触发紧急停止 (E-STOP)！", "err")

    def _zero_depth(self):
        self.sp_target_depth.setValue(0.0)
        self._send(build_frame(CMD_ZERO_DEPTH))

    def _all_stop(self):
        self.step_roll_deg = 0.0
        self.step_pitch_deg = 0.0
        if self.rc_drive_enabled:
            self._send_control_frame(0, 0, 0, 0)
        else:
            self._send(build_frame(CMD_SET_ALL, struct.pack("<8h", *([0] * 8))))
        for mc in self.motor_cards:
            mc.set_slider_silent(0)
        self.motor_test_page.on_emergency_or_stop()
        self._log("全部推进器已回中 (1500 µs)", "info")

    def _on_motor_test_cmd(self, idx: int, pct: float):
        if self.rc_drive_enabled:
            self._set_drive_source(False)
        speed = int(round(max(-100.0, min(100.0, pct)) * 10.0))
        self._send(build_frame(CMD_SET_MOTOR, bytes([idx]) + struct.pack("<h", speed)))
        if 0 <= idx < len(self.motor_cards):
            self.motor_cards[idx].set_slider_silent(int(round(pct)))

    def _on_motor_test_all_zero(self):
        self._set_drive_source(False)
        self._send(build_frame(CMD_SET_ALL, struct.pack("<8h", *([0] * 8))))
        for mc in self.motor_cards:
            mc.set_slider_silent(0)
        self._log("电机测试: 全部 8 台推进器已停转 (1500 µs)", "info")

    def _on_motor_slider(self, idx: int, val: int):
        # 拖动单电机滑块时自动切到直通点动模式，防止被 25Hz 混控帧覆盖
        if self.rc_drive_enabled and val != 0:
            self._set_drive_source(False)
        now = time.time()
        if val == 0 or (now - self.last_motor_send[idx] > 0.03):
            self.last_motor_send[idx] = now
            speed = int(val * 10)
            self._send(build_frame(CMD_SET_MOTOR, bytes([idx]) + struct.pack("<h", speed)))

    # =========================================================================
    # 25Hz 周期合成手柄 + 键盘遥控量并下发
    # =========================================================================
    def _tick_control_and_gamepad(self):
        self._poll_gamepad()

        lx, ly, rx, ry = self.gp_axes_norm

        def dz(v, th=0.08):
            return 0.0 if abs(v) < th else (v - math.copysign(th, v)) / (1.0 - th)

        # 手柄分量 (上推摇杆轴值为负，取反为正)
        vx_f = -dz(ly)
        vy_f = dz(lx)
        yaw_f = dz(rx)
        vz_f = -dz(ry)

        # 叠加键盘 WASD + Q/E + Space/C (未聚焦输入框时生效)
        fw = QApplication.focusWidget()
        if not isinstance(fw, (QDoubleSpinBox, QSpinBox, QLineEdit)):
            if Qt.Key_W in self.keys_held:
                vx_f += 1.0
            if Qt.Key_S in self.keys_held:
                vx_f -= 1.0
            if Qt.Key_D in self.keys_held:
                vy_f += 1.0
            if Qt.Key_A in self.keys_held:
                vy_f -= 1.0
            if Qt.Key_E in self.keys_held:
                yaw_f += 1.0
            if Qt.Key_Q in self.keys_held:
                yaw_f -= 1.0
            if Qt.Key_Space in self.keys_held:
                vz_f += 1.0
            if Qt.Key_C in self.keys_held:
                vz_f -= 1.0

        vx = int(max(-1.0, min(1.0, vx_f)) * 1000 * self.speed_scale)
        vy = int(max(-1.0, min(1.0, vy_f)) * 1000 * self.speed_scale)
        vz = int(max(-1.0, min(1.0, vz_f)) * 1000 * self.speed_scale)
        yaw = int(max(-1.0, min(1.0, yaw_f)) * 1000 * self.speed_scale)

        self.rc_cmd_lbl.setText(f"指令: Vx:{vx:+5d}  Vy:{vy:+5d}  Vz:{vz:+5d}  Yaw:{yaw:+5d}")

        if self.rc_drive_enabled and self.ser and self.ser.is_open:
            self._send_control_frame(vx, vy, vz, yaw)

    def _poll_gamepad(self):
        if self.gamepad_reader is None:
            return
        snap = self.gamepad_reader.snapshot()
        if snap is None:
            return
        th = self.theme
        if snap.get("error"):
            self.gp_status_lbl.setText("手柄不可用: " + snap["error"])
            self.gp_axes_norm = (0.0, 0.0, 0.0, 0.0)
            self.stick_canvas.set_sticks(None, None, None, None)
            return
        if not snap.get("connected"):
            self.gp_status_lbl.setText("未接手柄 (可直接用键盘 WASD / QE / Space / C 操控)")
            self.gp_status_lbl.setStyleSheet(f"color: {th['text_muted']}; font-size: 11px;")
            self.gp_axes_norm = (0.0, 0.0, 0.0, 0.0)
            self.stick_canvas.set_sticks(None, None, None, None)
            return

        name = snap.get("name", "")
        axes = snap.get("axes", [])
        buttons = snap.get("buttons", [])

        if self.calib is not None:
            self._calib_tick(buttons)
            return

        self.gp_status_lbl.setText(f"已连接: {name}")
        self.gp_status_lbl.setStyleSheet(f"color: {th['ok']}; font-size: 11px; font-weight: 700;")

        def axis(idx):
            return axes[idx] if idx < len(axes) else 0.0

        lx, ly = axis(AXIS_INDEX["LX"]), axis(AXIS_INDEX["LY"])
        rx, ry = axis(AXIS_INDEX["RX"]), axis(AXIS_INDEX["RY"])
        self.gp_axes_norm = (lx, ly, rx, ry)
        self.stick_canvas.set_sticks(lx, ly, rx, ry)

        for key, idx in self.face_map.items():
            active = idx < len(buttons) and buttons[idx]
            self._face_active[key] = active
            self._style_face_btn(self.gp_face[key], active)

    def _start_calib(self):
        prev = set()
        if self.gamepad_reader is not None:
            snap = self.gamepad_reader.snapshot()
            if snap and snap.get("connected"):
                btns = snap.get("buttons", [])
                prev = {i for i in range(len(btns)) if btns[i]}
        self.calib = {"step": 0, "labels": ["A", "B", "X", "Y"], "map": {}, "prev": prev}
        self.gp_status_lbl.setText("标定中: 请按下 A")
        self.gp_status_lbl.setStyleSheet(f"color: {self.theme['warn']}; font-size: 11px; font-weight: 700;")

    def _calib_tick(self, buttons):
        pressed = {i for i in range(len(buttons)) if buttons[i]}
        rising = pressed - self.calib["prev"]
        self.calib["prev"] = pressed
        for idx in sorted(rising):
            step = self.calib["step"]
            if step >= len(self.calib["labels"]):
                break
            label = self.calib["labels"][step]
            self.calib["map"][label] = idx
            self.calib["step"] = step + 1
            nxt = self.calib["labels"][self.calib["step"]] if self.calib["step"] < len(self.calib["labels"]) else None
            if nxt is None:
                self.face_map = dict(self.calib["map"])
                self.calib = None
                text = "标定完成: " + " ".join(f"{k}={v}" for k, v in sorted(self.face_map.items()))
                self.gp_status_lbl.setText(text)
                self._log(text, "ok")
            else:
                self.gp_status_lbl.setText(f"标定中: 请按下 {nxt}")

    # =========================================================================
    # PID 调参与混控面板逻辑
    # =========================================================================
    def _select_scope_ch(self, ch_key: str):
        self.pid_scope.set_channel(ch_key)
        for k, b in self.scope_btns.items():
            b.setProperty("active", "true" if k == ch_key else "false")
            b.style().unpolish(b)
            b.style().polish(b)

    def _toggle_scope_pause(self):
        paused = not self.pid_scope._paused
        self.pid_scope.set_paused(paused)
        self.pause_scope_btn.setText("继续波形" if paused else "暂停波形")

    def _set_step_angle(self, r_deg: float, p_deg: float, switch_ch: str | None):
        self.step_roll_deg = r_deg
        self.step_pitch_deg = p_deg
        if switch_ch:
            self._select_scope_ch(switch_ch)
        self._set_drive_source(True)
        self._send_control_frame()
        self._log(f"阶跃激励: 目标 Roll={r_deg:+.1f}°, Pitch={p_deg:+.1f}°", "info")

    def _step_depth(self, delta_m: float):
        new_d = max(0.0, self.sp_target_depth.value() + delta_m)
        self.sp_target_depth.setValue(new_d)
        self._select_scope_ch("depth")
        self._send_targets_cmd()

    def _request_all_params(self):
        if not (self.ser and self.ser.is_open):
            return
        self._send(build_frame(CMD_GET_PARAMS, bytes([0])))
        QTimer.singleShot(60, lambda: self._send(build_frame(CMD_GET_PARAMS, bytes([1]))))

    def _on_params_pack(self, part: int, data: bytes):
        if part == 0 and len(data) >= 80:
            for i in range(4):
                vals = struct.unpack("<5f", data[i * 20 : (i + 1) * 20])
                for sp, v in zip(self.pid_spin_rows[i], vals):
                    sp.setValue(v)
        elif part == 1 and len(data) >= 82:
            for idx, loop_i in enumerate(range(4, 7)):
                vals = struct.unpack("<5f", data[idx * 20 : (idx + 1) * 20])
                for sp, v in zip(self.pid_spin_rows[loop_i], vals):
                    sp.setValue(v)
            mix_bytes = data[60:82]
            unpacked = struct.unpack("<8B8bHhH", mix_bytes)
            s_map = list(unpacked[0:8])
            s_dir = list(unpacked[8:16])
            dz_comp, h_trim, max_pm = unpacked[16], unpacked[17], unpacked[18]

            for s_i, ch in enumerate(s_map):
                if 0 <= ch < 8:
                    self.slot_combos[s_i].setCurrentIndex(ch)
            self.rov_view.set_slot_map(s_map)

            for m_i, d in enumerate(s_dir):
                self._set_motor_dir_ui(m_i, -1 if d < 0 else 1)

            self.sp_deadzone.setValue(dz_comp)
            self.sp_hover_trim.setValue(h_trim)
            self.sp_max_power.setValue(max(15, min(100, int(round(max_pm / 10.0)))))
            self._log("已成功同步 STM32 当前全部 PID 与推进器分配参数", "ok")

    def _send_single_pid(self, loop_id: int):
        spins = self.pid_spin_rows[loop_id]
        vals = [sp.value() for sp in spins]
        payload = struct.pack("<B5f", loop_id, *vals)
        self._send(build_frame(CMD_SET_PID, payload))

    def _send_all_pids(self):
        for lid in range(len(PID_LOOPS_META)):
            QTimer.singleShot(lid * 25, lambda l=lid: self._send_single_pid(l))
        self._log("正在批量下发全部 7 路 PID 参数…", "info")

    def _toggle_motor_dir(self, m_idx: int):
        new_d = -1 if self.motor_dirs[m_idx] > 0 else 1
        self._set_motor_dir_ui(m_idx, new_d)

    def _set_motor_dir_ui(self, m_idx: int, d: int):
        self.motor_dirs[m_idx] = -1 if d < 0 else 1
        b = self.dir_btns[m_idx]
        if d < 0:
            b.setText(f"M{m_idx + 1}: 反桨 (-1)")
            b.setProperty("neg", "true")
        else:
            b.setText(f"M{m_idx + 1}: 正桨 (+1)")
            b.setProperty("neg", "false")
        b.style().unpolish(b)
        b.style().polish(b)

    def _apply_preset_finsrov(self):
        # FinsROV V33 原版: OutID={0,3,7,4}, InID={1,2,6,5}, Sign={1,-1,1,1,-1,-1,1,-1}
        fins_map = [0, 3, 7, 4, 1, 2, 6, 5]
        fins_sign = [1, -1, 1, 1, -1, -1, 1, -1]
        for i, ch in enumerate(fins_map):
            self.slot_combos[i].setCurrentIndex(ch)
        for i, s in enumerate(fins_sign):
            self._set_motor_dir_ui(i, s)
        self.sp_deadzone.setValue(50)
        self.rov_view.set_slot_map(fins_map)
        self._log("已载入 FinsROV V33 原版映射与正反桨预设 (点击右下角「下发」写入艇端)", "info")

    def _apply_preset_luorov(self):
        luo_map = [0, 2, 1, 3, 4, 6, 5, 7]
        for i, ch in enumerate(luo_map):
            self.slot_combos[i].setCurrentIndex(ch)
        for i in range(8):
            self._set_motor_dir_ui(i, 1)
        self.sp_deadzone.setValue(40)
        self.rov_view.set_slot_map(luo_map)
        self._log("已载入 LuoRov 顺序通道预设 (点击右下角「下发」写入艇端)", "info")

    def _send_mixer_cfg(self):
        s_map = [cb.currentIndex() for cb in self.slot_combos]
        if len(set(s_map)) < 8:
            QMessageBox.warning(self, "通道冲突", "8 个逻辑安装位存在重复的物理电机通道，请检查映射使 M1~M8 一一对应！")
            return
        self.rov_view.set_slot_map(s_map)
        payload = struct.pack(
            "<8B8bHhH",
            *s_map,
            *self.motor_dirs,
            int(self.sp_deadzone.value()),
            int(self.sp_hover_trim.value()),
            int(self.sp_max_power.value() * 10),
        )
        self._send(build_frame(CMD_SET_MIXER, payload))

    def _save_params_to_flash(self):
        if self.armed:
            QMessageBox.warning(self, "安全提示", "请先在驾驶舱点击「上锁 (DISARM)」停桨后，再固化参数到 STM32 Flash。")
            return
        self._send(build_frame(CMD_FLASH_OP, bytes([1])))

    def _restore_factory_params(self):
        self._send(build_frame(CMD_FLASH_OP, bytes([2])))
        QTimer.singleShot(100, self._request_all_params)

    def closeEvent(self, event):  # noqa: N802
        self._disconnect()
        if self.gamepad_reader is not None:
            self.gamepad_reader.stop()
        event.accept()


def main():
    app = QApplication(sys.argv)
    win = LuoRovWindow()
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
