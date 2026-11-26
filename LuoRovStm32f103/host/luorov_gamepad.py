#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LuoRov 手柄读取 (pygame + 后台线程)

只读取、不控制：在后台线程里轮询所有已连接手柄的轴和按键，把最新快照放进
线程安全的结构，供 tkinter 主循环通过 `after()` 定时读取显示。手柄数据不会
被映射到电机或任何控制命令。

用法::

    reader = GamepadReader()
    reader.start()
    snap = reader.snapshot()   # 见 GamepadReader.snapshot()
    reader.stop()

`pygame` 采用惰性导入：未安装时不会在 import 阶段崩溃，而是在
`reader.snapshot()` 里得到 ``{"connected": False, "error": ...}``。
"""

import os
import threading
import time

# ---------------------------------------------------------------------------
# 8BitDo Ultimate 2C Wireless 蓝牙(DInput)模式实测序号。
#
# 轴 (共 6 个):
#   0 = 左摇杆 X, 1 = 左摇杆 Y, 2 = 左摇杆 Y 的重复(幽灵轴, 忽略),
#   3 = 右摇杆 X, 4 = 右摇杆 Y, 5 = 未用/扳机
# 按键 (共 16 个):
#   X = 3, Y = 4 已实测; A = 1, B = 0 为 Nintendo 布局推测。
#   上位机里的「标定 A/B/X/Y」按钮可以运行时重新测得这四个序号。
# ---------------------------------------------------------------------------
AXIS_INDEX = {"LX": 0, "LY": 1, "RX": 3, "RY": 4}
DEFAULT_FACE = {"A": 1, "B": 0, "X": 3, "Y": 4}

_PYGAME = None


def _lazy_pygame():
    """延迟导入 pygame，避免未安装时模块 import 即失败。"""
    global _PYGAME
    if _PYGAME is None:
        os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
        import pygame  # noqa: F401
        _PYGAME = pygame
    return _PYGAME


class GamepadReader:
    """后台线程读取第一个已连接手柄，线程安全地暴露最新快照。"""

    def __init__(self, poll_ms: int = 20):
        self._poll_s = max(0.005, poll_ms / 1000.0)
        self._lock = threading.Lock()
        self._snap = None
        self._thread = None
        self._running = False

    # ---------- 对外接口 ----------
    def start(self):
        if self._thread is not None:
            return
        self._running = True
        self._thread = threading.Thread(
            target=self._run, name="luorov-gamepad", daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    def snapshot(self):
        """最新快照。

        返回 dict：
          {"connected": False}                    未检测到手柄
          {"connected": False, "error": "..."}     pygame 初始化失败(如未安装)
          {"connected": True, "name": str, "axes": [float, ...],
           "buttons": [bool, ...], "count": int}  正常
        线程尚未产出时返回 None。
        """
        with self._lock:
            return self._snap

    # ---------- 线程内部 ----------
    def _run(self):
        try:
            pygame = _lazy_pygame()
            pygame.init()
            pygame.joystick.init()
        except Exception as exc:  # noqa: BLE001
            self._set_snap({"connected": False, "error": str(exc)})
            return

        joysticks = []
        try:
            while self._running:
                # 1) 处理事件队列，捕获手柄插拔事件
                for event in pygame.event.get():
                    if event.type in (pygame.JOYDEVICEADDED, pygame.JOYDEVICEREMOVED):
                        self._rescan(pygame, joysticks)

                # 2) 兜底: 周期检测数量变化 (部分平台不派发插拔事件)
                if pygame.joystick.get_count() != len(joysticks):
                    self._rescan(pygame, joysticks)

                # 3) 采样
                self._poll(pygame, joysticks)
                time.sleep(self._poll_s)
        finally:
            for joy in joysticks:
                try:
                    joy.quit()
                except Exception:
                    pass
            try:
                pygame.quit()
            except Exception:
                pass

    def _rescan(self, pygame, joysticks):
        for joy in joysticks:
            try:
                joy.quit()
            except Exception:
                pass
        joysticks.clear()
        for index in range(pygame.joystick.get_count()):
            try:
                joy = pygame.joystick.Joystick(index)
                joy.init()
                joysticks.append(joy)
            except Exception:
                pass

    def _poll(self, pygame, joysticks):
        if not joysticks:
            self._set_snap({"connected": False})
            return
        joy = joysticks[0]  # 只显示第一个手柄
        try:
            axes = [round(joy.get_axis(i), 4) for i in range(joy.get_numaxes())]
            buttons = [bool(joy.get_button(i)) for i in range(joy.get_numbuttons())]
            name = joy.get_name()
            self._set_snap({
                "connected": True,
                "name": name,
                "axes": axes,
                "buttons": buttons,
                "count": len(joysticks),
            })
        except Exception:
            self._set_snap({"connected": False})

    def _set_snap(self, snap):
        with self._lock:
            self._snap = snap
