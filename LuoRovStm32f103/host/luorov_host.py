#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LuoRov 八推进器 ROV 上位机 (Windows)

功能:
  - 通过 USB 转 RS485 (或岸端 485) 连接 STM32，115200 8N1
  - 显示深度、压力、温度、姿态(横滚/俯仰/偏航)、角速度、加速度
  - 显示 8 路电调当前脉冲宽度与解锁状态
  - 调试: 逐台测试电机，拖动滑块控制转速
  - 安全: 解锁/上锁、紧急停止、深度置零

依赖: pyserial  (pip install pyserial)
运行: python luorov_host.py
"""

import queue
import struct
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox

import serial
import serial.tools.list_ports

# 手柄读取 (pygame 惰性导入: 未安装时面板显示"不可用", 不影响串口功能)
try:
    from luorov_gamepad import GamepadReader, AXIS_INDEX, DEFAULT_FACE
except Exception:  # noqa: BLE001
    GamepadReader = None
    AXIS_INDEX = {"LX": 0, "LY": 1, "RX": 2, "RY": 3}
    DEFAULT_FACE = {"A": 0, "B": 1, "X": 2, "Y": 3}

# ---------------- 协议常量 (与固件 protocol.h 一致) ----------------
SYNC1 = 0xAA
SYNC2 = 0x55

CMD_PING = 0x01
CMD_SET_MOTOR = 0x02
CMD_SET_ALL = 0x03
CMD_STOP = 0x04
CMD_ARM = 0x05
CMD_ZERO_DEPTH = 0x06

TELE_STATUS = 0x80
TELE_ACK = 0x81

# 遥测结构格式: <III hhhhhhhhhh HHHHHHHH BBBB  (小端, 52 字节)
TELE_FMT = "<IIIhhhhhhhhhhHHHHHHHHBBBB"

MOTOR_NAMES = ["M1 水平", "M2 水平", "M3 水平", "M4 水平",
               "M5 垂直", "M6 垂直", "M7 垂直", "M8 垂直"]


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
        # 找同步头
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
        total = 6 + ln  # 2 sync + len + type + payload + 2 crc
        if len(buf) < total:
            return None
        frame = bytes(buf[:total])
        del buf[:total]
        # CRC 校验
        crc = frame[-1] << 8 | frame[-2]
        if crc16_modbus(frame[2:-2]) != crc:
            return None
        return frame[3], frame[4:4 + ln]


class LuoRovApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("LuoRov 八推进器 ROV 上位机")
        self.ser = None
        self.parser = FrameParser()
        self.rx_q = queue.Queue()
        self.rx_thread = None
        self.running = False
        self.last_motor_send = [0.0] * 8

        self._build_ui()
        self._refresh_ports()

        # 手柄读取线程 (只读)
        self.face_map = dict(DEFAULT_FACE)   # A/B/X/Y -> 按键序号
        self.calib = None                     # 标定状态机
        self.gamepad_reader = None
        if GamepadReader is not None:
            self.gamepad_reader = GamepadReader()
            self.gamepad_reader.start()
        else:
            self.gp_status_var.set("手柄不可用 (模块加载失败)")
            self.gp_status_lbl.configure(foreground="#d9534f")

        # 周期任务
        self.root.after(80, self._poll_queue)
        self.root.after(100, self._send_ping)
        self.root.after(40, self._poll_gamepad)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # ---------------- UI ----------------
    def _build_ui(self):
        pad = {"padx": 6, "pady": 4}

        # 顶部: 串口连接
        top = ttk.LabelFrame(self.root, text="串口连接")
        top.pack(fill="x", **pad)
        row = ttk.Frame(top)
        row.pack(fill="x", **pad)

        ttk.Label(row, text="端口:").pack(side="left")
        self.port_var = tk.StringVar()
        self.port_cb = ttk.Combobox(row, textvariable=self.port_var, width=16)
        self.port_cb.pack(side="left", padx=4)

        ttk.Label(row, text="波特率:").pack(side="left", padx=(8, 0))
        self.baud_var = tk.StringVar(value="115200")
        self.baud_cb = ttk.Combobox(row, textvariable=self.baud_var, width=9,
                                    values=["115200", "9600", "57600"])
        self.baud_cb.pack(side="left", padx=4)

        self.refresh_btn = ttk.Button(row, text="刷新", command=self._refresh_ports)
        self.refresh_btn.pack(side="left", padx=4)

        self.connect_btn = ttk.Button(row, text="连接", command=self._toggle_connect)
        self.connect_btn.pack(side="left", padx=4)

        self.link_label = ttk.Label(row, text="未连接", foreground="red")
        self.link_label.pack(side="left", padx=8)

        # 手柄: 只读显示 (不控制电机)
        gp = ttk.LabelFrame(self.root, text="手柄 (只读显示)")
        gp.pack(fill="x", **pad)

        row1 = ttk.Frame(gp)
        row1.pack(fill="x", padx=8, pady=2)
        ttk.Label(row1, text="状态:").pack(side="left")
        self.gp_status_var = tk.StringVar(value="手柄初始化中…")
        self.gp_status_lbl = ttk.Label(row1, textvariable=self.gp_status_var, foreground="#888888")
        self.gp_status_lbl.pack(side="left", padx=4)

        self.gp_calib_btn = ttk.Button(row1, text="标定 A/B/X/Y", command=self._start_calib)
        self.gp_calib_btn.pack(side="right")

        self.gp_face = {}
        face_frame = ttk.Frame(row1)
        face_frame.pack(side="right", padx=(0, 8))
        for key in ("A", "B", "X", "Y"):
            lbl = tk.Label(face_frame, text=key, width=3, relief="groove",
                           bg="#eeeeee", fg="#999999", font=("Arial", 9, "bold"))
            lbl.pack(side="left", padx=2)
            self.gp_face[key] = lbl

        # 摇杆可视化
        self.gp_canvas = self._build_stick_canvas(gp)
        self.gp_canvas.pack(fill="x", padx=8, pady=(4, 0))

        row2 = ttk.Frame(gp)
        row2.pack(fill="x", padx=8, pady=2)
        self.gp_left_var = tk.StringVar(value="L  X:--   Y:--")
        self.gp_right_var = tk.StringVar(value="R  X:--   Y:--")
        ttk.Label(row2, textvariable=self.gp_left_var, font=("Consolas", 11)).pack(side="left")
        ttk.Label(row2, textvariable=self.gp_right_var, font=("Consolas", 11)).pack(side="left", padx=18)

        row3 = ttk.Frame(gp)
        row3.pack(fill="x", padx=8, pady=2)
        ttk.Label(row3, text="其他按键:").pack(side="left")
        self.gp_extra_var = tk.StringVar(value="无")
        ttk.Label(row3, textvariable=self.gp_extra_var, font=("Consolas", 10),
                  foreground="#666666").pack(side="left", padx=4)

        row4 = ttk.Frame(gp)
        row4.pack(fill="x", padx=8, pady=2)
        ttk.Label(row4, text="轴(原始):").pack(side="left")
        self.gp_axes_var = tk.StringVar(value="")
        ttk.Label(row4, textvariable=self.gp_axes_var, font=("Consolas", 9),
                  foreground="#888888").pack(side="left", padx=4)

        ttk.Label(gp, text="说明: 点「标定 A/B/X/Y」后依次按 A、B、X、Y，即可自动校正按键序号。",
                  foreground="#888888", font=("Arial", 8)).pack(anchor="w", padx=8, pady=(0, 3))

        # 中部: 状态 + 控制
        mid = ttk.Frame(self.root)
        mid.pack(fill="both", expand=True, **pad)

        # 左: 传感器状态
        sens = ttk.LabelFrame(mid, text="传感器状态")
        sens.grid(row=0, column=0, sticky="nsew", padx=4, pady=4)

        self.depth_var = tk.StringVar(value="-- m")
        self.press_var = tk.StringVar(value="-- kPa")
        self.temp_var = tk.StringVar(value="-- ℃")
        self.roll_var = tk.StringVar(value="-- °")
        self.pitch_var = tk.StringVar(value="-- °")
        self.yaw_var = tk.StringVar(value="-- °")
        self.gyro_var = tk.StringVar(value="-- dps")
        self.acc_var = tk.StringVar(value="-- g")
        self.state_var = tk.StringVar(value="IMU:--  深度:--  状态:未解锁")

        labels = [
            ("深度", self.depth_var), ("压力", self.press_var),
            ("温度", self.temp_var),
        ]
        r = 0
        for name, var in labels:
            ttk.Label(sens, text=name + ":").grid(row=r, column=0, sticky="w", padx=8, pady=2)
            ttk.Label(sens, textvariable=var, font=("Consolas", 12)).grid(row=r, column=1, sticky="w", padx=4)
            r += 1

        ttk.Label(sens, text="横滚 Roll:").grid(row=r, column=0, sticky="w", padx=8, pady=2)
        ttk.Label(sens, textvariable=self.roll_var, font=("Consolas", 12)).grid(row=r, column=1, sticky="w")
        r += 1
        ttk.Label(sens, text="俯仰 Pitch:").grid(row=r, column=0, sticky="w", padx=8, pady=2)
        ttk.Label(sens, textvariable=self.pitch_var, font=("Consolas", 12)).grid(row=r, column=1, sticky="w")
        r += 1
        ttk.Label(sens, text="偏航 Yaw:").grid(row=r, column=0, sticky="w", padx=8, pady=2)
        ttk.Label(sens, textvariable=self.yaw_var, font=("Consolas", 12)).grid(row=r, column=1, sticky="w")
        r += 1
        ttk.Label(sens, text="角速度:").grid(row=r, column=0, sticky="w", padx=8, pady=2)
        ttk.Label(sens, textvariable=self.gyro_var, font=("Consolas", 10)).grid(row=r, column=1, sticky="w")
        r += 1
        ttk.Label(sens, text="加速度:").grid(row=r, column=0, sticky="w", padx=8, pady=2)
        ttk.Label(sens, textvariable=self.acc_var, font=("Consolas", 10)).grid(row=r, column=1, sticky="w")
        r += 1
        ttk.Label(sens, textvariable=self.state_var, foreground="#0066cc").grid(
            row=r, column=0, columnspan=2, sticky="w", padx=8, pady=4)

        # 右: 控制按钮
        ctrl = ttk.LabelFrame(mid, text="控制")
        ctrl.grid(row=0, column=1, sticky="nsew", padx=4, pady=4)

        self.arm_btn = ttk.Button(ctrl, text="🔓 解锁 (ARM)", command=self._arm)
        self.arm_btn.pack(fill="x", padx=10, pady=4)
        self.stop_btn = tk.Button(ctrl, text="🛑 紧急停止", bg="#d9534f", fg="white",
                                  font=("Arial", 12, "bold"), command=self._stop)
        self.stop_btn.pack(fill="x", padx=10, pady=4)
        self.zero_btn = ttk.Button(ctrl, text="深度置零", command=self._zero_depth)
        self.zero_btn.pack(fill="x", padx=10, pady=4)

        ttk.Separator(ctrl, orient="horizontal").pack(fill="x", padx=10, pady=6)
        ttk.Label(ctrl, text="调试: 单电机测试 (先解锁)").pack(anchor="w", padx=10)
        self.all_stop_btn = ttk.Button(ctrl, text="全部停止", command=self._all_stop)
        self.all_stop_btn.pack(fill="x", padx=10, pady=4)

        mid.columnconfigure(0, weight=1)
        mid.columnconfigure(1, weight=1)

        # 下部: 8 路电机滑块
        motors = ttk.LabelFrame(self.root, text="电机调试 (拖动滑块, -100%~+100%)")
        motors.pack(fill="x", **pad)

        self.sliders = []
        self.slider_vals = []
        grid = ttk.Frame(motors)
        grid.pack(fill="x", padx=10, pady=6)
        for i in range(8):
            var = tk.IntVar(value=0)
            self.slider_vals.append(var)
            col = i % 4
            row_ = i // 4
            cell = ttk.Frame(grid)
            cell.grid(row=row_, column=col, sticky="ew", padx=8, pady=4)
            ttk.Label(cell, text=MOTOR_NAMES[i]).pack()
            lbl = ttk.Label(cell, text="0%")
            lbl.pack()
            s = ttk.Scale(cell, from_=-100, to=100, variable=var, orient="horizontal",
                          command=lambda v, idx=i, lab=lbl: self._on_slider(idx, v, lab))
            s.pack(fill="x")
            self.sliders.append(s)
        for c in range(4):
            grid.columnconfigure(c, weight=1)

        # 底部: 日志
        logf = ttk.LabelFrame(self.root, text="日志")
        logf.pack(fill="both", expand=True, **pad)
        self.log_text = tk.Text(logf, height=7, state="disabled")
        self.log_text.pack(fill="both", expand=True, padx=6, pady=4)

    def _log(self, msg: str):
        self.log_text.configure(state="normal")
        self.log_text.insert("end", time.strftime("[%H:%M:%S] ") + msg + "\n")
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    # ---------------- 串口 ----------------
    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_cb["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def _toggle_connect(self):
        if self.ser and self.ser.is_open:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        port = self.port_var.get().strip()
        baud = int(self.baud_var.get())
        if not port:
            messagebox.showwarning("提示", "请选择串口")
            return
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
        except Exception as e:
            messagebox.showerror("打开失败", str(e))
            return
        self.running = True
        self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.rx_thread.start()
        self.connect_btn.configure(text="断开")
        self.link_label.configure(text=f"已连接 {port} @ {baud}", foreground="green")
        self._log(f"连接 {port} @ {baud}")

    def _disconnect(self):
        self.running = False
        if self.rx_thread:
            self.rx_thread.join(timeout=1.0)
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self.connect_btn.configure(text="连接")
        self.link_label.configure(text="未连接", foreground="red")
        self._log("已断开")

    def _send(self, data: bytes):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(data)
            except Exception as e:
                self._log(f"发送失败: {e}")

    # ---------------- 接收线程 ----------------
    def _rx_loop(self):
        while self.running and self.ser and self.ser.is_open:
            try:
                data = self.ser.read(256)
                if not data:
                    continue
            except Exception:
                break
            for typ, payload in self.parser.feed(data):
                if typ == TELE_STATUS:
                    try:
                        t = struct.unpack(TELE_FMT, payload)
                        self.rx_q.put(("status", t))
                    except struct.error:
                        pass
                elif typ == TELE_ACK:
                    if len(payload) >= 2:
                        self.rx_q.put(("ack", payload[0], payload[1]))
        self.rx_q.put(("disconnect",))

    # ---------------- 周期任务 ----------------
    def _poll_queue(self):
        try:
            while True:
                item = self.rx_q.get_nowait()
                if item[0] == "status":
                    self._update_status(item[1])
                elif item[0] == "ack":
                    self._log(f"ACK: 命令 0x{item[1]:02X} -> {'成功' if item[2] == 0 else '失败'}")
                elif item[0] == "disconnect":
                    self._log("串口已断开")
        except queue.Empty:
            pass
        self.root.after(80, self._poll_queue)

    def _send_ping(self):
        if self.ser and self.ser.is_open:
            self._send(build_frame(CMD_PING))
        self.root.after(100, self._send_ping)

    # ---------------- 状态更新 ----------------
    def _update_status(self, t):
        (uptime, depth_mm, press_pa, temp_c100,
         roll, pitch, yaw, gx, gy, gz, ax, ay, az,
         m0, m1, m2, m3, m4, m5, m6, m7,
         armed, imu_ok, depth_ok, leak) = t

        self.depth_var.set(f"{depth_mm / 1000.0:.2f} m")
        self.press_var.set(f"{press_pa / 1000.0:.2f} kPa")
        self.temp_var.set(f"{temp_c100 / 100.0:.2f} ℃")
        self.roll_var.set(f"{roll / 100.0:.1f} °")
        self.pitch_var.set(f"{pitch / 100.0:.1f} °")
        self.yaw_var.set(f"{yaw / 100.0:.1f} °")
        self.gyro_var.set(f"X:{gx / 100.0:7.1f} Y:{gy / 100.0:7.1f} Z:{gz / 100.0:7.1f}")
        self.acc_var.set(f"X:{ax / 1000.0:.3f} Y:{ay / 1000.0:.3f} Z:{az / 1000.0:.3f}")
        self.state_var.set(
            f"IMU:{'OK' if imu_ok else '--'}  深度:{'OK' if depth_ok else '--'}  "
            f"状态:{'已解锁' if armed else '未解锁'}  运行:{uptime / 1000.0:.0f}s")

        motors = [m0, m1, m2, m3, m4, m5, m6, m7]
        # 更新滑块下方显示当前脉冲 (可选)
        for i in range(8):
            pass

    # ---------------- 手柄 (只读显示) ----------------
    def _build_stick_canvas(self, parent):
        c = tk.Canvas(parent, width=272, height=120, bg="#f5f6f7", highlightthickness=0)
        c.create_text(66, 116, text="左摇杆", fill="#888888", font=("Arial", 9))
        c.create_text(202, 116, text="右摇杆", fill="#888888", font=("Arial", 9))
        for cx in (66, 202):
            cy = 58
            c.create_oval(cx - 46, cy - 46, cx + 46, cy + 46, outline="#c8c8c8")
            c.create_line(cx - 46, cy, cx + 46, cy, fill="#e0e0e0")
            c.create_line(cx, cy - 46, cx, cy + 46, fill="#e0e0e0")
        return c

    def _draw_sticks(self, lx, ly, rx, ry):
        c = self.gp_canvas
        c.delete("dot")
        for cx, x, y in ((66, lx, ly), (202, rx, ry)):
            if x is None or y is None:
                continue
            px = cx + max(-1.0, min(1.0, x)) * 40
            py = 58 + max(-1.0, min(1.0, y)) * 40
            c.create_oval(px - 6, py - 6, px + 6, py + 6,
                          fill="#2a7de1", outline="#ffffff", width=2, tags="dot")

    def _poll_gamepad(self):
        if self.gamepad_reader is not None:
            self._update_gamepad(self.gamepad_reader.snapshot())
        self.root.after(40, self._poll_gamepad)

    def _update_gamepad(self, snap):
        if snap is None:
            self.gp_status_var.set("手柄初始化中…")
            self.gp_status_lbl.configure(foreground="#888888")
            return
        if snap.get("error"):
            self.gp_status_var.set("手柄不可用: " + snap["error"])
            self.gp_status_lbl.configure(foreground="#d9534f")
            self._reset_gamepad_display()
            return
        if not snap.get("connected"):
            self.gp_status_var.set("未检测到手柄 (连接后按一下手柄按键)")
            self.gp_status_lbl.configure(foreground="#888888")
            self._reset_gamepad_display()
            return

        name = snap.get("name", "")
        axes = snap.get("axes", [])
        buttons = snap.get("buttons", [])
        count = snap.get("count", 1)

        # 标定模式下只处理按键捕获
        if self.calib is not None:
            self._calib_tick(buttons)
            return

        status = f"{name}  ({len(axes)}轴/{len(buttons)}键)"
        if count > 1:
            status += f"  ·  共{count}个手柄"
        self.gp_status_var.set(status)
        self.gp_status_lbl.configure(foreground="#0a7a2f")

        def axis(idx):
            return axes[idx] if idx < len(axes) else None

        lx = axis(AXIS_INDEX["LX"])
        ly = axis(AXIS_INDEX["LY"])
        rx = axis(AXIS_INDEX["RX"])
        ry = axis(AXIS_INDEX["RY"])

        def fmt(v):
            return f"{v:+.3f}" if v is not None else "--"

        self.gp_left_var.set(f"L  X:{fmt(lx)}   Y:{fmt(ly)}")
        self.gp_right_var.set(f"R  X:{fmt(rx)}   Y:{fmt(ry)}")
        self._draw_sticks(lx, ly, rx, ry)

        # A/B/X/Y 按下状态 (按 face_map 序号)
        pressed = {}
        for key, idx in self.face_map.items():
            pressed[key] = idx < len(buttons) and buttons[idx]
        self._set_face(pressed)

        # 原始轴, 便于核对序号
        self.gp_axes_var.set(" ".join(f"{v:+.2f}" for v in axes) if axes else "--")

        # 其他按键 (显示原始序号, 排除已标定的 A/B/X/Y 序号)
        used = set(self.face_map.values())
        extras = [f"B{i}" for i in range(len(buttons)) if buttons[i] and i not in used]
        self.gp_extra_var.set(" ".join(extras) if extras else "无")

    def _reset_gamepad_display(self):
        self.gp_left_var.set("L  X:--   Y:--")
        self.gp_right_var.set("R  X:--   Y:--")
        self.gp_axes_var.set("")
        self.gp_extra_var.set("无")
        self._draw_sticks(None, None, None, None)
        self._set_face({})

    def _set_face(self, pressed):
        for key, lbl in self.gp_face.items():
            if pressed.get(key):
                lbl.configure(bg="#2a7de1", fg="#ffffff", relief="sunken")
            else:
                lbl.configure(bg="#eeeeee", fg="#999999", relief="groove")

    # ---------------- 手柄按键标定 ----------------
    def _start_calib(self):
        prev = set()
        if self.gamepad_reader is not None:
            snap = self.gamepad_reader.snapshot()
            if snap and snap.get("connected"):
                btns = snap.get("buttons", [])
                prev = {i for i in range(len(btns)) if btns[i]}
        self.calib = {"step": 0, "labels": ["A", "B", "X", "Y"], "map": {}, "prev": prev}
        self.gp_status_lbl.configure(foreground="#b36b00")
        self.gp_status_var.set("标定: 请按下 A")
        self._log("开始标定 A/B/X/Y：请依次按下 A、B、X、Y")

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
                text = "标定完成: " + "  ".join(f"{k}={v}" for k, v in sorted(self.face_map.items()))
                self.gp_status_var.set(text)
                self.gp_status_lbl.configure(foreground="#0a7a2f")
                self._log(text)
            else:
                self.gp_status_var.set(f"标定: 请按下 {nxt}")


    # ---------------- 控制命令 ----------------
    def _arm(self):
        self._send(build_frame(CMD_ARM, bytes([1])))
        self._log("发送解锁命令")

    def _stop(self):
        self._send(build_frame(CMD_STOP))
        for var in self.slider_vals:
            var.set(0)
        self._log("发送紧急停止命令")

    def _zero_depth(self):
        self._send(build_frame(CMD_ZERO_DEPTH))
        self._log("发送深度置零命令")

    def _all_stop(self):
        # 全部电机回中 (通过 SET_ALL 全零)
        self._send(build_frame(CMD_SET_ALL, struct.pack("<8h", *([0] * 8))))
        for var in self.slider_vals:
            var.set(0)
        self._log("全部电机回中")

    def _on_slider(self, idx, value, label):
        try:
            v = int(float(value))
        except ValueError:
            v = 0
        label.configure(text=f"{v}%")
        # 节流发送, 拖动时实时下发
        now = time.time()
        if now - self.last_motor_send[idx] > 0.03:
            self.last_motor_send[idx] = now
            speed = int(v * 10)  # -100..100 -> -1000..1000
            self._send(build_frame(CMD_SET_MOTOR, bytes([idx]) + struct.pack("<h", speed)))

    def _on_close(self):
        self._disconnect()
        if self.gamepad_reader is not None:
            self.gamepad_reader.stop()
        self.root.destroy()


def main():
    root = tk.Tk()
    root.geometry("800x900")
    app = LuoRovApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
