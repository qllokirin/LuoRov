#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LuoRov 电机测试工作台 (PySide6 · Flutter Material 3 风格)

功能:
  1. 电机选择: 8 路推进器 (M1~M8 / PA6~PB9) 单通道直通切换与安全控制
  2. 百分比精确输出: -100.0% ~ +100.0% 直接输入/滑块/微调，内置实测死区记录 (±10.5%)
  3. 百分比 - 推力 (g) 曲线: 仅录入百分比与推力 (g)，实时绘图并支持 CSV 导入导出
"""

from __future__ import annotations

import csv
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

from PySide6.QtCore import Qt, QPointF, QRectF, Signal
from PySide6.QtGui import QColor, QFont, QPainter, QPainterPath, QPen
from PySide6.QtWidgets import (
    QAbstractItemView,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QMessageBox,
    QPushButton,
    QSlider,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

MOTOR_SHORT_INFO = [
    ("M1", "PA6", "水平左前 H_FL"),
    ("M2", "PA7", "水平右前 H_FR"),
    ("M3", "PB0", "水平左后 H_RL"),
    ("M4", "PB1", "水平右后 H_RR"),
    ("M5", "PB6", "垂直左前 V_FL"),
    ("M6", "PB7", "垂直右前 V_FR"),
    ("M7", "PB8", "垂直左后 V_RL"),
    ("M8", "PB9", "垂直右后 V_RR"),
]

# 实测电机死区记录: ±10.5% (约 1448 ~ 1552 µs，对应混控 deadzone_comp ≈ 105 ‰)
MEASURED_DEADZONE_PCT = 10.5


@dataclass
class ThrustPoint:
    motor_idx: int     # 0..7
    cmd_pct: float     # -100.0 .. +100.0 (%)
    thrust_g: float    # 实测推力 (g)


class ThrustCurveWidget(QWidget):
    """百分比 (%) - 推力 (g) 单曲线实时绘图控件，高亮实测 ±10.5% 死区带。"""

    def __init__(self, theme: dict, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(260)
        self._theme = theme
        self._points: list[ThrustPoint] = []
        self._live_pct = 0.0
        self._title_motor = "M1 (PA6)"

    def set_theme(self, theme: dict):
        self._theme = theme
        self.update()

    def set_live_cursor(self, pct: float, motor_label: str):
        self._live_pct = pct
        self._title_motor = motor_label
        if self.isVisible():
            self.update()

    def set_points(self, points: list[ThrustPoint]):
        self._points = list(points)
        self.update()

    def paintEvent(self, event):  # noqa: N802
        t = self._theme
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, True)
        p.setRenderHint(QPainter.TextAntialiasing, True)

        w, h = self.width(), self.height()
        plot = QRectF(62, 28, max(60.0, w - 80), max(60.0, h - 54))

        # 背景框
        p.setPen(QPen(QColor(t["outline_soft"]), 1))
        p.setBrush(QColor(t["surface_inset"]))
        p.drawRoundedRect(plot, 10, 10)

        xlo, xhi = -100.0, 100.0

        def map_x(xv: float) -> float:
            norm = (xv - xlo) / (xhi - xlo)
            return plot.left() + max(0.0, min(1.0, norm)) * plot.width()

        # Y 轴范围 (推力 g)
        if self._points:
            t_vals = [pt.thrust_g for pt in self._points] + [0.0]
            tmin, tmax = min(t_vals), max(t_vals)
            span = max(100.0, (tmax - tmin) * 1.18)
            mid = 0.5 * (tmin + tmax)
            ylo, yhi = mid - span * 0.5, mid + span * 0.5
        else:
            ylo, yhi = -500.0, 500.0

        def map_y(yv: float) -> float:
            norm = (yv - ylo) / max(1e-6, yhi - ylo)
            return plot.bottom() - norm * plot.height()

        clip = QPainterPath()
        clip.addRoundedRect(plot.adjusted(1, 1, -1, -1), 9, 9)
        p.save()
        p.setClipPath(clip)

        # 实测 ±10.5% 死区高亮带
        x_dz_l = map_x(-MEASURED_DEADZONE_PCT)
        x_dz_r = map_x(+MEASURED_DEADZONE_PCT)
        wc = QColor(t["warn"])
        wc.setAlpha(30)
        p.fillRect(QRectF(x_dz_l, plot.top(), x_dz_r - x_dz_l, plot.height()), wc)
        p.setPen(QPen(QColor(t["warn"]), 1.1, Qt.DashLine))
        p.drawLine(QPointF(x_dz_l, plot.top()), QPointF(x_dz_l, plot.bottom()))
        p.drawLine(QPointF(x_dz_r, plot.top()), QPointF(x_dz_r, plot.bottom()))

        # 水平网格线
        for i in range(5):
            frac = i / 4.0
            y = plot.top() + frac * plot.height()
            p.setPen(QPen(QColor(t["outline_soft"]), 1, Qt.DotLine))
            p.drawLine(QPointF(plot.left(), y), QPointF(plot.right(), y))

        # 垂直网格线
        x_ticks = [-100, -50, 0, 50, 100]
        for xv in x_ticks:
            x = map_x(float(xv))
            is_zero = xv == 0
            p.setPen(
                QPen(
                    QColor(t["outline"] if is_zero else t["outline_soft"]),
                    1.2 if is_zero else 1.0,
                    Qt.SolidLine if is_zero else Qt.DotLine,
                )
            )
            p.drawLine(QPointF(x, plot.top()), QPointF(x, plot.bottom()))

        # 推力 0g 基准线
        if ylo <= 0.0 <= yhi:
            y0 = map_y(0.0)
            p.setPen(QPen(QColor(t["outline"]), 1.2))
            p.drawLine(QPointF(plot.left(), y0), QPointF(plot.right(), y0))

        # 当前输出百分比游标线
        cx = map_x(self._live_pct)
        cur_c = QColor(t["secondary"])
        cur_c.setAlpha(160)
        p.setPen(QPen(cur_c, 1.3, Qt.DashDotLine))
        p.drawLine(QPointF(cx, plot.top()), QPointF(cx, plot.bottom()))

        # 绘制 百分比-推力 曲线
        pts_sorted = sorted(self._points, key=lambda pt: pt.cmd_pct)
        if pts_sorted:
            prim_col = QColor(t["primary"])
            t_path = QPainterPath()
            coords: list[QPointF] = []
            for idx, pt in enumerate(pts_sorted):
                qpt = QPointF(map_x(pt.cmd_pct), map_y(pt.thrust_g))
                coords.append(qpt)
                if idx == 0:
                    t_path.moveTo(qpt)
                else:
                    t_path.lineTo(qpt)
            p.setPen(QPen(prim_col, 2.4))
            p.setBrush(Qt.NoBrush)
            if len(coords) >= 2:
                p.drawPath(t_path)
            p.setPen(QPen(QColor("#FFFFFF"), 1.3))
            p.setBrush(prim_col)
            for qpt in coords:
                p.drawEllipse(qpt, 4.3, 4.3)

        p.restore()

        # Y 轴刻度 (推力 g)
        p.setFont(QFont("Cascadia Code", 8))
        for i in range(5):
            frac = i / 4.0
            y = plot.top() + frac * plot.height()
            val_t = yhi - frac * (yhi - ylo)
            p.setPen(QColor(t["primary"]))
            p.drawText(QRectF(2, y - 7, 56, 14), Qt.AlignRight | Qt.AlignVCenter, f"{val_t:+.0f}g")

        # X 轴刻度 (百分比 %)
        p.setPen(QColor(t["text_muted"]))
        for xv in x_ticks:
            x = map_x(float(xv))
            txt = f"{xv:+d}%" if xv != 0 else "0%"
            p.drawText(QRectF(x - 24, plot.bottom() + 4, 48, 15), Qt.AlignCenter, txt)

        # 顶部图例
        p.setFont(QFont("Cascadia Code", 8, QFont.Bold))
        dz_txt = f"死区: ±{MEASURED_DEADZONE_PCT:.1f}%"
        if self._points:
            legend = f"{self._title_motor} · {len(self._points)} 点   ── 推力 (g)   |   {dz_txt}"
        else:
            legend = f"{self._title_motor} · 暂无测点 (下方输入百分比与推力 g 添加)   |   {dz_txt}"
        p.setPen(QColor(t["text_secondary"]))
        p.drawText(QRectF(plot.left(), 5, plot.width(), 18), Qt.AlignLeft | Qt.AlignVCenter, legend)
        p.end()


class MotorTestPage(QWidget):
    """精简电机测试页面: 电机选择、百分比精确输出(-100%~100%)、百分比-推力(g)曲线。"""

    motorCommandRequested = Signal(int, float)   # (ch_idx 0..7, thrust_pct -100.0..+100.0)
    allMotorsZeroRequested = Signal()
    armToggleRequested = Signal()
    estopRequested = Signal()
    logMessage = Signal(str, str)                # (msg, level)

    def __init__(self, theme: dict, parent=None):
        super().__init__(parent)
        self._theme = theme
        self._selected_ch = 0
        self._cmd_pct = 0.0
        self._motor_us = [1500] * 8
        self._armed = False
        self._connected = False

        self._points: list[ThrustPoint] = []
        self._show_all_motors = False

        self._build_ui()
        self.apply_theme(theme)

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

    def _make_tile(self, title: str, init_val: str):
        tile = QFrame()
        tile.setProperty("class", "M3Tile")
        lay = QVBoxLayout(tile)
        lay.setContentsMargins(10, 7, 10, 7)
        lay.setSpacing(2)
        lt = QLabel(title)
        lt.setProperty("class", "Muted")
        lv = QLabel(init_val)
        lv.setProperty("class", "MonoValue")
        lay.addWidget(lt)
        lay.addWidget(lv)
        return tile, lv

    # ---------------- 页面布局 ----------------
    def _build_ui(self):
        root = QHBoxLayout(self)
        root.setContentsMargins(14, 12, 14, 10)
        root.setSpacing(12)

        root.addWidget(self._build_left_control_col(), 47)
        root.addWidget(self._build_right_curve_col(), 53)

    def _build_left_control_col(self) -> QWidget:
        col = QWidget()
        lay = QVBoxLayout(col)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(12)

        # 1) 电机选择与安全控制
        self.chk_auto_zero = QCheckBox("切电机自动归零")
        self.chk_auto_zero.setChecked(True)
        self.chk_auto_zero.setToolTip("切换到另一台电机时，先将当前测试电机设为 0% (1500µs)")
        sel_card, sel_lay = self._make_card("电机选择", "MOTOR SELECT · PA6~PB9", self.chk_auto_zero)

        ch_grid = QGridLayout()
        ch_grid.setSpacing(7)
        self.ch_btns: list[QPushButton] = []
        for idx, (m_code, pin, pos_txt) in enumerate(MOTOR_SHORT_INFO):
            b = QPushButton(f"{m_code} · {pin}\n1500 µs")
            b.setProperty("class", "MotorSelBtn")
            b.setProperty("selected", "true" if idx == 0 else "false")
            b.setToolTip(f"{m_code} (通道 {idx}, {pin}) — 默认位置: {pos_txt}")
            b.setMinimumHeight(46)
            b.setCursor(Qt.PointingHandCursor)
            b.clicked.connect(lambda _=False, ci=idx: self._select_channel(ci))
            ch_grid.addWidget(b, idx // 4, idx % 4)
            self.ch_btns.append(b)
        sel_lay.addLayout(ch_grid)

        safe_row = QHBoxLayout()
        safe_row.setSpacing(8)
        self.arm_btn = QPushButton("🔓 解锁 (ARM)")
        self.arm_btn.setObjectName("ArmBtn")
        self.arm_btn.clicked.connect(self.armToggleRequested.emit)

        self.zero_cur_btn = QPushButton("当前归零 (0%)")
        self.zero_cur_btn.setMinimumHeight(36)
        self.zero_cur_btn.clicked.connect(self._zero_current_motor)

        self.zero_all_btn = QPushButton("全部停转 (1500µs)")
        self.zero_all_btn.setMinimumHeight(36)
        self.zero_all_btn.clicked.connect(self._request_all_zero)

        self.estop_btn = QPushButton("🛑 急停 (Esc)")
        self.estop_btn.setObjectName("EStopBtn")
        self.estop_btn.clicked.connect(self.estopRequested.emit)

        safe_row.addWidget(self.arm_btn, 2)
        safe_row.addWidget(self.zero_cur_btn, 2)
        safe_row.addWidget(self.zero_all_btn, 2)
        safe_row.addWidget(self.estop_btn, 2)
        sel_lay.addLayout(safe_row)
        lay.addWidget(sel_card, 0)

        # 2) 百分比精确输出 (-100.0% ~ +100.0%)
        self.chk_live_send = QCheckBox("实时跟随下发")
        self.chk_live_send.setChecked(True)
        self.chk_live_send.setToolTip("勾选后修改数值或拖动滑块立即发送；取消勾选可在输入完毕后点击「下发输出」或按回车输出")
        cmd_card, cmd_lay = self._make_card("百分比精确输出", "THRUST OUTPUT · -100.0% ~ +100.0%", self.chk_live_send)

        stat_row = QHBoxLayout()
        stat_row.setSpacing(8)
        t1, self.lbl_cmd_val = self._make_tile("设定百分比 / 固件值", "0.0%  (0)")
        t2, self.lbl_calc_us = self._make_tile("理论 PWM 脉宽", "1500 µs")
        t3, self.lbl_fb_us = self._make_tile("STM32 实测脉宽", "1500 µs")
        stat_row.addWidget(t1)
        stat_row.addWidget(t2)
        stat_row.addWidget(t3)
        cmd_lay.addLayout(stat_row)

        input_row = QHBoxLayout()
        input_row.setSpacing(8)

        lbl_in = QLabel("推力 (%):")
        lbl_in.setProperty("class", "Muted")
        self.sp_pct = QDoubleSpinBox()
        self.sp_pct.setRange(-100.0, 100.0)
        self.sp_pct.setDecimals(1)
        self.sp_pct.setSingleStep(0.5)
        self.sp_pct.setSuffix(" %")
        self.sp_pct.setValue(0.0)
        self.sp_pct.setMinimumWidth(108)
        self.sp_pct.setMinimumHeight(32)
        self.sp_pct.valueChanged.connect(self._on_spin_changed)
        self.sp_pct.editingFinished.connect(self._on_spin_enter)

        lbl_step = QLabel("步进:")
        lbl_step.setProperty("class", "Muted")
        self.cb_step = QComboBox()
        self.cb_step.addItems(["0.1%", "0.2% (1µs)", "0.5%", "1.0%", "5.0%", "10.0%"])
        self.cb_step.setCurrentIndex(2)
        self.cb_step.setFixedWidth(98)
        self.cb_step.currentIndexChanged.connect(self._on_step_changed)

        self.btn_apply_pct = QPushButton("⚡ 下发输出")
        self.btn_apply_pct.setProperty("class", "FilledBtn")
        self.btn_apply_pct.setMinimumHeight(32)
        self.btn_apply_pct.clicked.connect(self._apply_current_pct)

        input_row.addWidget(lbl_in)
        input_row.addWidget(self.sp_pct, 1)
        input_row.addWidget(lbl_step)
        input_row.addWidget(self.cb_step)
        input_row.addWidget(self.btn_apply_pct)
        cmd_lay.addLayout(input_row)

        self.pct_slider = QSlider(Qt.Horizontal)
        self.pct_slider.setRange(-1000, 1000)
        self.pct_slider.setValue(0)
        self.pct_slider.valueChanged.connect(self._on_slider_changed)
        cmd_lay.addWidget(self.pct_slider)

        nudge_row = QHBoxLayout()
        nudge_row.setSpacing(5)
        for label, delta in (
            ("-5%", -5.0),
            ("-1%", -1.0),
            ("-0.5%", -0.5),
            ("-0.2%", -0.2),
            ("0%", None),
            ("+0.2%", 0.2),
            ("+0.5%", 0.5),
            ("+1%", 1.0),
            ("+5%", 5.0),
        ):
            b = QPushButton(label)
            b.setProperty("class", "SmallPillBtn")
            b.setMinimumHeight(27)
            b.setCursor(Qt.PointingHandCursor)
            if delta is None:
                b.clicked.connect(self._zero_current_motor)
            else:
                b.clicked.connect(lambda _=False, d=delta: self._nudge_pct(d))
            nudge_row.addWidget(b, 1)
        cmd_lay.addLayout(nudge_row)

        preset_row = QHBoxLayout()
        preset_row.setSpacing(5)
        for val in (-100, -75, -50, -25, -10, 10, 25, 50, 75, 100):
            b = QPushButton(f"{val:+d}%")
            b.setProperty("class", "SmallPillBtn")
            b.setMinimumHeight(25)
            b.setCursor(Qt.PointingHandCursor)
            b.clicked.connect(lambda _=False, v=float(val): self._set_and_send_pct(v))
            preset_row.addWidget(b, 1)
        cmd_lay.addLayout(preset_row)

        # 实测死区记录卡片条
        dz_tile = QFrame()
        dz_tile.setProperty("class", "M3Tile")
        dz_lay = QVBoxLayout(dz_tile)
        dz_lay.setContentsMargins(10, 8, 10, 8)
        dz_lay.setSpacing(3)
        self.lbl_dz_record = QLabel(
            f"📌 实测死区记录: ±{MEASURED_DEADZONE_PCT:.1f}%  "
            f"(-{MEASURED_DEADZONE_PCT:.1f}% ~ +{MEASURED_DEADZONE_PCT:.1f}%，"
            f"约 {int(round(1500 - MEASURED_DEADZONE_PCT * 5))} ~ {int(round(1500 + MEASURED_DEADZONE_PCT * 5))} µs)"
        )
        self.lbl_dz_record.setProperty("class", "MonoValue")
        self.lbl_dz_note = QLabel(
            "±100% 对应固件 -1000~+1000 (1000~2000µs)，每 0.2% = 1µs；调参台建议死区补偿参考值约 105 ‰。"
        )
        self.lbl_dz_note.setProperty("class", "Muted")
        self.lbl_dz_note.setWordWrap(True)
        dz_lay.addWidget(self.lbl_dz_record)
        dz_lay.addWidget(self.lbl_dz_note)
        cmd_lay.addWidget(dz_tile)

        cmd_lay.addStretch()
        lay.addWidget(cmd_card, 1)
        return col

    def _build_right_curve_col(self) -> QWidget:
        col = QWidget()
        lay = QVBoxLayout(col)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(12)

        # 1) 百分比 - 推力 (g) 曲线
        self.lbl_curve_dz = QLabel(f"死区: ±{MEASURED_DEADZONE_PCT:.1f}%")
        self.lbl_curve_dz.setProperty("class", "MonoValue")
        plot_card, plot_lay = self._make_card(
            "百分比 - 推力曲线", "THRUST CURVE · PERCENT (%) VS THRUST (g)", self.lbl_curve_dz
        )
        self.curve_plot = ThrustCurveWidget(self._theme)
        plot_lay.addWidget(self.curve_plot, 1)
        lay.addWidget(plot_card, 5)

        # 2) 添加点 (百分比 + 推力 g) 与数据表
        self.chk_show_all_pts = QCheckBox("显示全部电机记录")
        self.chk_show_all_pts.setChecked(False)
        self.chk_show_all_pts.toggled.connect(self._on_filter_toggled)

        rec_card, rec_lay = self._make_card(
            "测点录入与数据表", "DATA POINTS · PERCENT (%) & THRUST (g)", self.chk_show_all_pts
        )

        entry_tile = QFrame()
        entry_tile.setProperty("class", "M3Tile")
        entry_row = QHBoxLayout(entry_tile)
        entry_row.setContentsMargins(10, 8, 10, 8)
        entry_row.setSpacing(8)

        l_p = QLabel("百分比 (%):")
        l_p.setProperty("class", "Muted")
        self.sp_pt_pct = QDoubleSpinBox()
        self.sp_pt_pct.setRange(-100.0, 100.0)
        self.sp_pt_pct.setDecimals(1)
        self.sp_pt_pct.setSingleStep(1.0)
        self.sp_pt_pct.setSuffix(" %")
        self.sp_pt_pct.setValue(0.0)
        self.sp_pt_pct.setMinimumWidth(96)
        self.sp_pt_pct.setMinimumHeight(30)

        l_th = QLabel("推力 (g):")
        l_th.setProperty("class", "Muted")
        self.sp_thrust_g = QDoubleSpinBox()
        self.sp_thrust_g.setRange(-50000.0, 50000.0)
        self.sp_thrust_g.setDecimals(1)
        self.sp_thrust_g.setSingleStep(10.0)
        self.sp_thrust_g.setSuffix(" g")
        self.sp_thrust_g.setValue(0.0)
        self.sp_thrust_g.setMinimumWidth(112)
        self.sp_thrust_g.setMinimumHeight(30)

        self.btn_add_pt = QPushButton("➕ 添加点")
        self.btn_add_pt.setProperty("class", "FilledBtn")
        self.btn_add_pt.setMinimumHeight(30)
        self.btn_add_pt.clicked.connect(self._record_point)

        entry_row.addWidget(l_p)
        entry_row.addWidget(self.sp_pt_pct, 1)
        entry_row.addWidget(l_th)
        entry_row.addWidget(self.sp_thrust_g, 1)
        entry_row.addWidget(self.btn_add_pt)
        rec_lay.addWidget(entry_tile)

        self.table = QTableWidget(0, 3)
        self.table.setHorizontalHeaderLabels(["电机", "百分比 (%)", "推力 (g)"])
        self.table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.table.setSelectionMode(QAbstractItemView.ExtendedSelection)
        self.table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.table.verticalHeader().setVisible(False)
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.table.setMinimumHeight(150)
        rec_lay.addWidget(self.table, 1)

        tbl_bar = QHBoxLayout()
        tbl_bar.setSpacing(6)
        btn_del_row = QPushButton("删除选中")
        btn_del_row.clicked.connect(self._delete_selected_rows)
        btn_clr_cur = QPushButton("清空当前电机")
        btn_clr_cur.clicked.connect(self._clear_current_motor_points)
        btn_clr_all = QPushButton("清空全部")
        btn_clr_all.clicked.connect(self._clear_all_points)

        btn_imp_csv = QPushButton("📂 导入 CSV")
        btn_imp_csv.clicked.connect(self._import_csv)
        btn_exp_csv = QPushButton("💾 导出 CSV")
        btn_exp_csv.setProperty("class", "FilledBtn")
        btn_exp_csv.clicked.connect(self._export_csv)

        tbl_bar.addWidget(btn_del_row)
        tbl_bar.addWidget(btn_clr_cur)
        tbl_bar.addWidget(btn_clr_all)
        tbl_bar.addStretch()
        tbl_bar.addWidget(btn_imp_csv)
        tbl_bar.addWidget(btn_exp_csv)
        rec_lay.addLayout(tbl_bar)

        lay.addWidget(rec_card, 6)
        return col

    # ---------------- 主题与遥测同步 ----------------
    def apply_theme(self, theme: dict):
        self._theme = theme
        mono_big = "font-family: 'Cascadia Code','Consolas',monospace; font-size: 14px; font-weight: 700;"
        self.lbl_cmd_val.setStyleSheet(f"{mono_big} color: {theme['primary']};")
        self.lbl_calc_us.setStyleSheet(f"{mono_big} color: {theme['text_primary']};")
        self.lbl_fb_us.setStyleSheet(f"{mono_big} color: {theme['tertiary']};")
        self.lbl_dz_record.setStyleSheet(
            f"font-family: 'Cascadia Code','Microsoft YaHei UI',sans-serif; font-size: 11px; "
            f"font-weight: 700; color: {theme['warn']};"
        )
        self.lbl_curve_dz.setStyleSheet(
            f"font-family: 'Cascadia Code','Microsoft YaHei UI',sans-serif; font-size: 11px; "
            f"font-weight: 700; color: {theme['warn']};"
        )
        self.curve_plot.set_theme(theme)
        self._refresh_readouts()

    def update_telemetry(self, motor_us: list[int], armed: bool, connected: bool):
        prev_armed = self._armed
        self._motor_us = list(motor_us[:8])
        self._armed = armed
        self._connected = connected

        if prev_armed and not armed:
            self.on_emergency_or_stop()

        if armed:
            self.arm_btn.setText("🔒 上锁 (DISARM)")
            self.arm_btn.setProperty("armed", "true")
        else:
            self.arm_btn.setText("🔓 解锁 (ARM)")
            self.arm_btn.setProperty("armed", "false")
        self.arm_btn.style().unpolish(self.arm_btn)
        self.arm_btn.style().polish(self.arm_btn)

        for idx, b in enumerate(self.ch_btns):
            m_code, pin, _ = MOTOR_SHORT_INFO[idx]
            us = self._motor_us[idx] if idx < len(self._motor_us) else 1500
            b.setText(f"{m_code} · {pin}\n{us} µs")

        self._refresh_readouts()

    def on_emergency_or_stop(self):
        self._set_pct_controls_silent(0.0)
        self._cmd_pct = 0.0
        self.sp_pt_pct.setValue(0.0)
        self._refresh_readouts()

    def on_page_leave(self):
        pass

    # ---------------- 电机通道与推力输出控制 ----------------
    def _select_channel(self, ch: int):
        if ch == self._selected_ch:
            return
        if self.chk_auto_zero.isChecked() and abs(self._cmd_pct) > 1e-6:
            old_ch = self._selected_ch
            self.motorCommandRequested.emit(old_ch, 0.0)
            self.logMessage.emit(f"切换测试通道：已将 M{old_ch + 1} 归零 (0%)", "info")

        self._selected_ch = ch
        self._cmd_pct = 0.0
        self._set_pct_controls_silent(0.0)
        self.sp_pt_pct.setValue(0.0)
        for i, b in enumerate(self.ch_btns):
            b.setProperty("selected", "true" if i == ch else "false")
            b.style().unpolish(b)
            b.style().polish(b)

        self._refresh_readouts()
        self._refresh_curve_and_table()

    def _set_pct_controls_silent(self, pct: float):
        pct = max(-100.0, min(100.0, round(pct, 1)))
        self.sp_pct.blockSignals(True)
        self.sp_pct.setValue(pct)
        self.sp_pct.blockSignals(False)
        self.pct_slider.blockSignals(True)
        self.pct_slider.setValue(int(round(pct * 10)))
        self.pct_slider.blockSignals(False)

    def _on_step_changed(self, idx: int):
        steps = [0.1, 0.2, 0.5, 1.0, 5.0, 10.0]
        if 0 <= idx < len(steps):
            self.sp_pct.setSingleStep(steps[idx])

    def _on_spin_changed(self, val: float):
        val = round(val, 1)
        self.pct_slider.blockSignals(True)
        self.pct_slider.setValue(int(round(val * 10)))
        self.pct_slider.blockSignals(False)
        self.sp_pt_pct.setValue(val)
        if self.chk_live_send.isChecked():
            self._send_pct(val)
        else:
            self._cmd_pct = val
            self._refresh_readouts()

    def _on_spin_enter(self):
        if not self.chk_live_send.isChecked():
            self._apply_current_pct()

    def _on_slider_changed(self, raw_v: int):
        val = round(raw_v / 10.0, 1)
        self.sp_pct.blockSignals(True)
        self.sp_pct.setValue(val)
        self.sp_pct.blockSignals(False)
        self.sp_pt_pct.setValue(val)
        if self.chk_live_send.isChecked():
            self._send_pct(val)
        else:
            self._cmd_pct = val
            self._refresh_readouts()

    def _apply_current_pct(self):
        val = round(self.sp_pct.value(), 1)
        self.sp_pt_pct.setValue(val)
        self._send_pct(val)

    def _nudge_pct(self, delta: float):
        new_v = max(-100.0, min(100.0, round(self.sp_pct.value() + delta, 1)))
        self._set_and_send_pct(new_v)

    def _set_and_send_pct(self, pct: float):
        pct = max(-100.0, min(100.0, round(pct, 1)))
        self._set_pct_controls_silent(pct)
        self.sp_pt_pct.setValue(pct)
        self._send_pct(pct)

    def _zero_current_motor(self):
        self._set_and_send_pct(0.0)

    def _request_all_zero(self):
        self._set_pct_controls_silent(0.0)
        self._cmd_pct = 0.0
        self.sp_pt_pct.setValue(0.0)
        self.allMotorsZeroRequested.emit()
        self._refresh_readouts()

    def _send_pct(self, pct: float):
        self._cmd_pct = round(max(-100.0, min(100.0, pct)), 1)
        self._refresh_readouts()
        if not self._connected and abs(self._cmd_pct) > 1e-6:
            self.logMessage.emit("电机测试: 串口未连接或 STM32 主控无响应，无法下发推力指令", "warn")
            return
        if not self._armed and abs(self._cmd_pct) > 1e-6:
            self.logMessage.emit("电机测试: 当前未解锁 (SAFE)，请先点击「🔓 解锁 (ARM)」后再输出推力", "warn")
            return
        self.motorCommandRequested.emit(self._selected_ch, self._cmd_pct)

    def _refresh_readouts(self):
        t = self._theme
        pct = self._cmd_pct
        speed_raw = int(round(pct * 10))
        calc_us = int(round(1500.0 + pct * 5.0))
        fb_us = self._motor_us[self._selected_ch] if self._selected_ch < len(self._motor_us) else 1500

        sign_str = f"{pct:+5.1f}%" if abs(pct) >= 0.05 else "0.0%"
        self.lbl_cmd_val.setText(f"{sign_str}  ({speed_raw:+d})")
        self.lbl_calc_us.setText(f"{calc_us} µs")

        fb_note = ""
        fb_color = t["tertiary"]
        if abs(pct) >= 0.1 and not self._armed and fb_us == 1500:
            fb_note = " (未解锁)"
            fb_color = t["text_muted"]
        elif 0.1 <= abs(pct) <= MEASURED_DEADZONE_PCT:
            fb_note = " (死区内)"
            fb_color = t["warn"]
        elif fb_us > 1500:
            fb_color = t["primary"]
        elif fb_us < 1500:
            fb_color = t["warn"]

        self.lbl_fb_us.setText(f"{fb_us} µs{fb_note}")
        self.lbl_fb_us.setStyleSheet(
            f"font-family: 'Cascadia Code','Consolas',monospace; font-size: 14px; font-weight: 700; color: {fb_color};"
        )

        m_code, pin, _ = MOTOR_SHORT_INFO[self._selected_ch]
        self.curve_plot.set_live_cursor(pct, f"{m_code} ({pin})")

    # ---------------- 测点录入与 CSV ----------------
    def _on_filter_toggled(self, checked: bool):
        self._show_all_motors = checked
        self._refresh_curve_and_table()

    def _record_point(self):
        pt = ThrustPoint(
            motor_idx=self._selected_ch,
            cmd_pct=round(self.sp_pt_pct.value(), 1),
            thrust_g=round(self.sp_thrust_g.value(), 1),
        )
        replaced = False
        for i, old in enumerate(self._points):
            if old.motor_idx == pt.motor_idx and abs(old.cmd_pct - pt.cmd_pct) < 0.05:
                self._points[i] = pt
                replaced = True
                break
        if not replaced:
            self._points.append(pt)

        self._refresh_curve_and_table()
        action = "更新" if replaced else "添加"
        self.logMessage.emit(f"已{action} M{pt.motor_idx + 1} 测点: {pt.cmd_pct:+.1f}% -> {pt.thrust_g:+.1f} g", "ok")

    def _visible_points(self) -> list[tuple[int, ThrustPoint]]:
        if self._show_all_motors:
            items = list(enumerate(self._points))
        else:
            items = [(i, pt) for i, pt in enumerate(self._points) if pt.motor_idx == self._selected_ch]
        items.sort(key=lambda pair: (pair[1].motor_idx, pair[1].cmd_pct))
        return items

    def _refresh_curve_and_table(self):
        vis = self._visible_points()
        self.table.setRowCount(len(vis))
        for row_i, (orig_idx, pt) in enumerate(vis):
            vals = [
                f"M{pt.motor_idx + 1}",
                f"{pt.cmd_pct:+.1f}",
                f"{pt.thrust_g:+.1f}",
            ]
            for col_i, txt in enumerate(vals):
                item = QTableWidgetItem(txt)
                item.setTextAlignment(Qt.AlignCenter)
                if col_i == 0:
                    item.setData(Qt.UserRole, orig_idx)
                self.table.setItem(row_i, col_i, item)

        self.curve_plot.set_points([pt for _, pt in vis])

    def _delete_selected_rows(self):
        sel = self.table.selectionModel().selectedRows()
        if not sel:
            return
        to_del: set[int] = set()
        for idx in sel:
            item = self.table.item(idx.row(), 0)
            if item is not None:
                orig = item.data(Qt.UserRole)
                if isinstance(orig, int):
                    to_del.add(orig)
        if to_del:
            self._points = [pt for i, pt in enumerate(self._points) if i not in to_del]
            self._refresh_curve_and_table()

    def _clear_current_motor_points(self):
        ch = self._selected_ch
        self._points = [pt for pt in self._points if pt.motor_idx != ch]
        self._refresh_curve_and_table()

    def _clear_all_points(self):
        if not self._points:
            return
        self._points.clear()
        self._refresh_curve_and_table()

    def _export_csv(self):
        vis = [pt for _, pt in self._visible_points()]
        if not vis:
            QMessageBox.information(self, "提示", "当前没有可导出的推力曲线数据。")
            return
        tag = "all" if self._show_all_motors else f"M{self._selected_ch + 1}"
        default_name = f"luorov_thrust_curve_{tag}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        path, _ = QFileDialog.getSaveFileName(self, "导出百分比-推力曲线 CSV", default_name, "CSV 文件 (*.csv)")
        if not path:
            return
        try:
            with open(path, "w", newline="", encoding="utf-8-sig") as f:
                writer = csv.writer(f)
                writer.writerow(["motor", "cmd_pct", "thrust_g"])
                for pt in vis:
                    writer.writerow([f"M{pt.motor_idx + 1}", f"{pt.cmd_pct:.1f}", f"{pt.thrust_g:.1f}"])
            self.logMessage.emit(f"已导出 {len(vis)} 条测点 -> {Path(path).name}", "ok")
        except Exception as exc:  # noqa: BLE001
            QMessageBox.critical(self, "导出失败", f"无法写入 CSV:\n{exc}")

    def _import_csv(self):
        path, _ = QFileDialog.getOpenFileName(self, "导入百分比-推力曲线 CSV", "", "CSV 文件 (*.csv)")
        if not path:
            return
        loaded = 0
        try:
            with open(path, "r", newline="", encoding="utf-8-sig") as f:
                reader = csv.DictReader(f)
                for row in reader:
                    m_str = (row.get("motor") or f"M{self._selected_ch + 1}").strip().upper()
                    if m_str.startswith("M") and m_str[1:].isdigit():
                        m_idx = max(0, min(7, int(m_str[1:]) - 1))
                    else:
                        m_idx = self._selected_ch
                    cmd_pct = float(row.get("cmd_pct", 0.0))
                    thrust_g = float(row.get("thrust_g", row.get("thrust_gf", 0.0)))
                    self._points.append(ThrustPoint(m_idx, round(cmd_pct, 1), round(thrust_g, 1)))
                    loaded += 1
            self._refresh_curve_and_table()
            self.logMessage.emit(f"已从 {Path(path).name} 导入 {loaded} 条测点", "ok")
        except Exception as exc:  # noqa: BLE001
            QMessageBox.critical(self, "导入失败", f"解析 CSV 失败:\n{exc}")
