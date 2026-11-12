#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PMW3901 光流轨迹上位机（PyQt5 + pyqtgraph）

功能：
  - 串口连接 ESP32-S3，解析固件发来的 $FLOW 数据帧（带 XOR 校验）
  - 把每帧 dx/dy 积分成 2D 轨迹（mm），实时显示
  - 实时曲线：dx/dy（counts）、SQUAL、Shutter
  - 状态栏：采样率、包数、坏帧、当前坐标、速度、累计距离
  - 高度 + 系数 K 换算；支持“推已知距离自动标定 K”
  - 清零、暂停、CSV 导出、窗口截图
  - --demo 演示模式（不接板子也能看界面）

用法：
  python flow_host.py                    # 打开界面，手动选串口连接
  python flow_host.py --port COM6        # 打开后自动连接
  python flow_host.py --demo             # 演示模式（合成圆周轨迹）
  python flow_host.py --selftest out.png # 无交互自检，生成截图（开发用）
"""

import argparse
import csv
import math
import os
import random
import sys
import time
from collections import deque

import numpy as np

from PyQt5 import QtCore, QtGui, QtWidgets
import pyqtgraph as pg

try:
    import serial
    from serial.tools import list_ports
except ImportError:          # 没装 pyserial 时界面仍能打开（演示/回放）
    serial = None
    list_ports = None


APP_NAME = "PMW3901 光流轨迹上位机"
DEFAULT_K = 0.0209          # 位移(mm) = counts × 高度(mm) × K
MAX_POINTS = 200000         # 轨迹点上限，超过后自动抽稀
ROLL_SECONDS = 10.0         # 右侧曲线滚动窗口（秒）
PROTO_TAG = b"$FLOW"


# ---------------------------------------------------------------------------
# 协议解析
# ---------------------------------------------------------------------------
def _xor8(data: bytes) -> int:
    x = 0
    for b in data:
        x ^= b
    return x


def parse_flow_line(raw: bytes):
    """解析一行 `$FLOW,t,dx,dy,motion,squal,shutter*XOR`，失败返回 None。"""
    line = raw.strip()
    if not line.startswith(PROTO_TAG):
        return None
    if b"*" not in line:
        return None
    body, _, chk = line[1:].partition(b"*")
    try:
        if _xor8(body) != int(chk[:2], 16):
            return None
        fields = body.decode("ascii").split(",")
        if len(fields) != 7 or fields[0] != "FLOW":
            return None
        return (int(fields[1]), int(fields[2]), int(fields[3]),
                int(fields[4]), int(fields[5]), int(fields[6]))
    except (ValueError, UnicodeDecodeError):
        return None


# ---------------------------------------------------------------------------
# 串口读取线程
# ---------------------------------------------------------------------------
class SerialWorker(QtCore.QThread):
    sample = QtCore.pyqtSignal(object)      # (t_ms, dx, dy, motion, squal, shutter)
    bad_frame = QtCore.pyqtSignal()
    error = QtCore.pyqtSignal(str)

    def __init__(self, port, baud, parent=None):
        super().__init__(parent)
        self.port = port
        self.baud = baud
        self._running = True

    def run(self):
        try:
            ser = serial.Serial(self.port, self.baud, timeout=0.1)
        except Exception as exc:            # noqa: BLE001
            self.error.emit(f"打开 {self.port} 失败：{exc}")
            return

        # 避免打开串口时 DTR/RTS 把板子按在复位状态
        try:
            ser.dtr = False
            ser.rts = False
        except Exception:                   # noqa: BLE001
            pass

        buf = bytearray()
        try:
            while self._running:
                chunk = ser.read(512)
                if not chunk:
                    continue
                buf.extend(chunk)
                while True:
                    idx = buf.find(b"\n")
                    if idx < 0:
                        break
                    line = bytes(buf[:idx])
                    del buf[:idx + 1]
                    if not line.strip():
                        continue
                    rec = parse_flow_line(line)
                    if rec is not None:
                        self.sample.emit(rec)
                    elif line.strip().startswith(b"$"):
                        # 以 $ 开头但不是合法 $FLOW：算坏帧（ESP_LOG 日志行不算）
                        self.bad_frame.emit()
                if len(buf) > 8192:         # 防止异常数据把缓冲撑爆
                    buf.clear()
        except Exception as exc:            # noqa: BLE001
            if self._running:
                self.error.emit(f"串口读取异常：{exc}")
        finally:
            try:
                ser.close()
            except Exception:               # noqa: BLE001
                pass

    def stop(self):
        self._running = False
        self.wait(500)


# ---------------------------------------------------------------------------
# 演示数据源（不接板子也能看界面）
# ---------------------------------------------------------------------------
def circle_samples(seconds, height_mm=100.0, k=DEFAULT_K,
                   radius=100.0, speed=120.0, hz=50):
    """生成一串合成 $FLOW 样本：匀速圆周 + 量化误差扩散，让轨迹画得平滑。"""
    res_x = res_y = 0.0
    ang = 0.0
    t_ms = 0
    for _ in range(int(seconds * hz)):
        t_ms += int(1000 / hz)
        dtheta = (speed / hz) / radius
        x0, y0 = radius * math.cos(ang), radius * math.sin(ang)
        ang += dtheta
        x1, y1 = radius * math.cos(ang), radius * math.sin(ang)
        scale = height_mm * k
        fx = (x1 - x0) / scale + random.gauss(0, 0.2) + res_x
        fy = (y1 - y0) / scale + random.gauss(0, 0.2) + res_y
        dx, dy = int(round(fx)), int(round(fy))
        res_x, res_y = fx - dx, fy - dy
        yield (t_ms, dx, dy, 1,
               int(200 + random.gauss(0, 5)),
               int(2800 + random.gauss(0, 60)))


class DemoSource(QtCore.QThread):
    sample = QtCore.pyqtSignal(object)

    def __init__(self, height_mm=100.0, k=DEFAULT_K, parent=None):
        super().__init__(parent)
        self.height_mm = height_mm
        self.k = k
        self._running = True

    def run(self):
        for rec in circle_samples(3600.0, self.height_mm, self.k):
            if not self._running:
                break
            self.sample.emit(rec)
            self.msleep(20)

    def stop(self):
        self._running = False
        self.wait(500)


# ---------------------------------------------------------------------------
# 双 Y 轴曲线控件（SQUAL 左轴 + Shutter 右轴）
# ---------------------------------------------------------------------------
class DualAxisPlot(pg.PlotWidget):
    def __init__(self, left_label, right_label, right_color="#ff9500", parent=None):
        super().__init__(parent=parent)
        self.setLabel("left", left_label)
        self.showAxis("right")
        self.getAxis("right").setLabel(right_label)
        self.getAxis("right").setPen(pg.mkPen(right_color))
        self.getAxis("right").setTextPen(pg.mkPen(right_color))

        self._vb_right = pg.ViewBox()
        self.scene().addItem(self._vb_right)
        self.getAxis("right").linkToView(self._vb_right)
        self._vb_right.setXLink(self.getPlotItem())
        self.getPlotItem().getViewBox().sigResized.connect(self._sync_views)
        QtCore.QTimer.singleShot(0, self._sync_views)

    def _sync_views(self):
        self._vb_right.setGeometry(self.getPlotItem().getViewBox().sceneBoundingRect())
        self._vb_right.linkedViewChanged(self.getPlotItem().getViewBox(), self._vb_right.XAxis)

    def add_right_curve(self, **kwargs):
        curve = pg.PlotCurveItem(**kwargs)
        self._vb_right.addItem(curve)
        self._vb_right.enableAutoRange(axis=pg.ViewBox.YAxis)
        return curve


# ---------------------------------------------------------------------------
# 标定对话框
# ---------------------------------------------------------------------------
class CalibDialog(QtWidgets.QDialog):
    """推一段已知距离，反算系数 K：K = 距离 / (counts × 高度)。"""

    def __init__(self, height_mm, get_counts, parent=None):
        super().__init__(parent)
        self.setWindowTitle("自动标定系数 K")
        self.setMinimumWidth(460)
        self.height_mm = height_mm
        self.get_counts = get_counts          # () -> (net_dx_counts, net_dy_counts)
        self.start_counts = None

        lay = QtWidgets.QVBoxLayout(self)
        tip = QtWidgets.QLabel(
            "步骤：\n"
            "1. 把模块固定在已知高度（与主界面“高度”一致），镜头朝下；\n"
            "2. 点击【开始记录】，然后沿直线（尽量沿模块 X 轴）匀速推一段距离；\n"
            "3. 输入实际推动距离，点击【计算并应用】。"
        )
        tip.setWordWrap(True)
        lay.addWidget(tip)

        self.lbl_live = QtWidgets.QLabel("当前累计 counts：dx=0, dy=0（幅值 0）")
        self.lbl_live.setStyleSheet("font-family: Consolas, monospace;")
        lay.addWidget(self.lbl_live)

        row = QtWidgets.QHBoxLayout()
        row.addWidget(QtWidgets.QLabel("实际推动距离 (mm)："))
        self.spin_dist = QtWidgets.QDoubleSpinBox()
        self.spin_dist.setRange(1.0, 5000.0)
        self.spin_dist.setDecimals(1)
        self.spin_dist.setValue(200.0)
        row.addWidget(self.spin_dist)
        row.addStretch(1)
        lay.addLayout(row)

        self.lbl_result = QtWidgets.QLabel("K 尚未计算")
        self.lbl_result.setStyleSheet("font-family: Consolas, monospace;")
        lay.addWidget(self.lbl_result)

        btns = QtWidgets.QHBoxLayout()
        self.btn_start = QtWidgets.QPushButton("开始记录")
        self.btn_calc = QtWidgets.QPushButton("计算并应用")
        self.btn_calc.setEnabled(False)
        btn_close = QtWidgets.QPushButton("关闭")
        btns.addWidget(self.btn_start)
        btns.addWidget(self.btn_calc)
        btns.addStretch(1)
        btns.addWidget(btn_close)
        lay.addLayout(btns)

        self.btn_start.clicked.connect(self._on_start)
        self.btn_calc.clicked.connect(self._on_calc)
        btn_close.clicked.connect(self.accept)

        self.result_k = None
        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self._refresh_live)
        self._timer.start(100)

    def _net_counts(self):
        cx, cy = self.get_counts()
        if self.start_counts is None:
            return cx, cy
        return cx - self.start_counts[0], cy - self.start_counts[1]

    def _refresh_live(self):
        dx, dy = self._net_counts()
        self.lbl_live.setText(
            f"当前累计 counts：dx={dx}, dy={dy}（幅值 {math.hypot(dx, dy):.0f}）")

    def _on_start(self):
        self.start_counts = self.get_counts()
        self.btn_calc.setEnabled(True)
        self.lbl_result.setText("已开始记录，推完后点【计算并应用】")

    def _on_calc(self):
        if self.start_counts is None:
            return
        dx, dy = self._net_counts()
        mag = math.hypot(dx, dy)
        if mag < 1.0:
            QtWidgets.QMessageBox.warning(self, "提示", "没有测到位移，先推一段再计算。")
            return
        dist = self.spin_dist.value()
        k = dist / (mag * self.height_mm)
        self.result_k = k
        self.lbl_result.setText(
            f"counts={mag:.0f}  距离={dist:.1f}mm  高度={self.height_mm:.0f}mm\n"
            f"K_实测 = {k:.6f}（当前使用 {DEFAULT_K:.6f}）")
        self.accept()


# ---------------------------------------------------------------------------
# 主窗口
# ---------------------------------------------------------------------------
class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, demo=False, auto_port=None):
        super().__init__()
        self.setWindowTitle(APP_NAME)
        self.resize(1500, 900)

        self.demo_mode = demo
        self.worker = None
        self.demo_source = None
        self.paused = False
        self.follow = True
        self.packets = 0
        self.bad_frames = 0
        self.motion_frames = 0

        # 轨迹与统计
        self.x = 0.0
        self.y = 0.0
        self.count_x = 0            # 净 counts（标定用，不受翻转影响）
        self.count_y = 0
        self.distance = 0.0
        self.xs, self.ys = [], []
        self.samples = []           # (t_ms, dx, dy, motion, squal, shutter, x, y)
        self.hist_t = deque(maxlen=600)
        self.hist_dx = deque(maxlen=600)
        self.hist_dy = deque(maxlen=600)
        self.hist_sq = deque(maxlen=600)
        self.hist_sh = deque(maxlen=600)
        self.speed_win = deque(maxlen=40)     # (t_ms, dx_mm, dy_mm)
        self.rate_t = deque(maxlen=100)       # 设备时间戳，用于算采样率
        self._dirty = True
        self._follow_tick = 0

        self._build_ui()
        self._load_settings()
        self._refresh_ports()

        self.timer_plot = QtCore.QTimer(self)
        self.timer_plot.timeout.connect(self._update_plots)
        self.timer_plot.start(40)             # 25 fps 刷新

        self.timer_stats = QtCore.QTimer(self)
        self.timer_stats.timeout.connect(self._update_stats)
        self.timer_stats.start(200)

        if auto_port:
            idx = self.combo_port.findData(auto_port)
            if idx < 0:
                self.combo_port.addItem(auto_port, auto_port)
                idx = self.combo_port.count() - 1
            self.combo_port.setCurrentIndex(idx)
            QtCore.QTimer.singleShot(300, self._toggle_connection)

        if demo:
            self._start_demo()

    # ---------------- UI ----------------
    def _build_ui(self):
        pg.setConfigOptions(antialias=True, background="w", foreground="k")

        central = QtWidgets.QWidget()
        root = QtWidgets.QVBoxLayout(central)
        root.setContentsMargins(8, 8, 8, 8)

        # ---- 第一行：连接 ----
        row1 = QtWidgets.QHBoxLayout()
        row1.addWidget(QtWidgets.QLabel("串口："))
        self.combo_port = QtWidgets.QComboBox()
        self.combo_port.setMinimumWidth(240)
        row1.addWidget(self.combo_port)
        self.btn_refresh = QtWidgets.QPushButton("刷新")
        row1.addWidget(self.btn_refresh)
        row1.addWidget(QtWidgets.QLabel("波特率："))
        self.combo_baud = QtWidgets.QComboBox()
        self.combo_baud.addItems(["115200", "230400", "460800", "921600"])
        self.combo_baud.setCurrentText("115200")
        row1.addWidget(self.combo_baud)
        self.btn_conn = QtWidgets.QPushButton("连接")
        self.btn_conn.setMinimumWidth(90)
        row1.addWidget(self.btn_conn)
        self.lbl_conn = QtWidgets.QLabel("● 未连接")
        self.lbl_conn.setStyleSheet("color:#c0392b; font-weight:bold;")
        row1.addWidget(self.lbl_conn)
        row1.addStretch(1)
        root.addLayout(row1)

        # ---- 第二行：参数与操作 ----
        row2 = QtWidgets.QHBoxLayout()
        row2.addWidget(QtWidgets.QLabel("对地高度："))
        self.spin_height = QtWidgets.QDoubleSpinBox()
        self.spin_height.setRange(10.0, 5000.0)
        self.spin_height.setDecimals(1)
        self.spin_height.setSingleStep(10.0)
        self.spin_height.setValue(100.0)
        self.spin_height.setSuffix(" mm")
        row2.addWidget(self.spin_height)

        row2.addWidget(QtWidgets.QLabel("系数 K："))
        self.spin_k = QtWidgets.QDoubleSpinBox()
        self.spin_k.setRange(0.0001, 1.0)
        self.spin_k.setDecimals(6)
        self.spin_k.setSingleStep(0.0005)
        self.spin_k.setValue(DEFAULT_K)
        row2.addWidget(self.spin_k)
        lbl_k_hint = QtWidgets.QLabel("（位移 = counts × 高度 × K）")
        lbl_k_hint.setStyleSheet("color:#666;")
        row2.addWidget(lbl_k_hint)

        self.chk_flip_x = QtWidgets.QCheckBox("翻转 X")
        self.chk_flip_y = QtWidgets.QCheckBox("翻转 Y")
        row2.addWidget(self.chk_flip_x)
        row2.addWidget(self.chk_flip_y)

        row2.addSpacing(12)
        self.btn_clear = QtWidgets.QPushButton("清零")
        self.btn_calib = QtWidgets.QPushButton("标定 K")
        self.btn_pause = QtWidgets.QPushButton("暂停")
        self.btn_follow = QtWidgets.QPushButton("跟随：开")
        self.btn_follow.setCheckable(True)
        self.btn_follow.setChecked(True)
        for b in (self.btn_clear, self.btn_calib, self.btn_pause, self.btn_follow):
            row2.addWidget(b)

        row2.addSpacing(12)
        self.btn_csv = QtWidgets.QPushButton("保存 CSV")
        self.btn_shot = QtWidgets.QPushButton("截图")
        row2.addWidget(self.btn_csv)
        row2.addWidget(self.btn_shot)
        row2.addStretch(1)
        root.addLayout(row2)

        # ---- 主体：左轨迹 + 右曲线 ----
        splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)

        left = QtWidgets.QWidget()
        left_lay = QtWidgets.QVBoxLayout(left)
        left_lay.setContentsMargins(0, 0, 0, 0)
        title = QtWidgets.QLabel("轨迹（mm）")
        title.setStyleSheet("font-weight:bold;")
        left_lay.addWidget(title)
        self.p_traj = pg.PlotWidget()
        self.p_traj.setAspectLocked(True)
        self.p_traj.showGrid(x=True, y=True, alpha=0.25)
        self.p_traj.setLabel("bottom", "X (mm)")
        self.p_traj.setLabel("left", "Y (mm)")
        self.curve_traj = self.p_traj.plot([], [], pen=pg.mkPen("#0a84ff", width=2))
        self.dot_now = pg.ScatterPlotItem(
            [0], [0], size=11, brush=pg.mkBrush("#ff3b30"),
            pen=pg.mkPen("w", width=1))
        self.dot_start = pg.ScatterPlotItem(
            [0], [0], size=9, brush=pg.mkBrush("#34c759"),
            pen=pg.mkPen("w", width=1))
        self.p_traj.addItem(self.dot_now)
        self.p_traj.addItem(self.dot_start)
        self.p_traj.addItem(pg.InfiniteLine(
            0, angle=90, pen=pg.mkPen("#bbbbbb", style=QtCore.Qt.DashLine)))
        self.p_traj.addItem(pg.InfiniteLine(
            0, angle=0, pen=pg.mkPen("#bbbbbb", style=QtCore.Qt.DashLine)))
        left_lay.addWidget(self.p_traj, 1)
        splitter.addWidget(left)

        right = QtWidgets.QWidget()
        right_lay = QtWidgets.QVBoxLayout(right)
        right_lay.setContentsMargins(0, 0, 0, 0)

        lbl1 = QtWidgets.QLabel("原始位移 dx / dy（counts）")
        lbl1.setStyleSheet("font-weight:bold;")
        right_lay.addWidget(lbl1)
        self.p_raw = pg.PlotWidget()
        self.p_raw.showGrid(x=True, y=True, alpha=0.25)
        self.p_raw.setLabel("bottom", "时间 (s)")
        self.p_raw.setLabel("left", "counts")
        self.p_raw.addLegend(offset=(-10, 10))
        self.curve_dx = self.p_raw.plot([], [], pen=pg.mkPen("#34c759", width=2), name="dx")
        self.curve_dy = self.p_raw.plot([], [], pen=pg.mkPen("#ff9500", width=2), name="dy")
        right_lay.addWidget(self.p_raw, 1)

        lbl2 = QtWidgets.QLabel("地表质量 SQUAL / 快门 Shutter")
        lbl2.setStyleSheet("font-weight:bold;")
        right_lay.addWidget(lbl2)
        self.p_qual = DualAxisPlot("SQUAL", "Shutter", right_color="#8e8e93")
        self.p_qual.showGrid(x=True, y=True, alpha=0.25)
        self.p_qual.setLabel("bottom", "时间 (s)")
        self.curve_sq = self.p_qual.plot([], [], pen=pg.mkPen("#0a84ff", width=2))
        self.curve_sh = self.p_qual.add_right_curve(pen=pg.mkPen("#8e8e93", width=1))
        right_lay.addWidget(self.p_qual, 1)

        splitter.addWidget(right)
        splitter.setSizes([820, 620])
        root.addWidget(splitter, 1)

        self.setCentralWidget(central)

        # ---- 状态栏 ----
        bar = self.statusBar()
        self.lbl_rate = QtWidgets.QLabel("采样率: -- Hz")
        self.lbl_packets = QtWidgets.QLabel("包: 0  坏帧: 0")
        self.lbl_pos = QtWidgets.QLabel("坐标: (0.0, 0.0) mm")
        self.lbl_speed = QtWidgets.QLabel("速度: (0.0, 0.0) mm/s")
        self.lbl_dist = QtWidgets.QLabel("累计距离: 0.0 mm")
        self.lbl_sensor = QtWidgets.QLabel("squal: --  shutter: --")
        for w in (self.lbl_rate, self.lbl_packets, self.lbl_pos,
                  self.lbl_speed, self.lbl_dist, self.lbl_sensor):
            w.setStyleSheet("padding: 0 8px; font-family: Consolas, monospace;")
            bar.addPermanentWidget(w)

        # ---- 信号 ----
        self.btn_refresh.clicked.connect(self._refresh_ports)
        self.btn_conn.clicked.connect(self._toggle_connection)
        self.btn_clear.clicked.connect(self._clear)
        self.btn_calib.clicked.connect(self._open_calib)
        self.btn_pause.clicked.connect(self._toggle_pause)
        self.btn_follow.toggled.connect(self._on_follow_toggled)
        self.btn_csv.clicked.connect(self._save_csv)
        self.btn_shot.clicked.connect(self._save_screenshot)

    # ---------------- 串口 ----------------
    def _refresh_ports(self):
        self.combo_port.clear()
        if list_ports is None:
            return
        ports = list(list_ports.comports())
        for p in ports:
            desc = p.description or ""
            self.combo_port.addItem(f"{p.device}  {desc}", p.device)
        # 优先选 CH340（本项目的板子）
        for i in range(self.combo_port.count()):
            data = self.combo_port.itemData(i) or ""
            text = self.combo_port.itemText(i).upper()
            if "CH340" in text or "CH34" in text:
                self.combo_port.setCurrentIndex(i)
                break

    def _toggle_connection(self):
        if self.worker is not None:
            self._disconnect()
            return
        if self.demo_mode:
            QtWidgets.QMessageBox.information(self, "提示", "当前是演示模式，先关闭再连接串口。")
            return
        if serial is None:
            QtWidgets.QMessageBox.critical(self, "错误", "未安装 pyserial：pip install pyserial")
            return
        port = self.combo_port.currentData()
        if not port:
            QtWidgets.QMessageBox.warning(self, "提示", "没有可用串口，点【刷新】试试。")
            return
        baud = int(self.combo_baud.currentText())
        self.worker = SerialWorker(port, baud)
        self.worker.sample.connect(self.on_sample)
        self.worker.bad_frame.connect(self._on_bad_frame)
        self.worker.error.connect(self._on_serial_error)
        self.worker.start()
        self.btn_conn.setText("断开")
        self.lbl_conn.setText(f"● 已连接 {port} @ {baud}")
        self.lbl_conn.setStyleSheet("color:#27ae60; font-weight:bold;")

    def _disconnect(self):
        if self.worker is not None:
            self.worker.stop()
            self.worker = None
        self.btn_conn.setText("连接")
        self.lbl_conn.setText("● 未连接")
        self.lbl_conn.setStyleSheet("color:#c0392b; font-weight:bold;")

    def _on_serial_error(self, msg):
        QtWidgets.QMessageBox.critical(self, "串口错误", msg)
        self._disconnect()

    def _on_bad_frame(self):
        self.bad_frames += 1

    # ---------------- 演示模式 ----------------
    def _start_demo(self):
        self.demo_source = DemoSource(self.spin_height.value(), self.spin_k.value())
        self.demo_source.sample.connect(self.on_sample)
        self.demo_source.start()
        self.btn_conn.setEnabled(False)
        self.combo_port.setEnabled(False)
        self.combo_baud.setEnabled(False)
        self.lbl_conn.setText("● 演示模式（合成圆周轨迹）")
        self.lbl_conn.setStyleSheet("color:#2980b9; font-weight:bold;")

    # ---------------- 数据处理 ----------------
    def on_sample(self, rec):
        t_ms, dx, dy, motion, squal, shutter = rec
        self.packets += 1
        self.rate_t.append(t_ms)
        if motion:
            self.motion_frames += 1

        height = self.spin_height.value()
        k = self.spin_k.value()
        scale = height * k                    # mm / count
        sx = -1.0 if self.chk_flip_x.isChecked() else 1.0
        sy = -1.0 if self.chk_flip_y.isChecked() else 1.0

        dx_mm = dx * scale * sx
        dy_mm = dy * scale * sy

        self.count_x += dx
        self.count_y += dy
        self.x += dx_mm
        self.y += dy_mm
        self.distance += math.hypot(dx_mm, dy_mm)

        self.samples.append((t_ms, dx, dy, motion, squal, shutter, self.x, self.y))
        self.xs.append(self.x)
        self.ys.append(self.y)
        if len(self.xs) > MAX_POINTS:         # 长跑抽稀，控制内存与绘图开销
            self.xs = self.xs[::2]
            self.ys = self.ys[::2]
            self.samples = self.samples[::2]

        self.hist_t.append(t_ms / 1000.0)
        self.hist_dx.append(dx)
        self.hist_dy.append(dy)
        self.hist_sq.append(squal)
        self.hist_sh.append(shutter)
        self.speed_win.append((t_ms, dx_mm, dy_mm))
        self._dirty = True

    def _update_plots(self):
        if self.paused or not self._dirty:
            return
        self._dirty = False

        xs = np.asarray(self.xs)
        ys = np.asarray(self.ys)
        self.curve_traj.setData(xs, ys)
        self.dot_now.setData([self.x], [self.y])

        n = len(self.hist_t)
        if n >= 2:
            t = np.asarray(self.hist_t)
            x = t - t[-1]
            self.curve_dx.setData(x, np.asarray(self.hist_dx))
            self.curve_dy.setData(x, np.asarray(self.hist_dy))
            self.curve_sq.setData(x, np.asarray(self.hist_sq))
            self.curve_sh.setData(x, np.asarray(self.hist_sh))

        # 轨迹视图跟随（每 5 帧调整一次，避免抖动）
        self._follow_tick += 1
        if self.follow and self._follow_tick % 5 == 0 and len(xs) > 1:
            xmin, xmax = float(xs.min()), float(xs.max())
            ymin, ymax = float(ys.min()), float(ys.max())
            span = max(xmax - xmin, ymax - ymin, 10.0)
            pad = max(20.0, span * 0.15)
            cx, cy = (xmin + xmax) / 2, (ymin + ymax) / 2
            self.p_traj.setXRange(cx - span / 2 - pad, cx + span / 2 + pad, padding=0)
            self.p_traj.setYRange(cy - span / 2 - pad, cy + span / 2 + pad, padding=0)

    def _update_stats(self):
        rate = 0.0
        if len(self.rate_t) >= 2:
            span = (self.rate_t[-1] - self.rate_t[0]) / 1000.0
            if span > 0.2:
                rate = (len(self.rate_t) - 1) / span

        vx = vy = 0.0
        if len(self.speed_win) >= 2:
            t0 = self.speed_win[0][0]
            t1 = self.speed_win[-1][0]
            dt = (t1 - t0) / 1000.0
            if dt > 0.05:
                vx = sum(s[1] for s in self.speed_win) / dt
                vy = sum(s[2] for s in self.speed_win) / dt

        sq = self.hist_sq[-1] if self.hist_sq else None
        sh = self.hist_sh[-1] if self.hist_sh else None

        self.lbl_rate.setText(f"采样率: {rate:5.1f} Hz")
        self.lbl_packets.setText(f"包: {self.packets}  坏帧: {self.bad_frames}")
        self.lbl_pos.setText(f"坐标: ({self.x:8.2f}, {self.y:8.2f}) mm")
        self.lbl_speed.setText(f"速度: ({vx:7.1f}, {vy:7.1f}) mm/s")
        self.lbl_dist.setText(f"累计距离: {self.distance:9.1f} mm")
        self.lbl_sensor.setText(
            f"squal: {sq if sq is not None else '--'}  shutter: {sh if sh is not None else '--'}")

    # ---------------- 操作 ----------------
    def _clear(self):
        self.x = self.y = 0.0
        self.count_x = self.count_y = 0
        self.distance = 0.0
        self.xs.clear()
        self.ys.clear()
        self.samples.clear()
        self.hist_t.clear()
        self.hist_dx.clear()
        self.hist_dy.clear()
        self.hist_sq.clear()
        self.hist_sh.clear()
        self.speed_win.clear()
        self.rate_t.clear()
        self.packets = self.bad_frames = self.motion_frames = 0
        self.curve_traj.setData([], [])
        self.dot_now.setData([], [])
        self.p_traj.setXRange(-150, 150, padding=0)
        self.p_traj.setYRange(-150, 150, padding=0)
        self._dirty = True

    def _toggle_pause(self):
        self.paused = not self.paused
        self.btn_pause.setText("继续" if self.paused else "暂停")

    def _on_follow_toggled(self, checked):
        self.follow = checked
        self.btn_follow.setText("跟随：开" if checked else "跟随：关")

    def _open_calib(self):
        dialog = CalibDialog(self.spin_height.value(),
                             lambda: (self.count_x, self.count_y), self)
        if dialog.exec_() == QtWidgets.QDialog.Accepted and dialog.result_k:
            self.spin_k.setValue(dialog.result_k)
            QtWidgets.QMessageBox.information(
                self, "标定完成",
                f"K 已更新为 {dialog.result_k:.6f}\n（已写入界面，下次启动会保存）")

    def write_csv(self, path):
        """把已采集样本写成 CSV，返回行数（供按钮和自检共用）。"""
        with open(path, "w", newline="", encoding="utf-8-sig") as f:
            writer = csv.writer(f)
            writer.writerow(["t_ms", "dx", "dy", "motion",
                             "squal", "shutter", "x_mm", "y_mm"])
            writer.writerows(self.samples)
        return len(self.samples)

    def _save_csv(self):
        if not self.samples:
            QtWidgets.QMessageBox.information(self, "提示", "还没有数据可保存。")
            return
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "保存数据", "flow_data.csv", "CSV 文件 (*.csv)")
        if not path:
            return
        try:
            n = self.write_csv(path)
            QtWidgets.QMessageBox.information(self, "完成", "已保存 %d 行，文件：%s" % (n, path))
        except OSError as exc:
            QtWidgets.QMessageBox.critical(self, "保存失败", str(exc))

    def auto_capture(self, shot_path):
        """联调/CI 用：保存当前界面截图 + CSV，并打印统计。"""
        self._update_plots()
        self._update_stats()
        ok = self.grab().save(shot_path)
        csv_path = os.path.splitext(shot_path)[0] + ".csv"
        rows = self.write_csv(csv_path) if self.samples else 0
        rate = 0.0
        if len(self.rate_t) >= 2:
            span = (self.rate_t[-1] - self.rate_t[0]) / 1000.0
            if span > 0.1:
                rate = (len(self.rate_t) - 1) / span
        print(f"CAPTURE_{'OK' if ok else 'FAIL'} {shot_path} packets={self.packets} "
              f"bad={self.bad_frames} rate={rate:.1f}Hz rows={rows} "
              f"distance={self.distance:.1f}mm")
        return ok

    def _save_screenshot(self):
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "保存截图", "flow_trajectory.png", "PNG 图片 (*.png)")
        if not path:
            return
        pix = self.grab()
        if not pix.save(path):
            QtWidgets.QMessageBox.critical(self, "失败", "保存图片失败。")
        else:
            QtWidgets.QMessageBox.information(self, "完成", f"截图已保存：\n{path}")

    # ---------------- 设置持久化 ----------------
    def _load_settings(self):
        s = QtCore.QSettings("pmw3901", "flow_host")
        try:
            h = float(s.value("height", 100.0))
            k = float(s.value("k", DEFAULT_K))
            self.spin_height.setValue(h)
            self.spin_k.setValue(k)
            self.chk_flip_x.setChecked(str(s.value("flip_x", "false")).lower() == "true")
            self.chk_flip_y.setChecked(str(s.value("flip_y", "false")).lower() == "true")
            self.combo_baud.setCurrentText(str(s.value("baud", "115200")))
        except (TypeError, ValueError):
            pass

    def _save_settings(self):
        s = QtCore.QSettings("pmw3901", "flow_host")
        s.setValue("height", self.spin_height.value())
        s.setValue("k", self.spin_k.value())
        s.setValue("flip_x", self.chk_flip_x.isChecked())
        s.setValue("flip_y", self.chk_flip_y.isChecked())
        s.setValue("baud", self.combo_baud.currentText())
        s.sync()

    def closeEvent(self, event):        # noqa: N802
        self._save_settings()
        if self.worker is not None:
            self.worker.stop()
        if self.demo_source is not None:
            self.demo_source.stop()
        event.accept()


# ---------------------------------------------------------------------------
# 自检：喂一段合成数据，出图（开发/CI 用，不需要硬件）
# ---------------------------------------------------------------------------
def run_selftest(out_path, seconds=10.5):
    app = QtWidgets.QApplication(sys.argv)
    win = MainWindow(demo=False)
    win.resize(1500, 900)
    win.show()
    win.spin_height.setValue(100.0)
    win.spin_k.setValue(DEFAULT_K)

    # 协议解析自测（正常帧 / 校验错 / 垃圾数据）
    body = b"FLOW,1234,1,-2,1,200,2800"
    good = b"$" + body + ("*%02X" % _xor8(body)).encode() + bytes([10])
    assert parse_flow_line(good) == (1234, 1, -2, 1, 200, 2800)
    assert parse_flow_line(b"$FLOW,1,2,3,4,5*garbage") is None
    assert parse_flow_line(b"$FLOW,1,2,3,4,5*00") is None
    print("PROTOCOL_SELFTEST_OK")

    for rec in circle_samples(seconds, 100.0, DEFAULT_K):
        win.on_sample(rec)
    win._update_plots()
    win._update_stats()
    for _ in range(5):
        app.processEvents()
    path = os.path.abspath(out_path)
    ok = win.grab().save(path)
    csv_path = os.path.splitext(path)[0] + ".csv"
    rows = win.write_csv(csv_path)
    rate = 0.0
    if len(win.rate_t) >= 2:
        span = (win.rate_t[-1] - win.rate_t[0]) / 1000.0
        if span > 0.1:
            rate = (len(win.rate_t) - 1) / span
    print(f"SELFTEST_{'OK' if ok else 'FAIL'} {path} samples={len(win.samples)} "
          f"packets={win.packets} rate={rate:.1f}Hz distance_mm={win.distance:.1f} "
          f"csv_rows={rows} csv={csv_path}")
    win.close()
    return 0 if (ok and rows == len(win.samples)) else 1


def main():
    parser = argparse.ArgumentParser(description=APP_NAME)
    parser.add_argument("--port", help="启动后自动连接该串口，例如 COM6")
    parser.add_argument("--demo", action="store_true", help="演示模式（合成数据，不接板子）")
    parser.add_argument("--selftest", metavar="PNG", help="自检：生成轨迹截图后退出（开发用）")
    parser.add_argument("--capture", type=float, metavar="SECONDS",
                        help="连接后采集 N 秒，自动截图并退出（联调用）")
    parser.add_argument("--shot", default="flow_capture.png",
                        help="--capture 的截图输出路径")
    args = parser.parse_args()

    if args.selftest:
        sys.exit(run_selftest(args.selftest))

    app = QtWidgets.QApplication(sys.argv)
    app.setApplicationName(APP_NAME)
    win = MainWindow(demo=args.demo, auto_port=args.port)
    win.show()

    if args.capture:
        def _do_capture():
            win.auto_capture(args.shot)
            app.quit()
        QtCore.QTimer.singleShot(int(args.capture * 1000), _do_capture)

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
