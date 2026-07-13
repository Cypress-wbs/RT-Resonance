import argparse
import re
import sys
import time
from pathlib import Path

import numpy as np
import pyqtgraph as pg
from PyQt5.QtCore import QThread, QTimer, pyqtSignal, Qt
from PyQt5.QtWidgets import (QApplication, QLabel, QMainWindow, QVBoxLayout, 
                             QHBoxLayout, QWidget, QFrame, QGridLayout)
from PyQt5.QtGui import QFont
from pyocd.core.helpers import ConnectHelper

# ==================== [符号解析表 (基于 Y 轴)] ====================
SYMBOL_NAMES = [
    "g_raw_gy",
    "g_filtered_gy",
    "g_RFP1_raw",
    "g_RFP1_filtered",
    "g_RFP1_tension_pct",
    "g_RFP1_min",
    "g_RFP1_max",
    "g_RFP1_feature_mean",
    "g_RFP1_feature_var",
    "g_RFP2_raw",
    "g_RFP2_filtered",
    "g_RFP2_tension_pct",
    "g_RFP2_min",
    "g_RFP2_max",
    "g_RFP2_feature_mean",
    "g_RFP2_feature_var",
    "g_RFP_calib_state",
    "g_vibrato_freq_x100",
    "g_vibrato_amplitude",
]

def default_map_path():
    return Path(__file__).resolve().parents[1] / "Debug" / "rtthread.map"

def parse_symbol_addresses(map_path):
    symbol_pattern = re.compile(r"^\s*0x([0-9a-fA-F]+)\s+([A-Za-z_][A-Za-z0-9_]*)\s*$")
    addresses = {}
    with open(map_path, "r", encoding="utf-8", errors="ignore") as fp:
        for line in fp:
            match = symbol_pattern.match(line)
            if not match: continue
            address = int(match.group(1), 16)
            name = match.group(2)
            if name in SYMBOL_NAMES:
                addresses[name] = address
    missing = [name for name in SYMBOL_NAMES if name not in addresses]
    if missing:
        raise RuntimeError(f"Map 文件缺少符号: {', '.join(missing)}\n请确认是否在 C 代码中加了 __attribute__((used)) 并重新编译。")
    return addresses

def RFP_calib_text(state):
    mapping = {
        -1: "ADC_ERROR (硬件故障)",
        0: "WAITING_RELAX (请保持手部放松)",
        1: "COLLECTING_RELAX (正在采集放松基准...)",
        2: "WAITING_SQUEEZE (请用力捏紧并保持!)",
        3: "COLLECTING_SQUEEZE (正在采集发力极限...)",
        4: "SYSTEM_READY (系统就绪, AI 监控中)",
    }
    return mapping.get(state, f"UNKNOWN_STATE({state})")

# ==================== [ST-Link 后台通信线程] ====================
class MemoryReaderThread(QThread):
    data_received = pyqtSignal(object)
    status_changed = pyqtSignal(str)

    def __init__(self, symbol_addresses, target_override, sample_interval_s=0.005):
        super().__init__()
        self.running = True
        self.symbol_addresses = symbol_addresses
        self.target_override = target_override
        self.sample_interval_s = sample_interval_s

    @staticmethod
    def _to_signed_int32(value):
        value &= 0xFFFFFFFF
        return value if value < 0x80000000 else value - 0x100000000

    def run(self):
        try:
            self.status_changed.emit("正在寻找 ST-Link / pyOCD 目标...")
            with ConnectHelper.session_with_chosen_probe(target_override=self.target_override) as session:
                target = session.board.target
                try:
                    target.resume()
                    time.sleep(0.05)
                    self.status_changed.emit("连接成功！200Hz 内存穿透通道已建立。")
                except Exception as exc:
                    self.status_changed.emit(f"已连接，但内核恢复失败: {exc}")

                while self.running:
                    try:
                        if "halt" in str(target.get_state()).lower():
                            target.resume()
                    except: pass

                    sample = {}
                    for name, address in self.symbol_addresses.items():
                        value = target.read32(address)
                        sample[name] = float(self._to_signed_int32(value))

                    self.data_received.emit(sample)
                    time.sleep(self.sample_interval_s)
        except Exception as exc:
            self.status_changed.emit(f"ST-Link 连接断开: {exc}")

    def stop(self):
        self.running = False
        self.wait()

# ==================== [创客风 UI 主窗口] ====================
class MainWindow(QMainWindow):
    def __init__(self, map_path, symbol_addresses, target_override):
        super().__init__()
        self.setWindowTitle("智能弦乐边缘 AI 动作评估系统")
        self.resize(1300, 850)
        
        # 深色创客风背景
        self.setStyleSheet("QMainWindow { background-color: #0D0D12; }")

        # 数据缓冲区
        self.max_points = 500
        self.gyro_max_points = 150  # 减小陀螺仪窗口长度(0.75秒)，使波形展开、滚动加快
        self.gyro_raw_buffer = np.zeros(self.gyro_max_points)
        self.gyro_lpf_buffer = np.zeros(self.gyro_max_points)
        self.RFP1_lpf_buffer = np.zeros(self.max_points)
        self.RFP1_tension_buffer = np.zeros(self.max_points)
        self.RFP2_lpf_buffer = np.zeros(self.max_points)
        self.RFP2_tension_buffer = np.zeros(self.max_points)

        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        main_layout.setContentsMargins(15, 15, 15, 15)
        main_layout.setSpacing(15)

        # ---------------- [顶部：赛博风标题栏] ----------------
        title_frame = QFrame()
        title_frame.setStyleSheet("background-color: #161620; border: 1px solid #00E5FF; border-radius: 8px;")
        title_layout = QHBoxLayout(title_frame)
        
        self.title_label = QLabel("🚀 智能弦乐边缘 AI 动作评估系统 - 实时监控终端")
        self.title_label.setStyleSheet("color: #00E5FF; font-size: 22px; font-weight: bold; font-family: 'Consolas';")
        self.status_label = QLabel("Status: 初始化中...")
        self.status_label.setStyleSheet("color: #FF007F; font-size: 16px; font-weight: bold;")
        self.status_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        
        title_layout.addWidget(self.title_label)
        title_layout.addWidget(self.status_label)
        main_layout.addWidget(title_frame)

        # ---------------- [中下部：分栏布局] ----------------
        content_layout = QHBoxLayout()
        main_layout.addLayout(content_layout)

        # ====== 左侧栏：RFP 双通道监控 ======
        left_layout = QVBoxLayout()
        
        # 【修改点】：统一替换为“食指指腹”与“拇指指腹”
        self.RFP1_plot = self.create_plot_widget("RFP 1: 食指指腹发力监控", "Percent (%)")
        self.RFP1_lpf_curve = self.RFP1_plot.plot(pen=pg.mkPen("#444444", width=2, style=pg.QtCore.Qt.DotLine), name="Raw Avg")
        self.RFP1_tension_curve = self.RFP1_plot.plot(pen=pg.mkPen("#FF007F", width=2), name="Tension %")
        
        self.RFP2_plot = self.create_plot_widget("RFP 2: 拇指指腹发力监控", "Percent (%)")
        self.RFP2_lpf_curve = self.RFP2_plot.plot(pen=pg.mkPen("#444444", width=2, style=pg.QtCore.Qt.DotLine), name="Raw Avg")
        self.RFP2_tension_curve = self.RFP2_plot.plot(pen=pg.mkPen("#00E5FF", width=2), name="Tension %")
        
        left_layout.addWidget(self.RFP1_plot)
        left_layout.addWidget(self.RFP2_plot)
        
        # ====== 右侧栏：MPU 运动学与 AI 判定板 ======
        right_layout = QVBoxLayout()
        
        self.gyro_plot = self.create_plot_widget("MPU6050 (Y轴): 揉弦律动监控", "Angular Velocity")
        self.gyro_raw_curve = self.gyro_plot.plot(pen=pg.mkPen("#444444", width=1), name="Raw")
        self.gyro_lpf_curve = self.gyro_plot.plot(pen=pg.mkPen("#39FF14", width=2), name="Filtered")
        right_layout.addWidget(self.gyro_plot, stretch=2)
        
        # === AI 判定数据大屏 ===
        self.ai_panel = QFrame()
        self.ai_panel.setStyleSheet("""
            QFrame { background-color: #12121A; border: 2px solid #39FF14; border-radius: 8px; }
            QLabel { font-family: 'Consolas'; color: #FFFFFF; font-size: 16px; border: none; }
        """)
        ai_layout = QGridLayout(self.ai_panel)
        
        panel_title = QLabel("⚡ EDGE AI 实时动作诊断")
        panel_title.setStyleSheet("color: #39FF14; font-size: 20px; font-weight: bold; margin-bottom: 10px;")
        ai_layout.addWidget(panel_title, 0, 0, 1, 2)

        # 实时数据标签 【修改点】：文本全替换
        self.lbl_freq = QLabel("揉弦频率: 0.00 Hz")
        self.lbl_amp = QLabel("动作幅度: 0")
        self.lbl_f1_info = QLabel("食指指腹: 0% (方差: 0)")
        self.lbl_f2_info = QLabel("拇指指腹: 0% (方差: 0)")
        
        ai_layout.addWidget(self.lbl_freq, 1, 0)
        ai_layout.addWidget(self.lbl_amp, 1, 1)
        ai_layout.addWidget(self.lbl_f1_info, 2, 0)
        ai_layout.addWidget(self.lbl_f2_info, 2, 1)

        # 核心 AI 判定结论大字号
        self.lbl_ai_vibrato = QLabel("律动评估: 等待采集...")
        self.lbl_ai_vibrato.setStyleSheet("color: #FFFF00; font-size: 18px; font-weight: bold; padding-top: 10px;")
        self.lbl_ai_tension = QLabel("肌肉评估: 等待采集...")
        self.lbl_ai_tension.setStyleSheet("color: #FF007F; font-size: 18px; font-weight: bold;")
        
        ai_layout.addWidget(self.lbl_ai_vibrato, 3, 0, 1, 2)
        ai_layout.addWidget(self.lbl_ai_tension, 4, 0, 1, 2)

        right_layout.addWidget(self.ai_panel, stretch=1)

        # 加入主结构
        content_layout.addLayout(left_layout, stretch=1)
        content_layout.addLayout(right_layout, stretch=1)

        # ---------------- [启动后台] ----------------
        self.reader_thread = MemoryReaderThread(symbol_addresses=symbol_addresses, target_override=target_override)
        self.reader_thread.data_received.connect(self.update_buffer)
        self.reader_thread.status_changed.connect(self.status_label.setText)
        self.reader_thread.start()

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(20)

    def create_plot_widget(self, title, ylabel):
        p = pg.PlotWidget(title=title)
        p.setBackground('#161620')
        p.showGrid(x=True, y=True, alpha=0.3)
        p.setLabel("left", ylabel)
        p.getAxis('bottom').setPen(pg.mkPen('#00E5FF'))
        p.getAxis('left').setPen(pg.mkPen('#00E5FF'))
        p.addLegend(offset=(10, 10))
        return p

    def update_buffer(self, sample):
        # 环形缓冲区压入数据
        self.gyro_raw_buffer[:-1] = self.gyro_raw_buffer[1:]
        self.gyro_raw_buffer[-1] = sample["g_raw_gy"]
        self.gyro_lpf_buffer[:-1] = self.gyro_lpf_buffer[1:]
        self.gyro_lpf_buffer[-1] = sample["g_filtered_gy"]

        self.RFP1_lpf_buffer[:-1] = self.RFP1_lpf_buffer[1:]
        self.RFP1_lpf_buffer[-1] = sample["g_RFP1_filtered"]
        self.RFP1_tension_buffer[:-1] = self.RFP1_tension_buffer[1:]
        self.RFP1_tension_buffer[-1] = sample["g_RFP1_tension_pct"]

        self.RFP2_lpf_buffer[:-1] = self.RFP2_lpf_buffer[1:]
        self.RFP2_lpf_buffer[-1] = sample["g_RFP2_filtered"]
        self.RFP2_tension_buffer[:-1] = self.RFP2_tension_buffer[1:]
        self.RFP2_tension_buffer[-1] = sample["g_RFP2_tension_pct"]

        # ==================== [边缘 AI 逻辑裁决引擎] ====================
        calib_state = int(sample["g_RFP_calib_state"])
        freq_hz = sample["g_vibrato_freq_x100"] / 100.0
        amp = int(sample["g_vibrato_amplitude"])
        f1_pct = int(sample["g_RFP1_tension_pct"])
        f1_var = int(sample["g_RFP1_feature_var"])
        f2_pct = int(sample["g_RFP2_tension_pct"])
        f2_var = int(sample["g_RFP2_feature_var"])
        
        self.status_label.setText(f"状态标志: {RFP_calib_text(calib_state)}")
        
        if calib_state == 4: 
            self.lbl_freq.setText(f"揉弦频率: {freq_hz:4.2f} Hz")
            self.lbl_amp.setText(f"动作幅度: {amp}")
            # 【修改点】：文本全替换为指腹，并补齐双通道方差监控
            self.lbl_f1_info.setText(f"食指指腹: {f1_pct}% (方差: {f1_var})")
            self.lbl_f2_info.setText(f"拇指指腹: {f2_pct}% (方差: {f2_var})")

            # 1. 揉弦律动诊断逻辑
            if freq_hz == 0:
                self.lbl_ai_vibrato.setText("律动评估: ⚪ 未检测到明显揉弦动作")
                self.lbl_ai_vibrato.setStyleSheet("color: #AAAAAA;")
            elif freq_hz < 4.0:
                self.lbl_ai_vibrato.setText("律动评估: 🐢 揉弦过慢，注意小臂带动手腕的惯性")
                self.lbl_ai_vibrato.setStyleSheet("color: #00E5FF;")
            elif 4.0 <= freq_hz <= 7.0:
                self.lbl_ai_vibrato.setText("律动评估: 🌟 完美！频率落在黄金大师区间")
                self.lbl_ai_vibrato.setStyleSheet("color: #39FF14;") 
            else:
                self.lbl_ai_vibrato.setText("律动评估: ⚠️ 频率过快(神经性颤动)，请深呼吸放缓")
                self.lbl_ai_vibrato.setStyleSheet("color: #FF007F;")

            # 2. 肌肉紧张度诊断逻辑
            if f1_pct > 80 or f2_pct > 80:
                self.lbl_ai_tension.setText("肌肉评估: ❌ 死力警告！手指过度紧绷！")
                self.lbl_ai_tension.setStyleSheet("color: #FF0000; font-weight: bold;")
            elif f1_var > 5000 or f2_var > 5000:
                self.lbl_ai_tension.setText("肌肉评估: ⚠️ 肌肉不受控发抖 (方差过大)，请放松")
                self.lbl_ai_tension.setStyleSheet("color: #FFFF00;")
            elif f1_pct < 10 and f2_pct < 10:
                self.lbl_ai_tension.setText("肌肉评估: ⚪ 琴弦未贴合 (脱手脱弓状态)")
                self.lbl_ai_tension.setStyleSheet("color: #AAAAAA;")
            else:
                self.lbl_ai_tension.setText("肌肉评估: ✅ 握力均匀，肌肉状态自然放松")
                self.lbl_ai_tension.setStyleSheet("color: #39FF14;")

    def update_plot(self):
        self.gyro_raw_curve.setData(self.gyro_raw_buffer)
        self.gyro_lpf_curve.setData(self.gyro_lpf_buffer)
        
        gyro_combined = np.concatenate((self.gyro_raw_buffer, self.gyro_lpf_buffer))
        gyro_low, gyro_high = float(np.min(gyro_combined)), float(np.max(gyro_combined))
        gyro_span = max(gyro_high - gyro_low, 1000.0)
        self.gyro_plot.setYRange(gyro_low - gyro_span * 0.2, gyro_high + gyro_span * 0.2, padding=0)

        # RFP 固定 0 - 100% 高度 (把原始值除以22强行压缩在同一视图里，完美适配 RFP 传感器 2200 的最大值)
        self.RFP1_lpf_curve.setData(self.RFP1_lpf_buffer / 22.0) 
        self.RFP1_tension_curve.setData(self.RFP1_tension_buffer)
        self.RFP1_plot.setYRange(-5, 110, padding=0)

        self.RFP2_lpf_curve.setData(self.RFP2_lpf_buffer / 22.0)
        self.RFP2_tension_curve.setData(self.RFP2_tension_buffer)
        self.RFP2_plot.setYRange(-5, 110, padding=0)

    def closeEvent(self, event):
        self.reader_thread.stop()
        event.accept()

def parse_args():
    parser = argparse.ArgumentParser(description="ST-Link 边缘 AI 数据大屏")
    parser.add_argument("--map", default=str(default_map_path()), help="指向 rtthread.map 的路径")
    parser.add_argument("--target", default="cortex_m", help="pyOCD target override")
    return parser.parse_args()

def main():
    args = parse_args()
    map_path = Path(args.map)
    symbol_addresses = parse_symbol_addresses(map_path)

    pg.setConfigOption("background", "#161620")
    pg.setConfigOption("foreground", "#FFFFFF")

    app = QApplication(sys.argv)
    win = MainWindow(map_path=map_path, symbol_addresses=symbol_addresses, target_override=args.target)
    win.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
