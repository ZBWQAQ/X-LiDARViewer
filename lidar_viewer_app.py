"""
Camsense X1 LiDAR Upper Computer
Features: Serial connection, polar/Cartesian point cloud, data stats, DAT replay
"""
import sys, os, time, math, struct, csv, threading
from typing import Optional
import serial
import serial.tools.list_ports
import numpy as np
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QComboBox, QPushButton, QLabel, QGroupBox, QSplitter,
    QTableWidget, QTableWidgetItem, QHeaderView, QTextEdit,
    QFileDialog, QTabWidget, QFrame, QSpinBox, QProgressBar, QCheckBox,
    QLineEdit
)
from PyQt6.QtCore import Qt, QTimer, QThread, pyqtSignal, QPointF, QRectF
from PyQt6.QtGui import QFont, QColor, QPainter, QPolygonF, QLinearGradient
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.pyplot as plt
from generic_parser import (
    GenericLidarParser, ConfigManager, LidarFrame, LidarScan, LidarPoint
)
from tfmini_plus_protocol import (
    TFminiPlusParser, TFminiPlusCommandBuilder, TFminiPlusResponseParser,
    TFMINI_PLUS_PRESET_COMMANDS, FPS_PRESETS, BAUDRATE_PRESETS,
    TFminiPlusDataPoint
)
POINTS_PER_FRAME = 8  # 保留向后兼容


class SerialReaderThread(QThread):
    frame_received = pyqtSignal(object)
    error_occurred = pyqtSignal(str)
    connection_lost = pyqtSignal()
    tfmini_data_received = pyqtSignal(object)  # TFminiPlusDataPoint
    tfmini_config_response = pyqtSignal(object)  # config response bytes

    def __init__(self, port, baudrate=115200, protocol_config=None, profile_key=''):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self._running = False
        self._serial = None
        self._profile_key = profile_key
        self._tfmini_parser = TFminiPlusParser() if profile_key == 'tfmini_plus' else None
        if profile_key != 'tfmini_plus':
            self._parser = GenericLidarParser(protocol_config or {})
        else:
            self._parser = None

    def send_command(self, cmd_bytes: bytes):
        """发送配置命令到串口"""
        if self._serial and self._serial.is_open:
            try:
                self._serial.write(cmd_bytes)
            except Exception as e:
                self.error_occurred.emit(f"Send failed: {e}")

    def run(self):
        try:
            self._serial = serial.Serial(
                port=self.port, baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE, timeout=0.1)
            self._running = True
            while self._running:
                if self._serial and self._serial.is_open and self._serial.in_waiting > 0:
                    raw = self._serial.read(self._serial.in_waiting)
                    if self._tfmini_parser is not None:
                        data_points, config_responses = self._tfmini_parser.feed_data(raw)
                        for dp in data_points:
                            self.tfmini_data_received.emit(dp)
                        for cr in config_responses:
                            self.tfmini_config_response.emit(cr)
                    elif self._parser is not None:
                        for frame in self._parser.feed_data(raw):
                            self.frame_received.emit(frame)
                else:
                    self.msleep(5)
        except serial.SerialException as e:
            self.error_occurred.emit(str(e))
        except Exception as e:
            self.error_occurred.emit(str(e))
        finally:
            self.connection_lost.emit()
            if self._serial and self._serial.is_open:
                try: self._serial.close()
                except: pass

    def stop(self):
        self._running = False
        if self._serial and self._serial.is_open:
            try: self._serial.close()
            except: pass


class FileReaderThread(QThread):
    frame_received = pyqtSignal(object)
    tfmini_data_received = pyqtSignal(object)
    finished_signal = pyqtSignal()
    progress = pyqtSignal(int)
    frame_position = pyqtSignal(int, int)  # (current, total)
    format_mismatch = pyqtSignal(str)  # 格式不匹配时发出

    def __init__(self, filepath, speed_mult=1.0, protocol_config=None, profile_key=''):
        super().__init__()
        self.filepath = filepath
        self.speed_mult = speed_mult
        self._running = False
        self._paused = False
        self._pause_event = threading.Event()
        self._protocol_config = protocol_config or {}
        self._profile_key = profile_key
        self.frames = []       # 预加载的帧列表
        self._frame_idx = 0    # 当前播放索引
        self.loaded = False    # 是否已加载完成
        self.detected_format = ''  # 检测到的文件格式

    def load_frames(self):
        """预加载所有帧到内存，自动检测文件格式"""
        fsize = os.path.getsize(self.filepath)
        if fsize == 0:
            self.loaded = True
            return
        # 读取前4096字节检测格式
        with open(self.filepath, 'rb') as f:
            head = f.read(min(4096, fsize))
        # 检测格式
        if self._detect_format(head):
            self._load_tfmini_frames()
        else:
            self._load_generic_frames()
        self.loaded = True

    def _detect_format(self, data):
        """检测文件格式，返回True表示TFmini Plus格式"""
        # TFmini Plus: 寻找连续的 0x59 0x59 帧头
        for i in range(len(data) - 8):
            if data[i] == 0x59 and data[i + 1] == 0x59:
                # 验证checksum
                frame = data[i:i + 9]
                ck = sum(frame[:8]) & 0xFF
                if ck == frame[8]:
                    return True
        return False

    def _load_generic_frames(self):
        """加载通用(Generic/Camsense X1等)格式的DAT"""
        self.detected_format = 'generic'
        parser = GenericLidarParser(self._protocol_config)
        fsize = os.path.getsize(self.filepath)
        bread = 0
        try:
            with open(self.filepath, 'rb') as f:
                while True:
                    chunk = f.read(4096)
                    if not chunk:
                        break
                    bread += len(chunk)
                    for frame in parser.feed_data(chunk):
                        self.frames.append(frame)
                    self.progress.emit(int(bread / fsize * 100))
        except: pass

    def _load_tfmini_frames(self):
        """加载TFmini Plus格式的DAT"""
        self.detected_format = 'tfmini_plus'
        parser = TFminiPlusParser()
        fsize = os.path.getsize(self.filepath)
        bread = 0
        try:
            with open(self.filepath, 'rb') as f:
                while True:
                    chunk = f.read(4096)
                    if not chunk:
                        break
                    bread += len(chunk)
                    dps, _ = parser.feed_data(chunk)
                    for dp in dps:
                        self.frames.append(dp)
                    self.progress.emit(int(bread / fsize * 100))
        except: pass

    def run(self):
        self._running = True
        self._paused = True   # 加载后默认暂停，不自动播放
        self._frame_idx = 0
        # 阶段1：预加载所有帧（带进度）
        self.load_frames()
        # 加载完成，立即发送帧位置（触发进度条切换为帧数模式）
        self.frame_position.emit(0, len(self.frames))
        # 检查格式是否匹配
        if self._profile_key == 'tfmini_plus' and self.detected_format != 'tfmini_plus':
            self.format_mismatch.emit(
                f"文件格式不匹配：当前配置为 TFmini Plus，但DAT文件为通用雷达格式。\n请切换到对应的雷达配置后再加载。")
            self.loaded = True
            return
        if self._profile_key != 'tfmini_plus' and self.detected_format == 'tfmini_plus':
            self.format_mismatch.emit(
                f"文件格式不匹配：当前配置为通用雷达，但DAT文件为 TFmini Plus 格式。\n请切换到 TFmini Plus 配置后再加载。")
            self.loaded = True
            return
        # 阶段2：逐帧回放
        while self._running and self._frame_idx < len(self.frames):
            self._pause_event.wait()
            if not self._running:
                break
            frame = self.frames[self._frame_idx]
            self._frame_idx += 1
            if self.detected_format == 'tfmini_plus':
                self.tfmini_data_received.emit(frame)
            else:
                self.frame_received.emit(frame)
            self.frame_position.emit(self._frame_idx, len(self.frames))
            if self.speed_mult > 0:
                time.sleep(0.001 / max(self.speed_mult, 0.01))
        self.finished_signal.emit()

    def step_one(self):
        """手动前进一帧（暂停状态下使用，不触发frame_received以避免污染统计）"""
        if self.loaded and self._paused and self._frame_idx < len(self.frames):
            self._frame_idx += 1
            self.frame_position.emit(self._frame_idx, len(self.frames))
            return True
        return False

    def step_back(self):
        """手动后退一帧（暂停状态下使用，不触发frame_received以避免污染统计）"""
        if self.loaded and self._paused and self._frame_idx > 0:
            self._frame_idx -= 1
            self.frame_position.emit(self._frame_idx, len(self.frames))
            return True
        return False

    @property
    def is_finished(self):
        return self.loaded and self._frame_idx >= len(self.frames)

    def reset_position(self):
        """重置播放位置到开头（保留已加载帧数据）"""
        if self.loaded:
            self._paused = True
            self._pause_event.clear()
            self._frame_idx = 0
            self.frame_position.emit(0, len(self.frames))

    def pause(self):
        self._paused = True
        self._pause_event.clear()

    def resume(self):
        self._paused = False
        self._pause_event.set()

    def stop(self):
        self._running = False
        self._pause_event.set()  # 解除暂停以便线程退出


class TriangleButton(QPushButton):
    """自定义带白色实心三角形的按钮，支持方向('up'/'down')"""
    _BASE_COLOR = QColor(40, 70, 100)
    _HOVER_COLOR = QColor(0, 212, 255)
    _PRESSED_COLOR = QColor(20, 50, 75)

    def __init__(self, direction="up", parent=None):
        super().__init__(parent)
        self._direction = direction
        self._hovered = False
        self._pressed = False
        self.setFixedSize(24, 15)
        self.setCursor(Qt.CursorShape.PointingHandCursor)
        self.setMouseTracking(True)

    def _base_color(self):
        return self._BASE_COLOR

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()

        # 背景渐变
        base = self._HOVER_COLOR if self._hovered else (self._PRESSED_COLOR if self._pressed else self._base_color())
        grad = QLinearGradient(0, 0, 0, h)
        grad.setColorAt(0, base.lighter(115))
        grad.setColorAt(1, base.darker(115))
        painter.setBrush(grad)
        painter.setPen(Qt.PenStyle.NoPen)

        # 圆角矩形背景
        painter.drawRoundedRect(QRectF(0, 0, w, h), 5, 5)

        # 绘制白色实心三角形
        painter.setBrush(QColor(255, 255, 255))
        painter.setPen(Qt.PenStyle.NoPen)
        margin_x = w * 0.25
        margin_y = h * 0.2
        cx = w / 2.0

        if self._direction == "up":
            points = [QPointF(cx, margin_y),
                      QPointF(w - margin_x, h - margin_y),
                      QPointF(margin_x, h - margin_y)]
        else:
            points = [QPointF(cx, h - margin_y),
                      QPointF(w - margin_x, margin_y),
                      QPointF(margin_x, margin_y)]

        painter.drawPolygon(QPolygonF(points))
        painter.end()

    def enterEvent(self, event):
        self._hovered = True
        self.update()

    def leaveEvent(self, event):
        self._hovered = False
        self._pressed = False
        self.update()

    def mousePressEvent(self, event):
        self._pressed = True
        self.update()
        super().mousePressEvent(event)

    def mouseReleaseEvent(self, event):
        self._pressed = False
        self.update()
        super().mouseReleaseEvent(event)


class PolarCanvas(FigureCanvas):
    def __init__(self, parent=None, dpi=100):
        self.fig = Figure(dpi=dpi, facecolor='#1a1a2e')
        self.ax = self.fig.add_subplot(111, polar=True)
        self.ax.set_facecolor('#16213e')
        super().__init__(self.fig)
        self.setParent(parent)
        self._setup()

    def _setup(self):
        self.ax.set_theta_zero_location('N')
        self.ax.set_theta_direction(-1)
        self.ax.tick_params(colors='#aaaaaa', labelsize=8)
        self.ax.grid(True, color='#333355', alpha=0.5, linewidth=0.5)
        self.ax.set_rlabel_position(135)
        self.fig.subplots_adjust(left=0.05, right=0.95, top=0.95, bottom=0.05)

    def update_data(self, scan, max_range=1000):
        self.ax.clear()
        self._setup()
        self.ax.set_rlim(0, max_range)
        step = max_range / 5
        ticks = [round(step * i) for i in range(1, 6)]
        self.ax.set_rticks(ticks)
        if scan and scan.valid_points:
            angles = np.array([math.radians(p.angle) for p in scan.valid_points])
            distances = np.array([p.distance for p in scan.valid_points])
            qualities = np.array([p.quality for p in scan.valid_points])
            norm = plt.Normalize(vmin=0, vmax=255)
            colors = plt.cm.viridis(norm(qualities))
            self.ax.scatter(angles, distances, c=colors, s=4, alpha=0.8, edgecolors='none')
            self.ax.plot(angles, distances, color='#00d4ff', linewidth=0.5, alpha=0.3)
        self.draw()
        self.flush_events()


class CartesianCanvas(FigureCanvas):
    def __init__(self, parent=None, dpi=100):
        self.fig = Figure(dpi=dpi, facecolor='#1a1a2e')
        self.ax = self.fig.add_subplot(111)
        self.ax.set_facecolor('#16213e')
        super().__init__(self.fig)
        self.setParent(parent)
        self._setup()

    def _setup(self):
        self.ax.set_aspect('equal')
        self.ax.tick_params(colors='#aaaaaa', labelsize=8)
        self.ax.grid(True, color='#333355', alpha=0.5, linewidth=0.5)
        self.ax.set_xlabel('X (mm)', color='#aaaaaa', fontsize=9)
        self.ax.set_ylabel('Y (mm)', color='#aaaaaa', fontsize=9)
        self.fig.subplots_adjust(left=0.1, right=0.95, top=0.95, bottom=0.1)

    def update_data(self, scan, max_range=1000):
        self.ax.clear()
        self._setup()
        r = max_range * 1.1
        if scan and scan.valid_points:
            x = np.array([p.x for p in scan.valid_points])
            y = np.array([p.y for p in scan.valid_points])
            qualities = np.array([p.quality for p in scan.valid_points])
            norm = plt.Normalize(vmin=0, vmax=255)
            colors = plt.cm.viridis(norm(qualities))
            self.ax.scatter(x, y, c=colors, s=4, alpha=0.8, edgecolors='none')
            self.ax.plot(x, y, color='#00d4ff', linewidth=0.5, alpha=0.3)
        self.ax.set_xlim(-r, r)
        self.ax.set_ylim(-r, r)
        self.draw()
        self.flush_events()


class HistogramCanvas(FigureCanvas):
    def __init__(self, parent=None, dpi=100):
        self.fig = Figure(dpi=dpi, facecolor='#1a1a2e')
        self.ax = self.fig.add_subplot(111)
        self.ax.set_facecolor('#16213e')
        super().__init__(self.fig)
        self.setParent(parent)
        self._setup()

    def _setup(self):
        self.ax.tick_params(colors='#aaaaaa', labelsize=8)
        self.ax.grid(True, color='#333355', alpha=0.5, linewidth=0.5, axis='y')
        self.ax.set_xlabel('Distance (mm)', color='#aaaaaa', fontsize=9)
        self.ax.set_ylabel('Count', color='#aaaaaa', fontsize=9)
        self.fig.subplots_adjust(left=0.15, right=0.95, top=0.92, bottom=0.15)

    def update_data(self, scan, max_range=1000):
        self.ax.clear()
        self._setup()
        if scan and scan.valid_points:
            distances = [p.distance for p in scan.valid_points]
            self.ax.hist(distances, bins=50, range=(0, max_range),
                         color='#00d4ff', alpha=0.7, edgecolor='#0a3d62')
            self.ax.set_title(f'Distance (valid: {len(distances)})',
                            color='#aaaaaa', fontsize=10)
        self.draw()
        self.flush_events()


class DistanceTimelineCanvas(FigureCanvas):
    """距离时间线折线图（类似TFmini Plus上位机的TIME LINE CHART）"""

    def __init__(self, parent=None, dpi=100):
        self.fig = Figure(dpi=dpi, facecolor='#1a1a2e')
        self.ax = self.fig.add_subplot(111)
        self.ax.set_facecolor('#16213e')
        super().__init__(self.fig)
        self.setParent(parent)
        self._distances = []
        self._max_points = 200
        self._ylabel = 'Distance(cm)'
        self._auto_range = True
        self._manual_max = 1000.0
        self._setup()

    def _setup(self):
        self.ax.tick_params(colors='#aaaaaa', labelsize=8)
        self.ax.grid(True, color='#333355', alpha=0.5, linewidth=0.5)
        self.ax.set_xlabel('Number of Points(a.u.)', color='#aaaaaa', fontsize=9)
        self.ax.set_ylabel(self._ylabel, color='#aaaaaa', fontsize=9)
        self.ax.set_title('TIME LINE CHART', color='#00d4ff', fontsize=11, fontweight='bold')
        self.fig.subplots_adjust(left=0.1, right=0.95, top=0.92, bottom=0.12)

    def set_ylabel(self, label: str):
        self._ylabel = label
        self.update_chart()

    def set_auto_range(self, auto: bool):
        self._auto_range = auto

    def set_manual_max(self, max_val: float):
        self._manual_max = max_val

    def add_point(self, distance_cm: float):
        self._distances.append(distance_cm)
        if len(self._distances) > self._max_points:
            self._distances = self._distances[-self._max_points:]

    def update_chart(self):
        self.ax.clear()
        self._setup()
        if self._distances:
            x = list(range(len(self._distances)))
            self.ax.plot(x, self._distances, color='#4488ff', linewidth=1.2, alpha=0.9)
            self.ax.fill_between(x, self._distances, alpha=0.08, color='#4488ff')
            if len(self._distances) > 1:
                self.ax.set_xlim(0, max(len(self._distances) - 1, 1))
            if self._auto_range:
                min_d = min(self._distances)
                max_d = max(self._distances)
                margin = max((max_d - min_d) * 0.1, 1.0)
                self.ax.set_ylim(min_d - margin, max_d + margin)
            else:
                self.ax.set_ylim(0, self._manual_max)
        self.draw()
        self.flush_events()

    def clear_data(self):
        self._distances.clear()


class LidarViewerWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("X LiDARViewer")
        self.setMinimumSize(1200, 800)
        self.resize(1400, 900)
        # 设置窗口标题栏图标
        icon_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'icon.ico')
        if os.path.exists(icon_path):
            from PyQt6.QtGui import QIcon
            self.setWindowIcon(QIcon(icon_path))
        self._reader_thread = None
        self._file_thread = None
        self._is_connected = False
        self._frame_count = 0
        self._total_points = 0
        self._scan_buffer = []
        self._scan_frame_count = 0
        self._scan_frames = []        # 当前scan周期内的帧（用于保存）
        self._live_frames = []        # 串口模式下所有接收的帧
        self._auto_save_path = None   # 当前auto-save文件路径
        self._frames_per_scan = 52
        self._current_scan = LidarScan()
        self._last_frame = None
        self._max_range = 1000.0  # 1m = 1000mm
        self._fps_counter = 0
        self._fps_last_time = time.time()
        self._current_fps = 0.0
        self._config_mgr = ConfigManager()
        self._protocol_config = self._config_mgr.active_profile.get('protocol', {})
        self._profile_key = self._config_mgr.active_profile_name
        # TFmini Plus 专用状态
        self._tfmini_data_points = []
        self._tfmini_last_point = None
        self._tfmini_effective_count = 0
        self._tfmini_last_fps_time = time.time()
        self._tfmini_effective_per_sec = 0

        BG = "#0f0f23"; PN = "#1a1a2e"; AC = "#00d4ff"; TX = "#e0e0e0"
        self.setStyleSheet(f"""
            QMainWindow {{ background-color: {BG}; }}
            QGroupBox {{ background-color: {PN}; border: 1px solid #333355;
                border-radius: 6px; margin-top: 12px; padding-top: 16px;
                color: {TX}; font-weight: bold; }}
            QGroupBox::title {{ subcontrol-origin: margin; left: 12px; padding: 0 6px; }}
            QLabel {{ color: {TX}; font-size: 12px; }}
            QPushButton {{ background-color: #16213e; color: {TX};
                border: 1px solid #333355; border-radius: 4px; padding: 6px 14px; font-size: 12px; }}
            QPushButton:hover {{ background-color: #1a3a5c; border-color: {AC}; }}
            QPushButton:disabled {{ background-color: #111122; color: #555555; }}
            QComboBox {{ background-color: #16213e; color: {TX};
                border: 1px solid #333355; border-radius: 4px; padding: 4px 8px; }}
            QTableWidget {{ background-color: #0d1117; color: {TX};
                border: 1px solid #333355; gridline-color: #222244; font-size: 11px; }}
            QHeaderView::section {{ background-color: #16213e; color: {TX};
                border: 1px solid #333355; padding: 4px; font-weight: bold; }}
            QTabWidget::pane {{ border: 1px solid #333355; background-color: {PN}; }}
            QTabBar::tab {{ background-color: #16213e; color: {TX};
                padding: 6px 14px; border: 1px solid #333355; border-bottom: none; margin-right: 2px; }}
            QTabBar::tab:selected {{ background-color: {PN}; border-bottom: 2px solid {AC}; }}
            QTextEdit {{ background-color: #0d1117; color: #00ff88;
                border: 1px solid #333355; font-family: Consolas; font-size: 11px; }}
            QSpinBox {{ background-color: #16213e; color: {TX};
                border: 1px solid #333355; border-radius: 4px; padding: 4px; }}
            QProgressBar {{ border: 1px solid #333355; border-radius: 4px;
                background-color: #0d1117; text-align: center; color: {TX}; }}
            QProgressBar::chunk {{ background-color: {AC}; border-radius: 3px; }}
        """)
        self._init_ui()
        self._init_timer()
        self._refresh_ports()

    def _init_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        ml = QVBoxLayout(central)
        ml.setContentsMargins(6, 6, 6, 4)
        ml.setSpacing(4)

        # 标题栏 (图标 + 标题 + 版本 + 状态)
        ml.addWidget(self._create_title_bar())
        # 工具栏行1: 配置管理
        ml.addWidget(self._create_config_toolbar())
        # 工具栏行2: 串口连接
        ml.addWidget(self._create_serial_toolbar())
        # 工具栏行3: 回放控制
        ml.addWidget(self._create_replay_toolbar())
        # 工具栏行4: 显示与导出
        ml.addWidget(self._create_display_toolbar())

        # 主内容区 - 使用 splitter 实现2:1比例
        sp = QSplitter(Qt.Orientation.Horizontal)
        sp.setHandleWidth(4)
        sp.setStyleSheet("QSplitter::handle{background-color:#333355}")
        tabs = QTabWidget()
        tabs.addTab(self._make_polar_tab(), "Polar View")
        tabs.addTab(self._make_cart_tab(), "Cartesian View")
        tabs.addTab(self._make_hist_tab(), "Distance Distribution")
        tabs.addTab(self._make_timeline_tab(), "Distance Timeline")
        self._tfmini_config_tab_widget = self._make_tfmini_config_tab()
        self._tfmini_config_tab_idx = tabs.addTab(self._tfmini_config_tab_widget, "TFmini Config")
        # 初始状态：根据当前profile决定TFmini Config标签页是否可见
        tabs.setTabVisible(self._tfmini_config_tab_idx, self._profile_key == 'tfmini_plus')
        self._main_tabs = tabs
        sp.addWidget(tabs)
        sp.addWidget(self._create_right_panel())
        sp.setStretchFactor(0, 2)
        sp.setStretchFactor(1, 1)
        ml.addWidget(sp, 1)

    def _make_polar_tab(self):
        w = QWidget(); lay = QVBoxLayout(w); lay.setContentsMargins(4,4,4,4)
        self._polar_canvas = PolarCanvas(dpi=100); lay.addWidget(self._polar_canvas)
        return w

    def _make_cart_tab(self):
        w = QWidget(); lay = QVBoxLayout(w); lay.setContentsMargins(4,4,4,4)
        self._cart_canvas = CartesianCanvas(dpi=100); lay.addWidget(self._cart_canvas)
        return w

    def _make_hist_tab(self):
        w = QWidget(); lay = QVBoxLayout(w); lay.setContentsMargins(4,4,4,4)
        self._hist_canvas = HistogramCanvas(dpi=100); lay.addWidget(self._hist_canvas)
        return w

    def _make_timeline_tab(self):
        """距离时间线标签页 - 通用，适用于所有雷达"""
        w = QWidget()
        lay = QVBoxLayout(w)
        lay.setContentsMargins(4, 4, 4, 4)
        self._timeline_canvas = DistanceTimelineCanvas(dpi=100)
        lay.addWidget(self._timeline_canvas, 1)
        return w
    def _make_tfmini_config_tab(self):
        """TFmini Plus 配置命令标签页"""
        w = QWidget()
        lay = QVBoxLayout(w)
        lay.setContentsMargins(8, 8, 8, 8)
        # 快捷命令区域
        quick_group = QGroupBox("快捷命令")
        quick_lay = QGridLayout(quick_group)
        quick_lay.setSpacing(6)
        row, col = 0, 0
        self._tfmini_cmd_buttons = {}
        for key, cmd_info in TFMINI_PLUS_PRESET_COMMANDS.items():
            btn = QPushButton(cmd_info['name'])
            btn.setToolTip(cmd_info['description'])
            btn.setStyleSheet(
                "QPushButton{background-color:#1a3a5c;color:#e0e0e0;border:1px solid #00d4ff;"
                "border-radius:4px;padding:6px 10px;font-size:11px}"
                "QPushButton:hover{background-color:#00d4ff;color:#0a0a1a}")
            btn.clicked.connect(lambda checked, k=key: self._send_tfmini_preset(k))
            quick_lay.addWidget(btn, row, col)
            self._tfmini_cmd_buttons[key] = btn
            col += 1
            if col >= 4:
                col = 0
                row += 1
        lay.addWidget(quick_group)
        # 帧率设置
        fps_group = QGroupBox("输出帧率设置 (1-1000Hz)")
        fps_lay = QHBoxLayout(fps_group)
        self._tfmini_fps_spin = QSpinBox()
        self._tfmini_fps_spin.setRange(1, 1000)
        self._tfmini_fps_spin.setValue(100)
        self._tfmini_fps_spin.setMinimumWidth(80)
        fps_lay.addWidget(self._tfmini_fps_spin)
        for fps_val in FPS_PRESETS:
            btn = QPushButton(str(fps_val))
            btn.setFixedWidth(50)
            btn.setStyleSheet("QPushButton{background-color:#16213e;color:#00d4ff;border:1px solid #00d4ff;border-radius:3px;font-size:11px}"
                              "QPushButton:hover{background-color:#00d4ff;color:#0a0a1a}")
            btn.clicked.connect(lambda checked, v=fps_val: self._send_tfmini_fps(v))
            fps_lay.addWidget(btn)
        btn = QPushButton("设置帧率")
        btn.setStyleSheet("QPushButton{background-color:#1a5c2e;color:#e0e0e0;border:1px solid #2ecc71;border-radius:4px;padding:6px 10px}"
                          "QPushButton:hover{background-color:#2ecc71;color:#0a0a1a}")
        btn.clicked.connect(self._send_tfmini_custom_fps)
        fps_lay.addWidget(btn)
        fps_lay.addStretch()
        lay.addWidget(fps_group)
        # 波特率设置
        baud_group = QGroupBox("波特率设置")
        baud_lay = QHBoxLayout(baud_group)
        self._tfmini_baud_combo = QComboBox()
        for b in BAUDRATE_PRESETS:
            self._tfmini_baud_combo.addItem(str(b), b)
        self._tfmini_baud_combo.setCurrentText("115200")
        self._tfmini_baud_combo.setMinimumWidth(100)
        baud_lay.addWidget(self._tfmini_baud_combo)
        btn = QPushButton("设置波特率")
        btn.setStyleSheet("QPushButton{background-color:#1a3a5c;color:#e0e0e0;border:1px solid #00d4ff;border-radius:4px;padding:6px 10px}"
                          "QPushButton:hover{background-color:#00d4ff;color:#0a0a1a}")
        btn.clicked.connect(self._send_tfmini_baudrate)
        baud_lay.addWidget(btn)
        baud_lay.addStretch()
        lay.addWidget(baud_group)
        # 自定义HEX命令
        custom_group = QGroupBox("自定义HEX命令")
        custom_lay = QHBoxLayout(custom_group)
        custom_lay.addWidget(QLabel("HEX:"))
        self._tfmini_custom_hex = QLineEdit()
        self._tfmini_custom_hex.setPlaceholderText("例: 5A 04 01 5F")
        self._tfmini_custom_hex.setStyleSheet("background-color:#0d1117;color:#00ff88;border:1px solid #333355;border-radius:4px;padding:4px 8px;font-family:Consolas;font-size:12px")
        custom_lay.addWidget(self._tfmini_custom_hex)
        btn = QPushButton("发送")
        btn.setStyleSheet("QPushButton{background-color:#5c1a1a;color:#e0e0e0;border:1px solid #e74c3c;border-radius:4px;padding:6px 14px}"
                          "QPushButton:hover{background-color:#e74c3c;color:#0a0a1a}")
        btn.clicked.connect(self._send_tfmini_custom_hex)
        custom_lay.addWidget(btn)
        lay.addWidget(custom_group)
        # 响应日志
        log_group = QGroupBox("命令响应日志")
        log_lay = QVBoxLayout(log_group)
        self._tfmini_log = QTextEdit()
        self._tfmini_log.setReadOnly(True)
        self._tfmini_log.setFont(QFont("Consolas", 10))
        self._tfmini_log.setMaximumHeight(200)
        log_lay.addWidget(self._tfmini_log)
        lay.addWidget(log_group)
        lay.addStretch()
        return w


    def _create_title_bar(self):
        bar = QFrame()
        bar.setStyleSheet("QFrame{background-color:#0a0a1a;border:1px solid #333355;border-radius:6px;padding:2px 8px}")
        lay = QHBoxLayout(bar)
        lay.setContentsMargins(12, 6, 12, 6)
        icon_label = QLabel()
        icon_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'icon.ico')
        if os.path.exists(icon_path):
            from PyQt6.QtGui import QPixmap
            pm = QPixmap(icon_path).scaled(60, 36, Qt.AspectRatioMode.KeepAspectRatio,
                                            Qt.TransformationMode.SmoothTransformation)
            icon_label.setPixmap(pm)
        icon_label.setFixedSize(64, 40)
        icon_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        lay.addWidget(icon_label)
        title = QLabel("X LiDARViewer")
        title.setStyleSheet("color:#00d4ff;font-size:16px;font-weight:bold")
        lay.addWidget(title)
        ver = QLabel("V1.2.0")
        ver.setStyleSheet("color:#555555;font-size:11px;border:1px solid #333355;border-radius:3px;padding:1px 6px")
        lay.addWidget(ver)
        lay.addStretch()
        self._status_indicator = QLabel("● Disconnected")
        self._status_indicator.setStyleSheet("color:#ff4444;font-weight:bold;font-size:13px")
        lay.addWidget(self._status_indicator)
        return bar

    def _make_toolbar(self, icon, color):
        bar = QFrame()
        bar.setStyleSheet("QFrame{background-color:#1a1a2e;border:1px solid #2a2a44;border-radius:5px;padding:2px}")
        lay = QHBoxLayout(bar)
        lay.setContentsMargins(8, 3, 8, 3)
        lay.setSpacing(8)
        lbl = QLabel(icon)
        lbl.setStyleSheet(f"color:{color};font-size:16px;font-weight:bold;padding:0px 2px")
        lbl.setFixedWidth(30)
        lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        lay.addWidget(lbl)
        return bar, lay

    def _create_config_toolbar(self):
        bar, lay = self._make_toolbar("⚙", "#f39c12")
        lay.addWidget(QLabel("LiDAR:"))
        self._profile_combo = QComboBox()
        self._profile_combo.setMinimumWidth(150)
        for name in self._config_mgr.profile_names:
            prof = self._config_mgr.get_profile(name)
            self._profile_combo.addItem(prof.get('name', name), name)
        idx = self._config_mgr.profile_names.index(self._config_mgr.active_profile_name) \
            if self._config_mgr.active_profile_name in self._config_mgr.profile_names else 0
        self._profile_combo.setCurrentIndex(idx)
        self._profile_combo.currentIndexChanged.connect(self._on_profile_changed)
        lay.addWidget(self._profile_combo)
        self._import_cfg_btn = QPushButton("Import Config")
        self._import_cfg_btn.setStyleSheet("QPushButton{background-color:#4a3a1a;border-color:#f39c12}QPushButton:hover{background-color:#f39c12}")
        self._import_cfg_btn.clicked.connect(self._import_config)
        lay.addWidget(self._import_cfg_btn)
        self._reload_cfg_btn = QPushButton("Reload")
        self._reload_cfg_btn.clicked.connect(self._reload_config)
        lay.addWidget(self._reload_cfg_btn)
        lay.addStretch()
        return bar

    def _create_serial_toolbar(self):
        bar, lay = self._make_toolbar("⚡", "#2ecc71")
        lay.addWidget(QLabel("Port:"))
        self._port_combo = QComboBox()
        self._port_combo.setMinimumWidth(130)
        lay.addWidget(self._port_combo)
        self._refresh_btn = QPushButton("Scan")
        self._refresh_btn.clicked.connect(self._refresh_ports)
        lay.addWidget(self._refresh_btn)
        lay.addWidget(QLabel("Baud:"))
        self._baud_combo = QComboBox()
        self._baud_combo.addItems(["9600","57600","115200","230400","460800","921600"])
        self._baud_combo.setCurrentText("115200")
        lay.addWidget(self._baud_combo)
        self._connect_btn = QPushButton("Connect")
        self._connect_btn.setStyleSheet("QPushButton{background-color:#1a5c2e;border-color:#2ecc71}QPushButton:hover{background-color:#2ecc71}")
        self._connect_btn.clicked.connect(self._toggle_connection)
        lay.addWidget(self._connect_btn)
        lay.addStretch()
        return bar

    def _create_replay_toolbar(self):
        bar, lay = self._make_toolbar("▶", "#00d4ff")
        self._file_btn = QPushButton("Open DAT")
        self._file_btn.clicked.connect(self._open_dat_file)
        lay.addWidget(self._file_btn)
        self._file_stop_btn = QPushButton("Stop")
        self._file_stop_btn.setEnabled(False)
        self._file_stop_btn.clicked.connect(self._stop_file)
        lay.addWidget(self._file_stop_btn)
        self._file_clear_btn = QPushButton("Clear DAT")
        self._file_clear_btn.setEnabled(False)
        self._file_clear_btn.clicked.connect(self._clear_dat)
        lay.addWidget(self._file_clear_btn)
        self._file_pause_btn = QPushButton("Play")
        self._file_pause_btn.setEnabled(False)
        self._file_pause_btn.clicked.connect(self._toggle_pause)
        self._file_pause_btn.setStyleSheet("QPushButton{background-color:#1a3a5c;border-color:#00d4ff}QPushButton:hover{background-color:#00d4ff}")
        lay.addWidget(self._file_pause_btn)
        _step_style = ("QPushButton{background-color:#1a3a5c;color:#00d4ff;border:1px solid #00d4ff;"
                       "border-radius:4px;font-weight:bold;min-width:60px;}"
                       "QPushButton:hover{background-color:#00d4ff;color:#0a0a1a;}"
                       "QPushButton:disabled{color:#555;border-color:#333355;}")
        self._file_step_back_btn = QPushButton("|◀ Step")
        self._file_step_back_btn.setEnabled(False)
        self._file_step_back_btn.clicked.connect(self._step_back_frame)
        self._file_step_back_btn.setStyleSheet(_step_style)
        lay.addWidget(self._file_step_back_btn)
        self._file_step_btn = QPushButton("Step ▶|")
        self._file_step_btn.setEnabled(False)
        self._file_step_btn.clicked.connect(self._step_frame)
        self._file_step_btn.setStyleSheet(_step_style)
        lay.addWidget(self._file_step_btn)
        lay.addWidget(QLabel("Speed:"))
        self._speed_combo = QComboBox()
        self._speed_combo.addItems(["0.25x","0.5x","1x","2x","5x","10x","Max"])
        self._speed_combo.setCurrentText("1x")
        self._speed_combo.currentTextChanged.connect(self._on_speed_changed)
        self._speed_combo.setMinimumWidth(70)
        lay.addWidget(self._speed_combo)
        self._file_progress = QProgressBar()
        self._file_progress.setMaximumWidth(180)
        self._file_progress.setFormat("Loading...")
        self._file_progress.setVisible(False)
        lay.addWidget(self._file_progress)
        lay.addStretch()
        return bar

    def _create_display_toolbar(self):
        bar, lay = self._make_toolbar("📊", "#9b59b6")
        self._auto_range_cb = QCheckBox("Auto Range")
        self._auto_range_cb.setChecked(True)
        self._auto_range_cb.setStyleSheet("color:#e0e0e0;font-size:12px")
        self._auto_range_cb.toggled.connect(self._on_auto_range_toggled)
        lay.addWidget(self._auto_range_cb)
        lay.addWidget(QLabel("Range:"))
        self._range_spin = QSpinBox()
        self._range_spin.setRange(1, 100)
        self._range_spin.setValue(1)
        self._range_spin.setMinimumWidth(80)
        self._range_spin.valueChanged.connect(self._on_range_value_changed)
        # 隐藏 SpinBox 自带的上下按钮
        self._range_spin.setStyleSheet(self._range_spin.styleSheet() + """
            QSpinBox::up-button, QSpinBox::down-button {
                width: 0; height: 0; border: none; padding: 0; margin: 0;
            }
        """)
        # 默认 Auto Range 时禁用 Range 控件
        self._range_spin.setEnabled(False)
        # 自定义三角形按钮（QPainter绘制，与UI风格统一）
        spin_container = QWidget()
        spin_lay = QVBoxLayout(spin_container)
        spin_lay.setContentsMargins(0, 0, 0, 0)
        spin_lay.setSpacing(0)
        self._range_up_btn = TriangleButton("up")
        self._range_up_btn.clicked.connect(lambda: self._range_spin.setValue(self._range_spin.value() + 1))
        self._range_down_btn = TriangleButton("down")
        self._range_down_btn.clicked.connect(lambda: self._range_spin.setValue(self._range_spin.value() - 1))
        spin_lay.addWidget(self._range_up_btn)
        spin_lay.addWidget(self._range_down_btn)
        self._range_spin_container = spin_container
        self._range_up_btn.setEnabled(False)
        self._range_down_btn.setEnabled(False)
        lay.addWidget(self._range_spin)
        lay.addWidget(spin_container)
        self._unit_combo = QComboBox()
        self._unit_combo.addItems(["mm", "cm", "m"])
        self._unit_combo.setCurrentText("m")
        self._unit_combo.setMinimumWidth(50)
        self._unit_combo.currentTextChanged.connect(self._on_unit_changed)
        self._unit_combo.setEnabled(False)
        lay.addWidget(self._unit_combo)
        self._clear_btn = QPushButton("Clean")
        self._clear_btn.clicked.connect(self._clear_data)
        lay.addWidget(self._clear_btn)
        sep = QFrame(); sep.setFrameShape(QFrame.Shape.VLine); sep.setStyleSheet("color:#333355")
        lay.addWidget(sep)
        self._save_btn = QPushButton("Save DAT")
        self._save_btn.setStyleSheet("QPushButton{background-color:#1a3a5c;border-color:#00d4ff}QPushButton:hover{background-color:#00d4ff}")
        self._save_btn.clicked.connect(self._save_point_cloud)
        lay.addWidget(self._save_btn)
        self._save_all_btn = QPushButton("Save All")
        self._save_all_btn.setStyleSheet("QPushButton{background-color:#1a3a5c;border-color:#00d4ff}QPushButton:hover{background-color:#00d4ff}")
        self._save_all_btn.clicked.connect(self._save_all_data)
        lay.addWidget(self._save_all_btn)
        self._auto_save_cb = QCheckBox("Auto Save")
        self._auto_save_cb.setStyleSheet("color:#e0e0e0")
        self._auto_save_cb.setToolTip("Auto save each scan to single .dat file in DATA folder")
        lay.addWidget(self._auto_save_cb)
        self._save_dir = ""
        self._save_index = 0
        self._last_dat_path = ""
        self._last_unit = "m"
        lay.addStretch()
        return bar

    def _create_right_panel(self):
        panel = QWidget()
        lay = QVBoxLayout(panel)
        lay.setContentsMargins(4, 4, 4, 4)
        lay.setSpacing(6)

        # Stats
        sg = QGroupBox("Statistics")
        gl = QGridLayout(sg); gl.setSpacing(4)
        self._lbl_fps = QLabel("FPS: 0.0")
        self._lbl_fps.setStyleSheet("color:#00ff88;font-size:16px;font-weight:bold")
        gl.addWidget(self._lbl_fps, 0, 0)
        self._lbl_frames = QLabel("Frames: 0")
        gl.addWidget(self._lbl_frames, 0, 1)
        self._lbl_points = QLabel("Points: 0")
        gl.addWidget(self._lbl_points, 0, 2)
        self._lbl_speed = QLabel("Speed: -- RPM")
        self._lbl_speed.setStyleSheet("color:#00d4ff;font-size:14px")
        gl.addWidget(self._lbl_speed, 1, 0, 1, 2)
        self._lbl_freq = QLabel("Freq: -- Hz")
        gl.addWidget(self._lbl_freq, 1, 2)
        self._lbl_range_info = QLabel("Range: min -- / avg -- / max -- mm")
        gl.addWidget(self._lbl_range_info, 2, 0, 1, 3)
        self._lbl_valid = QLabel("Valid: 0 / 0")
        gl.addWidget(self._lbl_valid, 3, 0, 1, 3)
        lay.addWidget(sg)

        # Frame detail table
        fg = QGroupBox("Frame Details")
        fl = QVBoxLayout(fg)
        self._frame_table = QTableWidget(0, 4)
        self._frame_table.setHorizontalHeaderLabels(["#","Angle","Dist(mm)","Quality"])
        self._frame_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self._frame_table.verticalHeader().setVisible(False)
        self._frame_table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        self._frame_table.setMaximumHeight(280)
        fl.addWidget(self._frame_table)
        lay.addWidget(fg)

        # Raw data
        rg = QGroupBox("Raw Data")
        rl = QVBoxLayout(rg)
        self._raw_text = QTextEdit()
        self._raw_text.setReadOnly(True)
        self._raw_text.setMaximumHeight(140)
        self._raw_text.setFont(QFont("Consolas", 10))
        rl.addWidget(self._raw_text)
        lay.addWidget(rg)
        lay.addStretch()
        return panel

    def _init_timer(self):
        self._display_timer = QTimer()
        self._display_timer.timeout.connect(self._update_display)
        self._display_timer.start(100)

    def _refresh_ports(self):
        self._port_combo.clear()
        for p in serial.tools.list_ports.comports():
            self._port_combo.addItem(f"{p.device} - {p.description}", p.device)
        if not self._port_combo.count():
            self._port_combo.addItem("(No ports)", "")

    def _on_profile_changed(self, index):
        profile_key = self._profile_combo.currentData()
        if not profile_key:
            return
        self._config_mgr.active_profile_name = profile_key
        self._profile_key = profile_key
        self._protocol_config = self._config_mgr.active_profile.get('protocol', {})
        scan_cfg = self._config_mgr.active_profile.get('scan', {})
        self._frames_per_scan = scan_cfg.get('frames_per_scan', 52)
        prof = self._config_mgr.active_profile
        name = prof.get('name', profile_key)
        desc = prof.get('description', '')
        self._raw_text.append(f"[Config] Switched to: {name} - {desc}")
        hdr = self._protocol_config.get('header', [])
        self._raw_text.append(
            f"  Header: {' '.join(f'{b:02X}' for b in hdr)} | "
            f"Frame: {self._protocol_config.get('frame_size', '?')}B | "
            f"Points/frame: {self._protocol_config.get('points_per_frame', '?')}")
        # 控制 TFmini Config 标签页的显示/隐藏
        is_tfmini = (profile_key == 'tfmini_plus')
        self._main_tabs.setTabVisible(self._tfmini_config_tab_idx, is_tfmini)
        # 切换时间线Y轴单位
        if is_tfmini:
            self._timeline_canvas.set_ylabel('Distance(cm)')
            self._frame_table.setHorizontalHeaderLabels(["#", "Dist(cm)", "Dist(mm)", "Strength"])
        else:
            self._timeline_canvas.set_ylabel('Distance(mm)')
            self._frame_table.setHorizontalHeaderLabels(["#", "Angle", "Dist(mm)", "Quality"])

    def _import_config(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "Import LiDAR Config", "",
            "JSON (*.json);;All (*)")
        if not path:
            return
        try:
            import shutil
            dest = self._config_mgr.config_path
            shutil.copy2(path, dest)
            self._config_mgr.load()
            self._refresh_profile_combo()
            self._raw_text.append(f"[Config] Imported: {path}")
        except Exception as e:
            self._raw_text.append(f"[Error] Import failed: {e}")

    def _reload_config(self):
        try:
            self._config_mgr.load()
            self._refresh_profile_combo()
            self._raw_text.append("[Config] Configuration reloaded")
        except Exception as e:
            self._raw_text.append(f"[Error] Reload failed: {e}")

    def _refresh_profile_combo(self):
        self._profile_combo.blockSignals(True)
        self._profile_combo.clear()
        for name in self._config_mgr.profile_names:
            prof = self._config_mgr.get_profile(name)
            self._profile_combo.addItem(prof.get('name', name), name)
        idx = 0
        if self._config_mgr.active_profile_name in self._config_mgr.profile_names:
            idx = self._config_mgr.profile_names.index(self._config_mgr.active_profile_name)
        self._profile_combo.setCurrentIndex(idx)
        self._profile_combo.blockSignals(False)
        self._on_profile_changed(idx)

    def _toggle_connection(self):
        if self._is_connected:
            self._disconnect()
        else:
            self._connect_serial()

    def _set_replay_buttons_enabled(self, enabled):
        """统一控制回放按钮的启用/禁用状态"""
        self._file_btn.setEnabled(enabled)
        self._file_stop_btn.setEnabled(enabled)
        self._file_clear_btn.setEnabled(enabled)
        self._file_pause_btn.setEnabled(enabled)
        self._file_step_btn.setEnabled(enabled)
        self._file_step_back_btn.setEnabled(enabled)

    def _connect_serial(self):
        port = self._port_combo.currentData()
        if not port:
            self._raw_text.append("[Error] Select port")
            return
        # 连接串口时自动清除已加载的DAT文件
        if self._file_thread:
            self._clear_dat()
        baud = int(self._baud_combo.currentText())
        self._reader_thread = SerialReaderThread(port, baud, self._protocol_config, self._profile_key)
        self._reader_thread.frame_received.connect(self._on_frame_received)
        self._reader_thread.error_occurred.connect(self._on_error)
        self._reader_thread.connection_lost.connect(self._on_conn_lost)
        self._reader_thread.tfmini_data_received.connect(self._on_tfmini_data)
        self._reader_thread.tfmini_config_response.connect(self._on_tfmini_config_response)
        self._reader_thread.start()
        self._is_connected = True
        self._connect_btn.setText("Disconnect")
        self._connect_btn.setStyleSheet("QPushButton{background-color:#8b1a1a;border-color:#e74c3c}")
        self._status_indicator.setText(f"Connected: {port}")
        self._status_indicator.setStyleSheet("color:#00ff88;font-weight:bold;font-size:13px")
        self._raw_text.append(f"[Info] Connected {port} @ {baud}")
        # 实时连接时禁用回放按钮
        self._set_replay_buttons_enabled(False)

    def _disconnect(self):
        if self._reader_thread:
            self._reader_thread.stop()
            self._reader_thread.wait(2000)
            self._reader_thread = None
        self._is_connected = False
        self._connect_btn.setText("Connect")
        self._connect_btn.setStyleSheet("QPushButton{background-color:#1a5c2e;border-color:#2ecc71}")
        self._status_indicator.setText("Disconnected")
        self._status_indicator.setStyleSheet("color:#ff4444;font-weight:bold;font-size:13px")
        # 断开后恢复回放按钮（Open DAT 可用，其余取决于是否有文件）
        self._file_btn.setEnabled(True)
        has_file = self._file_thread is not None
        self._file_stop_btn.setEnabled(has_file)
        self._file_clear_btn.setEnabled(has_file)
        self._file_pause_btn.setEnabled(has_file)
        self._file_step_btn.setEnabled(has_file)
        self._file_step_back_btn.setEnabled(has_file)

    def _on_error(self, msg):
        self._raw_text.append(f"[Error] {msg}")
        self._disconnect()

    def _on_conn_lost(self):
        if self._is_connected:
            self._on_error("Connection lost")

    def _open_dat_file(self):
        path, _ = QFileDialog.getOpenFileName(self, "Open DAT", "", "DAT (*.dat);;All (*)")
        if not path: return
        self._last_dat_path = path
        self._start_replay(path)

    def _start_replay(self, path):
        if self._is_connected: self._disconnect()
        speed_map = {"0.25x": 0.25, "0.5x": 0.5, "1x": 1.0,
                     "2x": 2.0, "5x": 5.0, "10x": 10.0, "Max": 100.0}
        speed = speed_map.get(self._speed_combo.currentText(), 1.0)
        self._file_thread = FileReaderThread(path, speed, self._protocol_config, self._profile_key)
        self._file_thread.frame_received.connect(self._on_frame_received)
        self._file_thread.tfmini_data_received.connect(self._on_tfmini_data)
        self._file_thread.format_mismatch.connect(self._on_format_mismatch)
        self._file_thread.progress.connect(self._on_load_progress)
        self._file_thread.frame_position.connect(self._on_frame_position)
        self._file_thread.finished_signal.connect(self._on_file_done)
        self._file_thread.start()
        self._file_btn.setEnabled(False)
        self._file_stop_btn.setEnabled(True)
        self._file_clear_btn.setEnabled(True)
        self._file_pause_btn.setEnabled(True)
        self._file_pause_btn.setText("Play")
        self._file_step_btn.setEnabled(True)
        self._file_step_back_btn.setEnabled(True)
        self._file_progress.setVisible(True)
        self._file_progress.setRange(0, 0)  # 不确定模式
        self._file_progress.setFormat("Loading...")
        self._status_indicator.setText(f"● Loading: {os.path.basename(path)}")
        self._status_indicator.setStyleSheet("color:#f39c12;font-weight:bold;font-size:13px")

    def _stop_file(self):
        """重置播放位置到开头，保留已加载文件"""
        if self._file_thread:
            self._file_thread.pause()
            self._file_thread.reset_position()
        self._scan_buffer.clear()
        self._scan_frame_count = 0
        self._frame_count = 0
        self._total_points = 0
        self._file_pause_btn.setText("Play")
        self._file_pause_btn.setEnabled(True)
        self._file_step_btn.setEnabled(True)
        self._file_step_back_btn.setEnabled(True)
        # 重建帧0的显示
        self._rebuild_scan_from_loaded()
        self._status_indicator.setText("● Replay: Paused")
        self._status_indicator.setStyleSheet("color:#f39c12;font-weight:bold;font-size:13px")

    def _clear_dat(self):
        """清除已加载的文件，完全重置"""
        if self._file_thread:
            self._file_thread.stop()
            self._file_thread.wait(2000)
            self._file_thread = None
        self._clear_data()
        self._file_btn.setEnabled(True)
        self._file_stop_btn.setEnabled(False)
        self._file_clear_btn.setEnabled(False)
        self._file_pause_btn.setEnabled(False)
        self._file_pause_btn.setText("Play")
        self._file_step_btn.setEnabled(False)
        self._file_step_back_btn.setEnabled(False)
        self._file_progress.setVisible(False)
        self._status_indicator.setText("● No Data")
        self._status_indicator.setStyleSheet("color:#888;font-weight:bold;font-size:13px")

    def _toggle_pause(self):
        if not self._file_thread:
            return
        btn = self._file_pause_btn
        # 回放完成状态 → 重新加载
        if btn.text() == "Replay":
            if self._last_dat_path:
                self._start_replay(self._last_dat_path)
            return
        # Play状态 → 开始/恢复播放
        if btn.text() == "Play":
            # 清空 scan_buffer，避免步进残留数据污染连续播放
            self._scan_buffer.clear()
            self._scan_frame_count = 0
            self._file_thread.resume()
            btn.setText("Pause")
            self._status_indicator.setText("● Replay: Playing")
            self._status_indicator.setStyleSheet("color:#2ecc71;font-weight:bold;font-size:13px")
        # Pause状态 → 暂停播放
        else:
            self._file_thread.pause()
            btn.setText("Play")
            self._status_indicator.setText("● Replay: Paused")
            self._status_indicator.setStyleSheet("color:#f39c12;font-weight:bold;font-size:13px")

    def _step_frame(self):
        """逐帧前进"""
        if not self._file_thread:
            return
        # 如果正在播放，先暂停
        if self._file_pause_btn.text() == "Pause":
            self._file_thread.pause()
            self._file_pause_btn.setText("Play")
        if self._file_thread.step_one():
            # 用 frames[0:idx] 全部有效点重建 scan
            self._rebuild_scan_from_loaded()
            # 检查是否已到末尾
            if self._file_thread.is_finished:
                self._on_file_done()
        else:
            # 无法前进（未加载完或已到末尾）
            if self._file_thread.loaded:
                self._on_file_done()

    def _step_back_frame(self):
        """逐帧后退"""
        if not self._file_thread:
            return
        # 如果正在播放，先暂停
        if self._file_pause_btn.text() == "Pause":
            self._file_thread.pause()
            self._file_pause_btn.setText("Play")
        if self._file_thread.step_back():
            # 用 frames[0:idx] 全部有效点重建 scan
            self._rebuild_scan_from_loaded()

    def _rebuild_scan_from_loaded(self):
        """从已加载帧列表重建当前 scan 并刷新画布"""
        ft = self._file_thread
        idx = ft._frame_idx
        is_tfmini = (ft.detected_format == 'tfmini_plus')

        if idx == 0:
            self._current_scan = LidarScan()
            self._polar_canvas.update_data(self._current_scan, self._max_range)
            self._cart_canvas.update_data(self._current_scan, self._max_range)
            self._hist_canvas.update_data(self._current_scan, self._max_range)
            self._timeline_canvas.clear_data()
            self._timeline_canvas.update_chart()
            if is_tfmini:
                self._tfmini_last_point = None
                self._tfmini_data_points.clear()
            return

        if is_tfmini:
            # TFmini Plus: 重建时间线和统计数据
            self._timeline_canvas.clear_data()
            self._tfmini_data_points.clear()
            for i in range(idx):
                dp = ft.frames[i]
                self._tfmini_data_points.append(dp)
                self._timeline_canvas.add_point(dp.distance_cm)
            self._timeline_canvas.update_chart()
            self._tfmini_last_point = ft.frames[idx - 1]
            # 空scan用于清除polar/cart/hist
            self._current_scan = LidarScan()
            self._polar_canvas.update_data(self._current_scan, self._max_range)
            self._cart_canvas.update_data(self._current_scan, self._max_range)
            self._hist_canvas.update_data(self._current_scan, self._max_range)
            return

        # 通用雷达: 原有逻辑
        scan_cfg = self._config_mgr.active_profile.get('scan', {})
        fps = scan_cfg.get('frames_per_scan', self._frames_per_scan)
        start = max(0, idx - fps)
        all_points = []
        for i in range(start, idx):
            all_points.extend(ft.frames[i].valid_points)

        if all_points:
            last_frame = ft.frames[idx - 1]
            self._current_scan = LidarScan(
                points=all_points,
                frame_count=idx - start,
                avg_speed_rpm=last_frame.speed_rpm)
            self._last_frame = last_frame
        else:
            self._current_scan = LidarScan()
        auto = self._auto_range_cb.isChecked()
        if auto and self._current_scan.valid_points:
            max_r = max(p.distance for p in self._current_scan.valid_points) * 1.2
            max_r = max(max_r, 100)
        else:
            max_r = self._max_range
        self._polar_canvas.update_data(self._current_scan, max_r)
        self._cart_canvas.update_data(self._current_scan, max_r)
        self._hist_canvas.update_data(self._current_scan, max_r)
        # 重建时间线
        self._timeline_canvas.clear_data()
        for i in range(idx):
            for p in ft.frames[i].valid_points:
                if p.distance > 0:
                    self._timeline_canvas.add_point(p.distance)
        self._timeline_canvas.update_chart()
        if self._last_frame:
            self._update_frame_table(self._last_frame)
            self._update_raw(self._last_frame)

    def _on_format_mismatch(self, msg):
        """DAT文件格式与当前配置不匹配"""
        self._raw_text.append(f"[Error] {msg}")
        self._status_indicator.setText("● Format Mismatch!")
        self._status_indicator.setStyleSheet("color:#ff4444;font-weight:bold;font-size:13px")
        # 重置回放状态
        self._file_btn.setEnabled(True)
        self._file_stop_btn.setEnabled(False)
        self._file_clear_btn.setEnabled(False)
        self._file_pause_btn.setEnabled(False)
        self._file_step_btn.setEnabled(False)
        self._file_step_back_btn.setEnabled(False)
        self._file_progress.setVisible(False)
        if self._file_thread:
            self._file_thread.stop()
            self._file_thread.wait(2000)
            self._file_thread = None

    def _on_load_progress(self, value):
        """加载阶段：显示不确定进度条"""
        if self._file_progress.maximum() != 0:
            self._file_progress.setRange(0, 0)
            self._file_progress.setFormat("Loading...")

    def _on_frame_position(self, current, total):
        """帧位置更新：进度条切换为帧数显示"""
        if total > 0:
            pct = int(current / total * 100)
        else:
            pct = 0
        self._file_progress.setRange(0, max(total, 1))
        self._file_progress.setFormat(f"Frame {current}/{total}  ({pct}%)")
        self._file_progress.setValue(current)

    def _on_speed_changed(self, text):
        speed_map = {
            "0.25x": 0.25, "0.5x": 0.5, "1x": 1.0,
            "2x": 2.0, "5x": 5.0, "10x": 10.0, "Max": 100.0
        }
        speed = speed_map.get(text, 1.0)
        if self._file_thread:
            self._file_thread.speed_mult = speed

    def _on_auto_range_toggled(self, checked):
        """Auto Range 复选框切换"""
        self._range_spin.setEnabled(not checked)
        self._range_up_btn.setEnabled(not checked)
        self._range_down_btn.setEnabled(not checked)
        self._unit_combo.setEnabled(not checked)
        if not checked:
            self._on_range_value_changed(self._range_spin.value())
        else:
            self._refresh_canvases()

    def _on_range_value_changed(self, value):
        unit = self._unit_combo.currentText()
        multiplier = {"mm": 1, "cm": 10, "m": 1000}.get(unit, 1)
        self._max_range = float(value * multiplier)
        self._refresh_canvases()

    def _on_unit_changed(self, unit):
        old_unit = getattr(self, '_last_unit', 'mm')
        old_mult = {"mm": 1, "cm": 10, "m": 1000}.get(old_unit, 1)
        new_mult = {"mm": 1, "cm": 10, "m": 1000}.get(unit, 1)
        old_val = self._range_spin.value()
        new_val = max(1, int(old_val * old_mult / new_mult))
        ranges = {"mm": (1, 100000), "cm": (1, 10000), "m": (1, 100)}
        lo, hi = ranges.get(unit, (1, 100000))
        self._range_spin.blockSignals(True)
        self._range_spin.setRange(lo, hi)
        self._range_spin.setValue(new_val)
        self._range_spin.blockSignals(False)
        self._max_range = float(new_val * new_mult)
        self._last_unit = unit
        self._refresh_canvases()

    def _refresh_canvases(self):
        """用当前数据和新的max_range刷新所有画布"""
        auto = self._auto_range_cb.isChecked()
        if self._current_scan and self._current_scan.valid_points:
            if auto:
                max_r = max(p.distance for p in self._current_scan.valid_points) * 1.2
                max_r = max(max_r, 100)
            else:
                max_r = self._max_range
            self._polar_canvas.update_data(self._current_scan, max_r)
            self._cart_canvas.update_data(self._current_scan, max_r)
            self._hist_canvas.update_data(self._current_scan, max_r)
        # Timeline: 数据单位取决于profile, 需要将mm的max_range转换
        self._timeline_canvas.set_auto_range(auto)
        if not auto:
            # _max_range 是mm, timeline数据是cm(tfmini)或mm(其他)
            if self._profile_key == 'tfmini_plus':
                self._timeline_canvas.set_manual_max(self._max_range / 10.0)
            else:
                self._timeline_canvas.set_manual_max(self._max_range)
        self._timeline_canvas.update_chart()

    def _on_file_done(self):
        self._file_btn.setEnabled(True)
        self._file_stop_btn.setEnabled(False)
        self._file_clear_btn.setEnabled(True)  # 文件仍在，可清除
        self._file_pause_btn.setEnabled(True)
        self._file_pause_btn.setText("Replay")
        self._file_step_btn.setEnabled(False)
        self._file_step_back_btn.setEnabled(False)
        self._file_progress.setVisible(False)
        self._status_indicator.setText("● Replay: Done!")
        self._status_indicator.setStyleSheet("color:#2ecc71;font-weight:bold;font-size:13px")

    def _clear_data(self):
        self._frame_count = 0
        self._total_points = 0
        self._scan_buffer.clear()
        self._scan_frame_count = 0
        self._scan_frames.clear()
        self._live_frames.clear()
        self._auto_save_path = None
        self._current_scan = LidarScan()
        self._last_frame = None
        self._frame_table.setRowCount(0)
        self._raw_text.clear()
        # 清除 Statistics 标签
        self._lbl_fps.setText("FPS: 0.0")
        self._lbl_frames.setText("Frames: 0")
        self._lbl_points.setText("Points: 0")
        self._lbl_speed.setText("Speed: -- RPM")
        self._lbl_freq.setText("Freq: -- Hz")
        self._lbl_range_info.setText("Range: min -- / avg -- / max -- mm")
        self._lbl_valid.setText("Valid: 0 / 0")
        empty = LidarScan()
        self._polar_canvas.update_data(empty, self._max_range)
        self._cart_canvas.update_data(empty, self._max_range)
        self._hist_canvas.update_data(empty, self._max_range)
        # 清除 TFmini Plus 状态
        self._tfmini_data_points.clear()
        self._tfmini_last_point = None
        self._tfmini_effective_count = 0
        self._tfmini_effective_per_sec = 0
        self._timeline_canvas.clear_data()

    def _on_frame_received(self, frame):
        self._last_frame = frame
        self._frame_count += 1
        self._total_points += len(frame.points)
        self._fps_counter += 1
        self._scan_buffer.extend(frame.valid_points)
        self._scan_frames.append(frame)
        # 通用：将每个有效点的距离添加到时间线（忽略角度）
        for p in frame.valid_points:
            if p.distance > 0:
                self._timeline_canvas.add_point(p.distance)
        # 串口模式下累积所有帧（用于 Save All）
        if self._reader_thread and not self._file_thread:
            self._live_frames.append(frame)
        self._scan_frame_count += 1
        scan_cfg = self._config_mgr.active_profile.get('scan', {})
        frames_per_scan = scan_cfg.get('frames_per_scan', self._frames_per_scan)
        if self._scan_frame_count >= frames_per_scan:
            self._current_scan = LidarScan(
                points=list(self._scan_buffer),
                frame_count=self._scan_frame_count,
                avg_speed_rpm=frame.speed_rpm)
            self._scan_buffer.clear()
            self._scan_frame_count = 0
            # 自动保存：将本scan的帧追加到.dat文件
            self._auto_save_frames(self._scan_frames)
            self._scan_frames = []

    def _update_display(self):
        now = time.time()
        elapsed = now - self._fps_last_time
        if elapsed >= 1.0:
            self._current_fps = self._fps_counter / elapsed
            self._fps_counter = 0
            self._fps_last_time = now
        self._lbl_fps.setText(f"FPS: {self._current_fps:.1f}")
        self._lbl_frames.setText(f"Frames: {self._frame_count}")
        self._lbl_points.setText(f"Points: {self._total_points}")
        if self._last_frame:
            f = self._last_frame
            self._lbl_speed.setText(f"Speed: {f.speed_rpm:.1f} RPM")
            self._lbl_freq.setText(f"Freq: {f.speed_hz:.2f} Hz")
            self._lbl_valid.setText(f"Valid: {len(f.valid_points)}/{len(f.points)}")
            # Range: 优先从 _current_scan 获取（步进模式下更准确），回退到 _scan_buffer
            dists = None
            if self._current_scan and self._current_scan.valid_points:
                dists = [p.distance for p in self._current_scan.valid_points]
            elif self._scan_buffer:
                dists = [p.distance for p in self._scan_buffer if p.is_valid]
            if dists:
                self._lbl_range_info.setText(
                    f"Range: min {min(dists)} / avg {sum(dists)/len(dists):.0f} / max {max(dists)} mm")
            self._update_frame_table(f)
            self._update_raw(f)
        if self._current_scan.valid_points:
            auto = self._auto_range_cb.isChecked()
            if auto:
                max_r = max(p.distance for p in self._current_scan.valid_points) * 1.2
                max_r = max(max_r, 100)
            else:
                max_r = self._max_range
            self._polar_canvas.update_data(self._current_scan, max_r)
            self._cart_canvas.update_data(self._current_scan, max_r)
            self._hist_canvas.update_data(self._current_scan, max_r)
        # 时间线更新（通用：所有profile都更新）
        self._timeline_canvas.update_chart()
        # TFmini Plus: 更新右侧Statistics / Frame Details / Raw Data
        if self._profile_key == 'tfmini_plus':
            dp = self._tfmini_last_point
            if dp:
                self._lbl_fps.setText(f"FPS: {self._tfmini_effective_per_sec:.1f}")
                self._lbl_frames.setText(f"Frames: {self._tfmini_data_points[-1].timestamp + 1 if self._tfmini_data_points else 0}")
                self._lbl_points.setText(f"Points: {len(self._tfmini_data_points)}")
                self._lbl_speed.setText(f"Dist: {dp.distance}mm")
                self._lbl_freq.setText(f"Freq: {self._tfmini_effective_per_sec:.1f} Hz")
                self._lbl_range_info.setText(
                    f"Strength: {dp.strength} | Temp: {dp.temperature_c:.1f}°C")
                # Valid 统计
                total = len(self._tfmini_data_points)
                valid = sum(1 for p in self._tfmini_data_points[-100:] if p.distance > 0 and p.strength > 100)
                self._lbl_valid.setText(f"Valid: {valid} / {min(total, 100)}")
                # Frame Details 表格: 显示最近一条数据
                self._frame_table.setRowCount(1)
                self._frame_table.setItem(0, 0, QTableWidgetItem("0"))
                self._frame_table.setItem(0, 1, QTableWidgetItem(f"{dp.distance_cm:.1f}"))
                self._frame_table.setItem(0, 2, QTableWidgetItem(str(dp.distance)))
                qi = QTableWidgetItem(str(dp.strength))
                if dp.strength > 200: qi.setForeground(QColor("#00ff88"))
                elif dp.strength > 100: qi.setForeground(QColor("#f39c12"))
                else: qi.setForeground(QColor("#ff4444"))
                self._frame_table.setItem(0, 3, qi)
                # Raw Data
                dist_l = dp.distance & 0xFF
                dist_h = (dp.distance >> 8) & 0xFF
                str_l = dp.strength & 0xFF
                str_h = (dp.strength >> 8) & 0xFF
                temp_l = dp.temperature & 0xFF
                temp_h = (dp.temperature >> 8) & 0xFF
                hx = f"59 59 {dist_l:02X} {dist_h:02X} {str_l:02X} {str_h:02X} {temp_l:02X} {temp_h:02X} XX"
                info = (f"#{int(dp.timestamp)} | Dist: {dp.distance}mm({dp.distance_cm:.1f}cm) | "
                        f"Str: {dp.strength} | Temp: {dp.temperature_c:.1f}°C\n"
                        f"HEX: {hx}\n{'-'*60}\n")
                doc = self._raw_text.document()
                if doc.blockCount() > 600:
                    cur = self._raw_text.textCursor()
                    cur.movePosition(cur.MoveOperation.Start)
                    cur.movePosition(cur.MoveOperation.Down, cur.MoveMode.KeepAnchor, 100)
                    cur.removeSelectedText()
                self._raw_text.append(info)

    def _update_frame_table(self, frame):
        pts = frame.points
        self._frame_table.setRowCount(len(pts))
        for i, p in enumerate(pts):
            self._frame_table.setItem(i, 0, QTableWidgetItem(str(i)))
            ai = QTableWidgetItem(f"{p.angle:.2f}")
            di = QTableWidgetItem(str(p.distance))
            qi = QTableWidgetItem(str(p.quality))
            if not p.is_valid:
                ai.setForeground(QColor("#555555"))
                di.setForeground(QColor("#555555"))
                qi.setForeground(QColor("#555555"))
            elif p.quality > 200: qi.setForeground(QColor("#00ff88"))
            elif p.quality > 100: qi.setForeground(QColor("#f39c12"))
            self._frame_table.setItem(i, 1, ai)
            self._frame_table.setItem(i, 2, di)
            self._frame_table.setItem(i, 3, qi)

    def _update_raw(self, frame):
        if frame.raw_data:
            hx = ' '.join(f'{b:02X}' for b in frame.raw_data)
            info = (f"#{self._frame_count} | {frame.speed_rpm:.1f}RPM | "
                    f"{frame.start_angle:.1f}~{frame.end_angle:.1f} | CRC:{frame.crc:#06x}\n"
                    f"HEX: {hx}\n{'-'*60}\n")
            doc = self._raw_text.document()
            if doc.blockCount() > 600:
                cur = self._raw_text.textCursor()
                cur.movePosition(cur.MoveOperation.Start)
                cur.movePosition(cur.MoveOperation.Down, cur.MoveMode.KeepAnchor, 100)
                cur.removeSelectedText()
            self._raw_text.append(info)

    def _get_available_frames(self):
        """获取当前可用于保存的帧列表"""
        if self._file_thread and self._file_thread.loaded:
            return self._file_thread.frames  # 回放模式：所有已加载帧
        elif self._live_frames:
            return self._live_frames  # 串口模式：已接收的所有帧
        return []

    def _save_point_cloud(self):
        """保存当前scan的帧到.dat文件"""
        frames = self._scan_frames
        if not frames:
            self._raw_text.append("[Warning] No frames to save")
            return

        default_dir = self._save_dir if self._save_dir else os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "DATA")
        path, _ = QFileDialog.getSaveFileName(
            self, "Save DAT",
            os.path.join(default_dir, f"scan_{time.strftime('%Y%m%d_%H%M%S')}.DAT"),
            "DAT (*.DAT);;All (*)")
        if not path:
            return

        self._save_dir = os.path.dirname(path)
        self._write_dat(path, frames)
        self._raw_text.append(f"[Info] Saved {len(frames)} frames ({len(frames) * 36} bytes) to {os.path.basename(path)}")

    def _save_all_data(self):
        """保存所有帧到.dat文件"""
        frames = self._get_available_frames()
        if not frames:
            self._raw_text.append("[Warning] No data to save")
            return

        default_dir = self._save_dir if self._save_dir else os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "DATA")
        path, _ = QFileDialog.getSaveFileName(
            self, "Save All DAT",
            os.path.join(default_dir, f"all_{time.strftime('%Y%m%d_%H%M%S')}.DAT"),
            "DAT (*.DAT);;All (*)")
        if not path:
            return

        self._save_dir = os.path.dirname(path)
        self._write_dat(path, frames)
        self._raw_text.append(f"[Info] Saved {len(frames)} frames ({len(frames) * 36} bytes) to {os.path.basename(path)}")

    def _write_dat(self, filepath, frames):
        """将帧列表写入.dat二进制文件（与原始DAT格式一致）"""
        with open(filepath, 'wb') as f:
            for frame in frames:
                if frame.raw_data:
                    f.write(frame.raw_data)

    def _auto_save_frames(self, frames):
        """将scan帧追加到单个.dat文件（Auto Save模式）"""
        if not self._auto_save_cb.isChecked():
            return
        if not frames:
            return

        save_dir = self._save_dir if self._save_dir else os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "DATA")
        os.makedirs(save_dir, exist_ok=True)

        # 首次保存时创建新文件
        if not self._auto_save_path:
            filename = f"auto_save_{time.strftime('%Y%m%d_%H%M%S')}.DAT"
            self._auto_save_path = os.path.join(save_dir, filename)
            self._raw_text.append(f"[AutoSave] Creating {filename}")

        # 追加写入（二进制模式）
        with open(self._auto_save_path, 'ab') as f:
            for frame in frames:
                if frame.raw_data:
                    f.write(frame.raw_data)
        self._save_index += 1

    # ──────────── TFmini Plus 专用方法 ────────────

    def _on_tfmini_data(self, dp):
        """处理 TFmini Plus 数据点"""
        self._tfmini_last_point = dp
        self._tfmini_data_points.append(dp)
        # 限制缓存大小
        if len(self._tfmini_data_points) > 10000:
            self._tfmini_data_points = self._tfmini_data_points[-5000:]
        # 统计有效点数(距离>0且强度>100)
        if dp.distance > 0 and dp.strength > 100:
            self._tfmini_effective_count += 1
        # 更新每秒有效点数
        now = time.time()
        if now - self._tfmini_last_fps_time >= 1.0:
            self._tfmini_effective_per_sec = self._tfmini_effective_count
            self._tfmini_effective_count = 0
            self._tfmini_last_fps_time = now
        # 添加到时间线
        self._timeline_canvas.add_point(dp.distance_cm)

    def _on_tfmini_config_response(self, resp_bytes):
        """处理 TFmini Plus 配置响应"""
        parsed = TFminiPlusResponseParser.parse_response(resp_bytes)
        desc = parsed.get('description', '未知')
        raw_hex = parsed.get('raw', '')
        msg = f"[Config] {desc}: {raw_hex}"
        if 'firmware' in parsed:
            msg += f" → {parsed['firmware']}"
        elif 'success' in parsed:
            msg += f" → {'成功' if parsed['success'] else '失败'}"
        elif 'fps' in parsed:
            msg += f" → {parsed['fps']}Hz"
        elif 'baudrate' in parsed:
            msg += f" → {parsed['baudrate']}"
        self._tfmini_log.append(msg)
        self._raw_text.append(msg)

    def _send_tfmini_command(self, cmd_bytes, desc=""):
        """通用发送TFmini Plus命令"""
        if not self._is_connected:
            self._tfmini_log.append("[Error] 未连接串口，无法发送命令")
            return
        hex_str = ' '.join(f'{b:02X}' for b in cmd_bytes)
        self._tfmini_log.append(f"[TX] {desc}: {hex_str}")
        self._reader_thread.send_command(cmd_bytes)

    def _send_tfmini_preset(self, key):
        """发送预定义命令"""
        cmd_info = TFMINI_PLUS_PRESET_COMMANDS.get(key)
        if cmd_info:
            self._send_tfmini_command(cmd_info['command'], cmd_info['name'])

    def _send_tfmini_fps(self, fps):
        """设置帧率"""
        cmd = TFminiPlusCommandBuilder.set_output_fps(fps)
        self._send_tfmini_command(cmd, f"设置帧率={fps}Hz")

    def _send_tfmini_custom_fps(self):
        """从SpinBox读取并设置帧率"""
        fps = self._tfmini_fps_spin.value()
        self._send_tfmini_fps(fps)

    def _send_tfmini_baudrate(self):
        """设置波特率"""
        baud = self._tfmini_baud_combo.currentData()
        if baud:
            cmd = TFminiPlusCommandBuilder.set_baudrate(baud)
            self._send_tfmini_command(cmd, f"设置波特率={baud}")

    def _send_tfmini_custom_hex(self):
        """发送自定义HEX命令"""
        hex_str = self._tfmini_custom_hex.text().strip()
        if not hex_str:
            return
        try:
            hex_str = hex_str.replace(',', ' ').replace('0x', '').replace('0X', '')
            parts = hex_str.split()
            cmd_bytes = bytes([int(p, 16) for p in parts])
            self._send_tfmini_command(cmd_bytes, "自定义命令")
        except ValueError:
            self._tfmini_log.append(f"[Error] 无效的HEX格式: {hex_str}")

    def closeEvent(self, event):
        if self._reader_thread:
            self._reader_thread.stop()
            self._reader_thread.wait(2000)
        if self._file_thread:
            self._file_thread.stop()
            self._file_thread.wait(2000)
        event.accept()


def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    app.setFont(QFont("Microsoft YaHei", 10))
    w = LidarViewerWindow()
    w.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
