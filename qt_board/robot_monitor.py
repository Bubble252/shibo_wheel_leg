"""
ESP32轮腿机器人实时监控UI
功能:
- 串口数据接收与解析
- 实时参数显示
- 电机速度波形图
- 日志记录
"""

import sys
import re
from datetime import datetime
from collections import deque
import serial
import serial.tools.list_ports
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QLabel, QComboBox, QPushButton, 
                             QTextEdit, QGroupBox, QGridLayout, QSplitter)
from PyQt5.QtCore import QTimer, Qt, pyqtSignal, QThread
from PyQt5.QtGui import QFont
import pyqtgraph as pg

class SerialThread(QThread):
    """串口读取线程"""
    data_received = pyqtSignal(str)
    
    def __init__(self):
        super().__init__()
        self.serial_port = None
        self.running = False
        
    def set_serial(self, port, baudrate=115200):
        try:
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()
            self.serial_port = serial.Serial(port, baudrate, timeout=0.1)
            return True
        except Exception as e:
            print(f"串口打开失败: {e}")
            return False
    
    def run(self):
        self.running = True
        buffer = ""
        while self.running:
            if self.serial_port and self.serial_port.is_open:
                try:
                    if self.serial_port.in_waiting:
                        data = self.serial_port.read(self.serial_port.in_waiting).decode('utf-8', errors='ignore')
                        buffer += data
                        
                        # 按行处理
                        while '\n' in buffer:
                            line, buffer = buffer.split('\n', 1)
                            self.data_received.emit(line.strip())
                except Exception as e:
                    print(f"串口读取错误: {e}")
            self.msleep(10)
    
    def stop(self):
        self.running = False
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()

class RobotMonitorUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ESP32轮腿机器人实时监控")
        self.setGeometry(100, 100, 1400, 900)
        
        # 数据存储
        self.max_points = 500  # 波形显示最大点数
        self.time_data = deque(maxlen=self.max_points)
        self.encoder1_vel_data = deque(maxlen=self.max_points)
        self.encoder2_vel_data = deque(maxlen=self.max_points)
        self.control_freq_data = deque(maxlen=self.max_points)
        self.time_counter = 0
        
        # 当前数据
        self.current_data = {
            'control_freq': 0,
            'encoder_feedback_freq': 0,
            'battery': 0,
            'imu_y': 0,
            'imu_z': 0,
            'encoder1_angle': 0,
            'encoder1_vel': 0,
            'encoder2_angle': 0,
            'encoder2_vel': 0,
            'servo1_pos': 0,
            'servo2_pos': 0
        }
        
        # 串口线程
        self.serial_thread = SerialThread()
        self.serial_thread.data_received.connect(self.process_serial_data)
        
        self.init_ui()
        
        # 定时刷新波形
        self.plot_timer = QTimer()
        self.plot_timer.timeout.connect(self.update_plots)
        self.plot_timer.start(50)  # 20Hz刷新
        
    def init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        
        # ===== 串口控制区 =====
        serial_group = QGroupBox("串口设置")
        serial_layout = QHBoxLayout()
        
        self.port_combo = QComboBox()
        self.refresh_ports()
        serial_layout.addWidget(QLabel("串口:"))
        serial_layout.addWidget(self.port_combo)
        
        self.refresh_btn = QPushButton("刷新")
        self.refresh_btn.clicked.connect(self.refresh_ports)
        serial_layout.addWidget(self.refresh_btn)
        
        self.connect_btn = QPushButton("连接")
        self.connect_btn.clicked.connect(self.toggle_connection)
        serial_layout.addWidget(self.connect_btn)
        
        self.status_label = QLabel("未连接")
        self.status_label.setStyleSheet("color: red; font-weight: bold;")
        serial_layout.addWidget(self.status_label)
        
        serial_layout.addStretch()
        serial_group.setLayout(serial_layout)
        main_layout.addWidget(serial_group)
        
        # ===== 分割器:左侧数据+右侧波形 =====
        splitter = QSplitter(Qt.Horizontal)
        
        # ----- 左侧:实时数据显示 -----
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)
        
        # 系统状态
        sys_group = QGroupBox("系统状态")
        sys_layout = QGridLayout()
        sys_layout.setSpacing(10)
        
        self.freq_label = self.create_data_label("0 Hz")
        self.encoder_freq_label = self.create_data_label("0 Hz")
        self.battery_label = self.create_data_label("0.00 V")
        
        sys_layout.addWidget(QLabel("控制频率:"), 0, 0)
        sys_layout.addWidget(self.freq_label, 0, 1)
        sys_layout.addWidget(QLabel("编码器反馈频率:"), 1, 0)
        sys_layout.addWidget(self.encoder_freq_label, 1, 1)
        sys_layout.addWidget(QLabel("电池电压:"), 2, 0)
        sys_layout.addWidget(self.battery_label, 2, 1)
        
        sys_group.setLayout(sys_layout)
        left_layout.addWidget(sys_group)
        
        # IMU数据
        imu_group = QGroupBox("IMU姿态")
        imu_layout = QGridLayout()
        imu_layout.setSpacing(10)
        
        self.imu_y_label = self.create_data_label("0.00°")
        self.imu_z_label = self.create_data_label("0.00°")
        
        imu_layout.addWidget(QLabel("俯仰角(Y):"), 0, 0)
        imu_layout.addWidget(self.imu_y_label, 0, 1)
        imu_layout.addWidget(QLabel("偏航角(Z):"), 1, 0)
        imu_layout.addWidget(self.imu_z_label, 1, 1)
        
        imu_group.setLayout(imu_layout)
        left_layout.addWidget(imu_group)
        
        # 编码器数据
        enc_group = QGroupBox("编码器反馈")
        enc_layout = QGridLayout()
        enc_layout.setSpacing(10)
        
        self.enc1_angle_label = self.create_data_label("0.00 rad")
        self.enc1_vel_label = self.create_data_label("0.00 rad/s")
        self.enc2_angle_label = self.create_data_label("0.00 rad")
        self.enc2_vel_label = self.create_data_label("0.00 rad/s")
        
        enc_layout.addWidget(QLabel("编码器1 角度:"), 0, 0)
        enc_layout.addWidget(self.enc1_angle_label, 0, 1)
        enc_layout.addWidget(QLabel("编码器1 速度:"), 1, 0)
        enc_layout.addWidget(self.enc1_vel_label, 1, 1)
        enc_layout.addWidget(QLabel("编码器2 角度:"), 2, 0)
        enc_layout.addWidget(self.enc2_angle_label, 2, 1)
        enc_layout.addWidget(QLabel("编码器2 速度:"), 3, 0)
        enc_layout.addWidget(self.enc2_vel_label, 3, 1)
        
        enc_group.setLayout(enc_layout)
        left_layout.addWidget(enc_group)
        
        # 舵机数据
        servo_group = QGroupBox("舵机位置")
        servo_layout = QGridLayout()
        servo_layout.setSpacing(10)
        
        self.servo1_label = self.create_data_label("0")
        self.servo2_label = self.create_data_label("0")
        
        servo_layout.addWidget(QLabel("舵机1 (ID1):"), 0, 0)
        servo_layout.addWidget(self.servo1_label, 0, 1)
        servo_layout.addWidget(QLabel("舵机2 (ID2):"), 1, 0)
        servo_layout.addWidget(self.servo2_label, 1, 1)
        
        servo_group.setLayout(servo_layout)
        left_layout.addWidget(servo_group)
        
        # 日志区域
        log_group = QGroupBox("系统日志")
        log_layout = QVBoxLayout()
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMaximumHeight(150)
        self.log_text.setStyleSheet("background-color: #1e1e1e; color: #d4d4d4; font-family: Consolas;")
        log_layout.addWidget(self.log_text)
        log_group.setLayout(log_layout)
        left_layout.addWidget(log_group)
        
        left_layout.addStretch()
        
        # ----- 右侧:波形图 -----
        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)
        
        # 波形图1: 控制频率
        self.freq_plot = pg.PlotWidget(title="控制频率")
        self.freq_plot.setLabel('left', '频率', units='Hz')
        self.freq_plot.setLabel('bottom', '时间', units='s')
        self.freq_plot.showGrid(x=True, y=True, alpha=0.3)
        self.freq_curve = self.freq_plot.plot(pen=pg.mkPen(color='y', width=2))
        right_layout.addWidget(self.freq_plot)
        
        # 波形图2: 电机速度
        self.motor_plot = pg.PlotWidget(title="电机速度反馈")
        self.motor_plot.setLabel('left', '角速度', units='rad/s')
        self.motor_plot.setLabel('bottom', '时间', units='s')
        self.motor_plot.showGrid(x=True, y=True, alpha=0.3)
        self.motor_plot.addLegend()
        self.motor1_curve = self.motor_plot.plot(pen=pg.mkPen(color='g', width=2), name='电机1')
        self.motor2_curve = self.motor_plot.plot(pen=pg.mkPen(color='c', width=2), name='电机2')
        # 目标速度参考线
        self.motor_plot.addItem(pg.InfiniteLine(pos=3.0, angle=0, pen=pg.mkPen('r', style=Qt.DashLine, width=1)))
        right_layout.addWidget(self.motor_plot)
        
        splitter.addWidget(left_widget)
        splitter.addWidget(right_widget)
        splitter.setStretchFactor(0, 1)
        splitter.setStretchFactor(1, 2)
        
        main_layout.addWidget(splitter)
        
    def create_data_label(self, text):
        """创建数据显示标签"""
        label = QLabel(text)
        label.setStyleSheet("font-size: 16px; font-weight: bold; color: #00ff00; background-color: #2b2b2b; padding: 5px;")
        label.setAlignment(Qt.AlignCenter)
        return label
        
    def refresh_ports(self):
        """刷新可用串口"""
        self.port_combo.clear()
        ports = serial.tools.list_ports.comports()
        for port in ports:
            self.port_combo.addItem(f"{port.device} - {port.description}")
            
    def toggle_connection(self):
        """切换串口连接状态"""
        if not self.serial_thread.running:
            port = self.port_combo.currentText().split(' ')[0]
            if self.serial_thread.set_serial(port):
                self.serial_thread.start()
                self.connect_btn.setText("断开")
                self.status_label.setText("已连接")
                self.status_label.setStyleSheet("color: green; font-weight: bold;")
                self.log_message(f"已连接到 {port}")
            else:
                self.log_message("串口连接失败!", error=True)
        else:
            self.serial_thread.stop()
            self.serial_thread.wait()
            self.connect_btn.setText("连接")
            self.status_label.setText("未连接")
            self.status_label.setStyleSheet("color: red; font-weight: bold;")
            self.log_message("串口已断开")
            
    def process_serial_data(self, line):
        """解析串口数据"""
        if not line:
            return
            
        # 解析控制频率: "控制频率: 800.5 Hz | 周期: 1.25 ms"
        match = re.search(r'控制频率:\s*([\d.]+)\s*Hz', line)
        if match:
            self.current_data['control_freq'] = float(match.group(1))
            self.freq_label.setText(f"{self.current_data['control_freq']:.1f} Hz")
        
        # 解析编码器反馈频率: "编码器反馈频率: 850.3 Hz"
        match = re.search(r'编码器反馈频率:\s*([\d.]+)\s*Hz', line)
        if match:
            self.current_data['encoder_feedback_freq'] = float(match.group(1))
            self.encoder_freq_label.setText(f"{self.current_data['encoder_feedback_freq']:.1f} Hz")
            
        # 解析电压: "电压: 8.13V | LED: 亮"
        match = re.search(r'电压:\s*([\d.]+)V', line)
        if match:
            self.current_data['battery'] = float(match.group(1))
            self.battery_label.setText(f"{self.current_data['battery']:.2f} V")
            
        # 解析IMU: "IMU Y轴角度: 0.74° | Z轴角度: 2.35°"
        match = re.search(r'Y轴角度:\s*([-\d.]+)°.*Z轴角度:\s*([-\d.]+)°', line)
        if match:
            self.current_data['imu_y'] = float(match.group(1))
            self.current_data['imu_z'] = float(match.group(2))
            self.imu_y_label.setText(f"{self.current_data['imu_y']:.2f}°")
            self.imu_z_label.setText(f"{self.current_data['imu_z']:.2f}°")
            
        # 解析编码器1: "编码器1: 158.0813 rad (0.0000 rad/s) | 目标: 3.0 rad/s"
        match = re.search(r'编码器1:\s*([-\d.]+)\s*rad\s*\(([-\d.]+)\s*rad/s\)', line)
        if match:
            self.current_data['encoder1_angle'] = float(match.group(1))
            self.current_data['encoder1_vel'] = float(match.group(2))
            self.enc1_angle_label.setText(f"{self.current_data['encoder1_angle']:.2f} rad")
            self.enc1_vel_label.setText(f"{self.current_data['encoder1_vel']:.2f} rad/s")
            
        # 解析编码器2
        match = re.search(r'编码器2:\s*([-\d.]+)\s*rad\s*\(([-\d.]+)\s*rad/s\)', line)
        if match:
            self.current_data['encoder2_angle'] = float(match.group(1))
            self.current_data['encoder2_vel'] = float(match.group(2))
            self.enc2_angle_label.setText(f"{self.current_data['encoder2_angle']:.2f} rad")
            self.enc2_vel_label.setText(f"{self.current_data['encoder2_vel']:.2f} rad/s")
            
            # 当编码器2数据更新时,记录一次完整数据到波形缓冲区
            self.time_counter += 0.2  # 每200ms一个数据点
            self.time_data.append(self.time_counter)
            self.control_freq_data.append(self.current_data['control_freq'])
            self.encoder1_vel_data.append(self.current_data['encoder1_vel'])
            self.encoder2_vel_data.append(self.current_data['encoder2_vel'])
            
        # 解析舵机: "舵机位置: ID1=2050 | ID2=2083"
        match = re.search(r'ID1=(\d+).*ID2=(\d+)', line)
        if match:
            self.current_data['servo1_pos'] = int(match.group(1))
            self.current_data['servo2_pos'] = int(match.group(2))
            self.servo1_label.setText(str(self.current_data['servo1_pos']))
            self.servo2_label.setText(str(self.current_data['servo2_pos']))
            
        # 显示特殊日志
        if any(keyword in line for keyword in ['初始化', '任务', '启动', '失败', 'I2C', '✓', '警告']):
            self.log_message(line)
            
    def update_plots(self):
        """更新波形图"""
        if len(self.time_data) > 1:
            time_list = list(self.time_data)
            self.freq_curve.setData(time_list, list(self.control_freq_data))
            self.motor1_curve.setData(time_list, list(self.encoder1_vel_data))
            self.motor2_curve.setData(time_list, list(self.encoder2_vel_data))
            
    def log_message(self, msg, error=False):
        """记录日志"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        color = "red" if error else "white"
        self.log_text.append(f'<span style="color: gray;">[{timestamp}]</span> '
                            f'<span style="color: {color};">{msg}</span>')
        
    def closeEvent(self, event):
        """关闭窗口时清理资源"""
        self.serial_thread.stop()
        self.serial_thread.wait()
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    app.setStyle('Fusion')  # 使用Fusion主题
    
    # 深色主题
    from PyQt5.QtGui import QPalette, QColor
    palette = QPalette()
    palette.setColor(QPalette.Window, QColor(53, 53, 53))
    palette.setColor(QPalette.WindowText, Qt.white)
    palette.setColor(QPalette.Base, QColor(25, 25, 25))
    palette.setColor(QPalette.AlternateBase, QColor(53, 53, 53))
    palette.setColor(QPalette.ToolTipBase, Qt.white)
    palette.setColor(QPalette.ToolTipText, Qt.white)
    palette.setColor(QPalette.Text, Qt.white)
    palette.setColor(QPalette.Button, QColor(53, 53, 53))
    palette.setColor(QPalette.ButtonText, Qt.white)
    palette.setColor(QPalette.BrightText, Qt.red)
    palette.setColor(QPalette.Link, QColor(42, 130, 218))
    palette.setColor(QPalette.Highlight, QColor(42, 130, 218))
    palette.setColor(QPalette.HighlightedText, Qt.black)
    app.setPalette(palette)
    
    window = RobotMonitorUI()
    window.show()
    sys.exit(app.exec_())
