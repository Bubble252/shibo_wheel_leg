"""
SimpleFOC Commander PID参数调试工具
功能:
- 通过串口使用Commander协议读取/设置PID参数
- 图形化界面调节P、I、D、Limit参数
- 一键查询当前参数值
- 支持多个PID控制器切换
"""

import sys
import serial
import serial.tools.list_ports
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QLabel, QComboBox, QPushButton, 
                             QTextEdit, QGroupBox, QGridLayout, QLineEdit,
                             QTabWidget, QDoubleSpinBox, QMessageBox)
from PyQt5.QtCore import QTimer, Qt, pyqtSignal, QThread
from PyQt5.QtGui import QFont, QPalette, QColor
import re

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
    
    def send_command(self, cmd):
        """发送命令到串口"""
        if self.serial_port and self.serial_port.is_open:
            try:
                # 清空输入缓冲区,避免旧数据干扰
                self.serial_port.reset_input_buffer()
                # 发送命令 - SimpleFOC Commander需要换行符!
                cmd_with_newline = cmd + '\n'
                self.serial_port.write(cmd_with_newline.encode('utf-8'))
                # 等待数据发送完成
                self.serial_port.flush()
                # 短暂延迟让ESP32处理命令
                self.msleep(50)
                return True
            except Exception as e:
                print(f"发送命令失败: {e}")
                return False
        return False
    
    def run(self):
        self.running = True
        buffer = ""
        while self.running:
            if self.serial_port and self.serial_port.is_open:
                try:
                    if self.serial_port.in_waiting:
                        raw_data = self.serial_port.read(self.serial_port.in_waiting)
                        # 显示原始字节(用于调试)
                        # print(f"Raw bytes: {raw_data.hex()}")
                        
                        data = raw_data.decode('utf-8', errors='ignore')
                        buffer += data
                        
                        # 按行处理
                        while '\n' in buffer:
                            line, buffer = buffer.split('\n', 1)
                            line = line.strip()
                            if line:  # 只发送非空行
                                self.data_received.emit(line)
                except Exception as e:
                    print(f"串口读取错误: {e}")
            self.msleep(10)
    
    def stop(self):
        self.running = False
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()


class PIDControlPanel(QWidget):
    """单个PID控制器的参数面板"""
    
    def __init__(self, name, commander_id, parent=None):
        super().__init__(parent)
        self.name = name
        self.commander_id = commander_id  # Commander命令ID (如 'A', 'B', 'C')
        self.parent_window = parent
        self.init_ui()
        
    def init_ui(self):
        layout = QVBoxLayout(self)
        
        # 参数输入区域
        param_group = QGroupBox(f"{self.name} 参数设置")
        param_layout = QGridLayout()
        param_layout.setSpacing(15)
        
        # P参数
        param_layout.addWidget(QLabel("P (比例):"), 0, 0)
        self.p_input = QDoubleSpinBox()
        self.p_input.setRange(-1000, 1000)
        self.p_input.setDecimals(4)
        self.p_input.setSingleStep(0.1)
        self.p_input.setStyleSheet("font-size: 14px; padding: 5px;")
        param_layout.addWidget(self.p_input, 0, 1)
        
        self.p_set_btn = QPushButton("设置 P")
        self.p_set_btn.clicked.connect(lambda: self.set_param('P', self.p_input.value()))
        param_layout.addWidget(self.p_set_btn, 0, 2)
        
        # I参数
        param_layout.addWidget(QLabel("I (积分):"), 1, 0)
        self.i_input = QDoubleSpinBox()
        self.i_input.setRange(-1000, 1000)
        self.i_input.setDecimals(4)
        self.i_input.setSingleStep(0.1)
        self.i_input.setStyleSheet("font-size: 14px; padding: 5px;")
        param_layout.addWidget(self.i_input, 1, 1)
        
        self.i_set_btn = QPushButton("设置 I")
        self.i_set_btn.clicked.connect(lambda: self.set_param('I', self.i_input.value()))
        param_layout.addWidget(self.i_set_btn, 1, 2)
        
        # D参数
        param_layout.addWidget(QLabel("D (微分):"), 2, 0)
        self.d_input = QDoubleSpinBox()
        self.d_input.setRange(-1000, 1000)
        self.d_input.setDecimals(4)
        self.d_input.setSingleStep(0.1)
        self.d_input.setStyleSheet("font-size: 14px; padding: 5px;")
        param_layout.addWidget(self.d_input, 2, 1)
        
        self.d_set_btn = QPushButton("设置 D")
        self.d_set_btn.clicked.connect(lambda: self.set_param('D', self.d_input.value()))
        param_layout.addWidget(self.d_set_btn, 2, 3)
        
        # Limit参数
        param_layout.addWidget(QLabel("Limit (限幅):"), 3, 0)
        self.limit_input = QDoubleSpinBox()
        self.limit_input.setRange(0, 1000)
        self.limit_input.setDecimals(4)
        self.limit_input.setSingleStep(0.5)
        self.limit_input.setStyleSheet("font-size: 14px; padding: 5px;")
        param_layout.addWidget(self.limit_input, 3, 1)
        
        self.limit_set_btn = QPushButton("设置 Limit")
        self.limit_set_btn.clicked.connect(lambda: self.set_param('L', self.limit_input.value()))
        param_layout.addWidget(self.limit_set_btn, 3, 2)
        
        # Ramp参数 (输出变化率限制)
        param_layout.addWidget(QLabel("Ramp (变化率):"), 4, 0)
        self.ramp_input = QDoubleSpinBox()
        self.ramp_input.setRange(0, 1000000)
        self.ramp_input.setDecimals(0)
        self.ramp_input.setSingleStep(1000)
        self.ramp_input.setStyleSheet("font-size: 14px; padding: 5px;")
        param_layout.addWidget(self.ramp_input, 4, 1)
        
        self.ramp_set_btn = QPushButton("设置 Ramp")
        self.ramp_set_btn.clicked.connect(lambda: self.set_param('R', self.ramp_input.value()))
        param_layout.addWidget(self.ramp_set_btn, 4, 2)
        
        param_group.setLayout(param_layout)
        layout.addWidget(param_group)
        
        # 快捷操作区
        action_group = QGroupBox("快捷操作")
        action_layout = QHBoxLayout()
        
        self.query_btn = QPushButton("🔍 查询当前参数")
        self.query_btn.setStyleSheet("font-size: 14px; font-weight: bold; padding: 10px;")
        self.query_btn.clicked.connect(self.query_params)
        action_layout.addWidget(self.query_btn)
        
        self.set_all_btn = QPushButton("📤 发送全部参数")
        self.set_all_btn.setStyleSheet("font-size: 14px; font-weight: bold; padding: 10px;")
        self.set_all_btn.clicked.connect(self.set_all_params)
        action_layout.addWidget(self.set_all_btn)
        
        self.reset_btn = QPushButton("🔄 重置为0")
        self.reset_btn.setStyleSheet("font-size: 14px; padding: 10px;")
        self.reset_btn.clicked.connect(self.reset_params)
        action_layout.addWidget(self.reset_btn)
        
        action_group.setLayout(action_layout)
        layout.addWidget(action_group)
        
        # 当前参数显示区
        display_group = QGroupBox("当前参数值 (从ESP32读取)")
        display_layout = QGridLayout()
        display_layout.setSpacing(10)
        
        self.p_display = self.create_param_display("--")
        self.i_display = self.create_param_display("--")
        self.d_display = self.create_param_display("--")
        self.limit_display = self.create_param_display("--")
        self.ramp_display = self.create_param_display("--")
        
        display_layout.addWidget(QLabel("P:"), 0, 0)
        display_layout.addWidget(self.p_display, 0, 1)
        display_layout.addWidget(QLabel("I:"), 0, 2)
        display_layout.addWidget(self.i_display, 0, 3)
        display_layout.addWidget(QLabel("D:"), 1, 0)
        display_layout.addWidget(self.d_display, 1, 1)
        display_layout.addWidget(QLabel("Limit:"), 1, 2)
        display_layout.addWidget(self.limit_display, 1, 3)
        display_layout.addWidget(QLabel("Ramp:"), 2, 0)
        display_layout.addWidget(self.ramp_display, 2, 1)
        
        display_group.setLayout(display_layout)
        layout.addWidget(display_group)
        
        layout.addStretch()
        
    def create_param_display(self, text):
        """创建参数显示标签"""
        label = QLabel(text)
        label.setStyleSheet("font-size: 16px; font-weight: bold; color: #00ff00; "
                          "background-color: #2b2b2b; padding: 8px; border-radius: 3px;")
        label.setAlignment(Qt.AlignCenter)
        return label
    
    def set_param(self, param_type, value):
        """设置单个参数
        Commander格式: <ID><param_char><value>
        例如: AP1.5 设置P=1.5, AI0.5 设置I=0.5
        """
        if not self.parent_window.is_connected():
            QMessageBox.warning(self, "警告", "请先连接串口!")
            return
        
        # 构建命令 - 所有参数都需要子命令字母
        cmd = f"{self.commander_id}{param_type}{value}"
        
        if self.parent_window.send_command(cmd):
            self.parent_window.log(f"发送: {cmd} -> {self.name} {param_type}={value}")
        else:
            self.parent_window.log(f"✗ 发送失败: {cmd}", is_error=True)
        
    def set_all_params(self):
        """发送所有参数"""
        if not self.parent_window.is_connected():
            QMessageBox.warning(self, "警告", "请先连接串口!")
            return
        
        p = self.p_input.value()
        i = self.i_input.value()
        d = self.d_input.value()
        limit = self.limit_input.value()
        ramp = self.ramp_input.value()
        
        # 依次发送所有参数(添加延迟避免粘包)
        import time
        self.parent_window.send_command(f"{self.commander_id}P{p}")
        time.sleep(0.05)
        self.parent_window.send_command(f"{self.commander_id}I{i}")
        time.sleep(0.05)
        self.parent_window.send_command(f"{self.commander_id}D{d}")
        time.sleep(0.05)
        self.parent_window.send_command(f"{self.commander_id}L{limit}")
        time.sleep(0.05)
        self.parent_window.send_command(f"{self.commander_id}R{ramp}")
        
        self.parent_window.log(f"已发送 {self.name} 全部参数: P={p}, I={i}, D={d}, L={limit}, R={ramp}")
        
    def query_params(self):
        """查询当前参数"""
        if not self.parent_window.is_connected():
            QMessageBox.warning(self, "警告", "请先连接串口!")
            return
        
        # Commander查询格式: <ID>? 查询所有参数
        cmd = f"{self.commander_id}?"
        self.parent_window.send_command(cmd)
        self.parent_window.log(f"查询 {self.name} 参数...")
        
    def reset_params(self):
        """重置所有参数为0"""
        self.p_input.setValue(0)
        self.i_input.setValue(0)
        self.d_input.setValue(0)
        self.limit_input.setValue(0)
        self.ramp_input.setValue(100000)
        
    def update_display(self, p, i, d, limit, ramp):
        """更新显示的参数值"""
        self.p_display.setText(f"{p:.4f}")
        self.i_display.setText(f"{i:.4f}")
        self.d_display.setText(f"{d:.4f}")
        self.limit_display.setText(f"{limit:.4f}")
        self.ramp_display.setText(f"{ramp:.0f}")


class LPFControlPanel(QWidget):
    """低通滤波器参数面板"""
    
    def __init__(self, name, commander_id, parent=None):
        super().__init__(parent)
        self.name = name
        self.commander_id = commander_id
        self.parent_window = parent
        self.init_ui()
        
    def init_ui(self):
        layout = QVBoxLayout(self)
        
        # Tf参数输入
        param_group = QGroupBox(f"{self.name} 滤波器设置")
        param_layout = QGridLayout()
        param_layout.setSpacing(15)
        
        param_layout.addWidget(QLabel("Tf (时间常数):"), 0, 0)
        self.tf_input = QDoubleSpinBox()
        self.tf_input.setRange(0, 10)
        self.tf_input.setDecimals(4)
        self.tf_input.setSingleStep(0.01)
        self.tf_input.setStyleSheet("font-size: 14px; padding: 5px;")
        param_layout.addWidget(self.tf_input, 0, 1)
        
        self.tf_set_btn = QPushButton("设置 Tf")
        self.tf_set_btn.clicked.connect(self.set_tf)
        param_layout.addWidget(self.tf_set_btn, 0, 2)
        
        param_group.setLayout(param_layout)
        layout.addWidget(param_group)
        
        # 快捷操作
        action_layout = QHBoxLayout()
        
        self.query_btn = QPushButton("🔍 查询当前 Tf")
        self.query_btn.setStyleSheet("font-size: 14px; font-weight: bold; padding: 10px;")
        self.query_btn.clicked.connect(self.query_tf)
        action_layout.addWidget(self.query_btn)
        
        layout.addLayout(action_layout)
        
        # 当前值显示
        display_group = QGroupBox("当前 Tf 值")
        display_layout = QHBoxLayout()
        
        self.tf_display = QLabel("--")
        self.tf_display.setStyleSheet("font-size: 16px; font-weight: bold; color: #00ff00; "
                                     "background-color: #2b2b2b; padding: 8px;")
        self.tf_display.setAlignment(Qt.AlignCenter)
        display_layout.addWidget(self.tf_display)
        
        display_group.setLayout(display_layout)
        layout.addWidget(display_group)
        
        layout.addStretch()
        
    def set_tf(self):
        """设置Tf参数"""
        if not self.parent_window.is_connected():
            QMessageBox.warning(self, "警告", "请先连接串口!")
            return
        
        tf = self.tf_input.value()
        cmd = f"{self.commander_id}{tf}"
        self.parent_window.send_command(cmd)
        self.parent_window.log(f"发送: {cmd} -> {self.name} Tf={tf}")
        
    def query_tf(self):
        """查询Tf参数"""
        if not self.parent_window.is_connected():
            QMessageBox.warning(self, "警告", "请先连接串口!")
            return
        
        cmd = f"{self.commander_id}?"
        self.parent_window.send_command(cmd)
        self.parent_window.log(f"查询 {self.name} Tf...")
        
    def update_display(self, tf):
        """更新显示值"""
        self.tf_display.setText(f"{tf:.4f}")


class PIDTunerUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("SimpleFOC Commander PID调参工具")
        self.setGeometry(100, 100, 1000, 700)
        
        # 串口线程
        self.serial_thread = SerialThread()
        self.serial_thread.data_received.connect(self.process_serial_data)
        
        # 存储所有控制面板
        self.pid_panels = {}
        self.lpf_panels = {}
        
        self.init_ui()
        
    def init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        
        # ===== 串口控制区 =====
        serial_group = QGroupBox("串口连接")
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
        
        self.test_btn = QPushButton("🔧 测试通信")
        self.test_btn.clicked.connect(self.test_communication)
        self.test_btn.setEnabled(False)
        serial_layout.addWidget(self.test_btn)
        
        self.status_label = QLabel("未连接")
        self.status_label.setStyleSheet("color: red; font-weight: bold; font-size: 14px;")
        serial_layout.addWidget(self.status_label)
        
        serial_layout.addStretch()
        serial_group.setLayout(serial_layout)
        main_layout.addWidget(serial_group)
        
        # ===== Tab切换区 =====
        self.tab_widget = QTabWidget()
        
        # 根据wl_pro_robot.cpp中的Commander映射创建标签页
        # PID控制器
        self.pid_panels['angle'] = PIDControlPanel("角度控制 (Angle)", "A", self)
        self.tab_widget.addTab(self.pid_panels['angle'], "A - 角度PID")
        
        self.pid_panels['gyro'] = PIDControlPanel("角速度控制 (Gyro)", "B", self)
        self.tab_widget.addTab(self.pid_panels['gyro'], "B - 角速度PID")
        
        self.pid_panels['distance'] = PIDControlPanel("位移控制 (Distance)", "C", self)
        self.tab_widget.addTab(self.pid_panels['distance'], "C - 位移PID")
        
        self.pid_panels['speed'] = PIDControlPanel("速度控制 (Speed)", "D", self)
        self.tab_widget.addTab(self.pid_panels['speed'], "D - 速度PID")
        
        self.pid_panels['yaw_angle'] = PIDControlPanel("YAW角度控制", "E", self)
        self.tab_widget.addTab(self.pid_panels['yaw_angle'], "E - YAW角度PID")
        
        self.pid_panels['yaw_gyro'] = PIDControlPanel("YAW角速度控制", "F", self)
        self.tab_widget.addTab(self.pid_panels['yaw_gyro'], "F - YAW角速度PID")
        
        self.pid_panels['lqr_u'] = PIDControlPanel("LQR输出补偿", "H", self)
        self.tab_widget.addTab(self.pid_panels['lqr_u'], "H - LQR输出PID")
        
        self.pid_panels['zeropoint'] = PIDControlPanel("零点自适应", "I", self)
        self.tab_widget.addTab(self.pid_panels['zeropoint'], "I - 零点PID")
        
        self.pid_panels['roll_angle'] = PIDControlPanel("Roll轴平衡", "K", self)
        self.tab_widget.addTab(self.pid_panels['roll_angle'], "K - Roll角度PID")
        
        # 低通滤波器
        self.lpf_panels['joyy'] = LPFControlPanel("摇杆Y轴滤波", "G", self)
        self.tab_widget.addTab(self.lpf_panels['joyy'], "G - 摇杆滤波")
        
        self.lpf_panels['zeropoint'] = LPFControlPanel("零点滤波", "J", self)
        self.tab_widget.addTab(self.lpf_panels['zeropoint'], "J - 零点滤波")
        
        self.lpf_panels['roll'] = LPFControlPanel("Roll角度滤波", "L", self)
        self.tab_widget.addTab(self.lpf_panels['roll'], "L - Roll滤波")
        
        main_layout.addWidget(self.tab_widget)
        
        # ===== 日志区 =====
        log_group = QGroupBox("通信日志")
        log_layout = QVBoxLayout()
        
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMaximumHeight(150)
        self.log_text.setStyleSheet("background-color: #1e1e1e; color: #d4d4d4; "
                                    "font-family: Consolas; font-size: 12px;")
        log_layout.addWidget(self.log_text)
        
        # 日志控制按钮
        log_btn_layout = QHBoxLayout()
        clear_log_btn = QPushButton("清除日志")
        clear_log_btn.clicked.connect(self.log_text.clear)
        log_btn_layout.addWidget(clear_log_btn)
        
        help_btn = QPushButton("📖 使用说明")
        help_btn.clicked.connect(self.show_help)
        log_btn_layout.addWidget(help_btn)
        
        debug_btn = QPushButton("🐛 显示所有串口数据")
        debug_btn.setCheckable(True)
        debug_btn.toggled.connect(self.toggle_debug_mode)
        log_btn_layout.addWidget(debug_btn)
        
        log_layout.addLayout(log_btn_layout)
        
        log_group.setLayout(log_layout)
        main_layout.addWidget(log_group)
        
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
                self.test_btn.setEnabled(True)
                self.status_label.setText("✓ 已连接")
                self.status_label.setStyleSheet("color: green; font-weight: bold; font-size: 14px;")
                self.log(f"✓ 已连接到 {port}")
                self.log("💡 提示: 先点击'测试通信'检查Commander是否正常工作")
            else:
                QMessageBox.critical(self, "错误", "串口连接失败!")
                self.log("✗ 串口连接失败")
        else:
            self.serial_thread.stop()
            self.serial_thread.wait()
            self.connect_btn.setText("连接")
            self.test_btn.setEnabled(False)
            self.status_label.setText("未连接")
            self.status_label.setStyleSheet("color: red; font-weight: bold; font-size: 14px;")
            self.log("串口已断开")
    
    def test_communication(self):
        """测试Commander通信"""
        if not self.is_connected():
            return
        
        self.log("="*50)
        self.log("🔧 开始测试Commander通信...")
        
        # 显示串口信息
        port_info = self.serial_thread.serial_port
        self.log(f"📌 串口: {port_info.port}")
        self.log(f"📌 波特率: {port_info.baudrate}")
        self.log(f"📌 数据位: {port_info.bytesize}, 停止位: {port_info.stopbits}")
        
        # 测试1: 发送简单命令
        self.log("\\n[测试1] 发送设置命令: A1.5")
        self.send_command("A1.5")
        self.log("⏳ 等待ESP32响应...")
        self.log("")
        self.log("📋 结果判断:")
        self.log("   ✓ <b>没有任何响应</b> = <b>正常!</b> (Commander默认静默模式)")
        self.log("   ✗ 看到'err' = 命令格式错误或Commander配置问题")
        self.log("   ℹ 看到参数值 = ESP32已开启verbose模式(很好!)")
        self.log("")
        self.log("💡 验证方法:")
        self.log("   1. 如果没有'err',说明命令已被ESP32接收")
        self.log("   2. 观察机器人是否响应参数变化来验证")
        self.log("   3. 或者在ESP32代码中添加 command.verbose 来开启响应")
        self.log("="*50)
            
    def is_connected(self):
        """检查串口是否已连接"""
        return self.serial_thread.running
    
    def send_command(self, cmd):
        """发送命令到串口"""
        return self.serial_thread.send_command(cmd)
    
    def process_serial_data(self, line):
        """处理串口接收的数据"""
        # 显示所有接收的数据
        self.log(f"← {line}", is_receive=True)
        
        # 检测错误响应
        if "err" in line.lower():
            self.log("⚠ ESP32返回错误,可能是命令格式不正确或Commander未就绪", is_error=True)
            return
        
        # 检测是否是普通的调试输出(不是Commander响应)
        if any(keyword in line for keyword in ['MOT', 'Angle', '角度', '频率', 'Hz', 'rad', 'V']):
            # 这是ESP32的普通调试输出,不是Commander响应
            return
        
        # 解析Commander返回的PID参数
        # 格式示例: "PID: P: 1.5000 I: 0.0000 D: 0.0000 R: 100000.0000 L: 8.0000"
        match = re.search(r'PID:\s*P:\s*([-\d.]+)\s*I:\s*([-\d.]+)\s*D:\s*([-\d.]+)\s*R:\s*([-\d.]+)\s*L:\s*([-\d.]+)', line)
        if match:
            p = float(match.group(1))
            i = float(match.group(2))
            d = float(match.group(3))
            ramp = float(match.group(4))
            limit = float(match.group(5))
            
            # 更新当前激活标签页的显示
            current_widget = self.tab_widget.currentWidget()
            if isinstance(current_widget, PIDControlPanel):
                current_widget.update_display(p, i, d, limit, ramp)
                
        # 解析LPF返回的Tf参数
        # 格式示例: "LPF: Tf: 0.2000"
        match = re.search(r'LPF:\s*Tf:\s*([-\d.]+)', line)
        if match:
            tf = float(match.group(1))
            current_widget = self.tab_widget.currentWidget()
            if isinstance(current_widget, LPFControlPanel):
                current_widget.update_display(tf)
    
    def log(self, msg, is_receive=False, is_error=False):
        """记录日志"""
        from datetime import datetime
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        
        if is_error:
            color = "#ff4444"  # 红色表示错误
            prefix = "ERR"
        elif is_receive:
            color = "#00aaff"  # 蓝色表示接收
            prefix = "RX"
        else:
            color = "#00ff00"  # 绿色表示发送
            prefix = "TX"
            
        self.log_text.append(f'<span style="color: gray;">[{timestamp}]</span> '
                            f'<span style="color: {color}; font-weight: bold;">[{prefix}]</span> '
                            f'<span style="color: white;">{msg}</span>')
        
    def toggle_debug_mode(self, enabled):
        """切换调试模式 - 显示所有串口原始数据"""
        if enabled:
            self.log("🐛 调试模式已开启 - 将显示所有串口数据", is_error=True)
        else:
            self.log("🐛 调试模式已关闭", is_error=True)
    
    def show_help(self):
        """显示使用说明"""
        help_text = """
<h3>SimpleFOC Commander PID调参工具使用说明</h3>

<h4>📌 快速开始</h4>
<ol>
<li><b>连接串口</b>: 选择ESP32的COM口,点击"连接"</li>
<li><b>测试通信</b>: 点击"测试通信"按钮,检查Commander是否正常</li>
<li><b>选择PID</b>: 切换到要调节的PID标签页</li>
<li><b>修改参数</b>: 在输入框输入新值,点击对应的"设置"按钮</li>
</ol>

<h4>⚠ 常见问题</h4>
<ul>
<li><b>发送命令后无任何响应</b>: 
  <ul>
    <li><b>正常现象!</b> SimpleFOC Commander默认<b>静默模式</b>,不返回确认信息</li>
    <li>只要没有返回"err",说明命令已被接受</li>
    <li>通过观察机器人行为来验证参数是否生效</li>
    <li>如需响应,在ESP32代码中添加: <code>command.verbose = VerboseMode::user_friendly;</code></li>
  </ul>
</li>
<li><b>返回"err"错误</b>: 
  <ul>
    <li>确认ESP32已上传wl_pro_robot.cpp程序</li>
    <li>确认代码中有 command.run() 调用</li>
    <li>命令格式错误或Commander ID不匹配</li>
  </ul>
</li>
<li><b>串口无法连接</b>: 
  <ul>
    <li>检查波特率是否为115200</li>
    <li>确认其他程序(如Arduino IDE串口监视器)没有占用串口</li>
    <li>重启ESP32后重新连接</li>
  </ul>
</li>
<li><b>"查询当前参数"功能不可用</b>:
  <ul>
    <li>SimpleFOC Commander在静默模式下不支持查询</li>
    <li>需要ESP32端配置verbose模式才能返回参数值</li>
    <li>建议采用"设置-观察"方式调参</li>
  </ul>
</li>
</ul>

<h4>🔧 ESP32端配置(可选)</h4>
<p>如果想让Commander返回确认信息,在setup()中添加:</p>
<pre>
command.verbose = VerboseMode::user_friendly;  // 友好模式
// 或
command.verbose = VerboseMode::machine_readable; // 机器可读模式
</pre>

<h4>💡 调参技巧</h4>
<ul>
<li><b>从P开始</b>: 先调节P参数,观察响应速度</li>
<li><b>逐步增加</b>: 每次小幅度调整(±0.1),避免震荡</li>
<li><b>测试稳定性</b>: 轻推机器人,观察恢复情况</li>
<li><b>记录参数</b>: 找到好的参数后记录下来</li>
</ul>

<h4>🎯 Commander命令格式</h4>
<pre>
A1.5      → 设置角度PID的P=1.5
AI0.5     → 设置角度PID的I=0.5
AD0.1     → 设置角度PID的D=0.1
AL8       → 设置角度PID的Limit=8
AR100000  → 设置角度PID的Ramp=100000
A?        → 查询角度PID所有参数(可能不稳定)
</pre>
        """
        
        msg_box = QMessageBox(self)
        msg_box.setWindowTitle("使用说明")
        msg_box.setTextFormat(Qt.RichText)
        msg_box.setText(help_text)
        msg_box.setStandardButtons(QMessageBox.Ok)
        msg_box.exec_()
    
    def closeEvent(self, event):
        """关闭窗口时清理资源"""
        if self.serial_thread.running:
            self.serial_thread.stop()
            self.serial_thread.wait()
        event.accept()


def apply_dark_theme(app):
    """应用深色主题"""
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


if __name__ == '__main__':
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    apply_dark_theme(app)
    
    window = PIDTunerUI()
    window.show()
    sys.exit(app.exec_())
