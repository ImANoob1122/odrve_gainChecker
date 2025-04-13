#!/usr/bin/env python3
"""
ROS2 Humble + odrive (v0.5.6)  
・全接続 odrive の各モーターに対してキャリブレーション実施  
・プログラム開始時に vel_gain, vel_integrator_gain, vel_integrator_limit を設定  
・joy (sensor_msgs/Joy) を購読し、R1 ボタン（ここでは buttons[5]）で正方向、L1 ボタン（buttons[4]）で負方向へ
  ramped_velocity mode で回転させる。ただし、どちらのボタンも押されていない場合は速度が 0 になる。  
・さらに、Qt（PyQt5）と matplotlib により，時間軸グラフ上に指令速度と実際速度を表示し、  
  一時停止／再開ボタンと表示するモーターを選択するプルダウンメニューを提供する。
"""

import rclpy
from rclpy.node import Node
import threading
import json
import time
import sys

# odrive API
import odrive
import odrive.utils

# ROS2 標準メッセージ（joy, Bool）とユーザー定義メッセージ
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy
from rogidrive_msg.msg import RogidriveMessage, RogidriveMultiArray
from rcl_interfaces.msg import SetParametersResult


# 定数（odrive の状態／モード）
AXIS_STATE_CLOSED_LOOP_CONTROL = 8
AXIS_STATE_IDLE = 1
MODE_VEL_RAMP = 2     # ramped velocity mode として利用
VEL_RAMP_RATE = 5     # [単位/s]：ボタン押下時の速度変化レート
FULL_CALIBRATION_SEQUENCE = 3

# -------------------------------
# ROS2 ノード：odrive の接続・キャリブレーション・制御を行う部分
# -------------------------------
class OdriveControllerNode(Node):
    def __init__(self):
        super().__init__('odrive_controller')

        # パラメータ宣言（configファイルパス、各パラメータの設定値）
        self.declare_parameter('config_path', 'config.json')
        self.declare_parameter('vel_gain', 0.16)
        self.declare_parameter('vel_integrator_gain', 0.32)
        self.declare_parameter('vel_integrator_limit', 100.0)

        self.config_path = self.get_parameter('config_path').get_parameter_value().string_value
        self.vel_gain = self.get_parameter('vel_gain').get_parameter_value().double_value
        self.vel_integrator_gain = self.get_parameter('vel_integrator_gain').get_parameter_value().double_value
        self.vel_integrator_limit = self.get_parameter('vel_integrator_limit').get_parameter_value().double_value

        # JSONファイルから設定読み込み
        with open(self.config_path, 'r') as f:
            # 例：{"motors": { "motor1": { "serial": "xxxx", "axis": 0 }, ... }}
            self.config = json.load(f)['motors']

        # 各モーターごとのグラフデータを保持する辞書を初期化
        self.data = {}  # key: motor名, value: dict(time, commanded, actual)
        for motor_name in self.config:
            self.data[motor_name] = {'time': [], 'commanded': [], 'actual': []}
        self.data_lock = threading.Lock()

        self.commanded_velocity = 0.0   # 現在の指令速度
        self.last_time = time.time()      # タイムスタンプ
        self.paused = False             # 「停止／再開」切替用フラグ
        self.last_joy = None            # 最新の joy メッセージ

        # joy の購読（トピック名は "joy" とする）
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        # enable 用の Bool メッセージ購読
        self.enable_sub = self.create_subscription(Bool, 'odrive_enable', self.odrive_enable_callback, 10)
        # ステータス送信用パブリッシャ（例：RogidriveMultiArray）
        self.status_pub = self.create_publisher(RogidriveMultiArray, 'odrive_status', 10)

        # 以下、odrive との接続・初期化
        self.connect_odrive()
        self.calibration_check()
        self.apply_velocity_parameters()
        self.set_closed_loop()

        # 100ms周期のタイマーで定期更新
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.add_on_set_parameters_callback(self.parameter_update_callback)

    def odrive_enable_callback(self, msg: Bool):
        if msg.data:
            if self.are_all_calibrated():
                self.set_closed_loop()
                self.get_logger().info("All motors set to closed loop")
            else:
                self.get_logger().error("Some motors are not calibrated!")

    def connect_odrive(self):
        self.get_logger().info("Connecting to odrives ...")
        serial_set = set()
        for motor_name, motor in self.config.items():
            serial_set.add(motor['serial'])
        self.devs = []
        for serial in serial_set:
            self.get_logger().info(f"Connecting to odrive with serial: {serial}")
            dev = odrive.find_any(serial_number=serial)
            self.devs.append(dev)
            # 各モーターの設定辞書に dev オブジェクトを登録（axis 0 または 1）
            for motor_name, motor in self.config.items():
                if motor['serial'] == serial:
                    if motor['axis'] == 0:
                        motor['dev'] = dev.axis0
                    elif motor['axis'] == 1:
                        motor['dev'] = dev.axis1
        self.get_logger().info("All odrives connected.")

    def calibration_check(self):
        self.get_logger().info("Starting calibration for all motors...")
        for dev in self.devs:
            odrive.utils.dump_errors(dev)
        for motor_name, motor in self.config.items():
            motor['dev'].requested_state = FULL_CALIBRATION_SEQUENCE
        # キャリブレーション完了まで待機（必要に応じて調整）
        time.sleep(18)
        if self.are_all_calibrated():
            self.get_logger().info("All motors calibrated successfully.")
        else:
            self.get_logger().error("Calibration failed for some motors!")
            for dev in self.devs:
                odrive.utils.dump_errors(dev)

    def are_all_calibrated(self):
        count = 0
        for motor_name, motor in self.config.items():
            if motor['dev'].encoder.is_ready:
                count += 1
        return count == len(self.config)

    def apply_velocity_parameters(self):
        self.get_logger().info("Applying velocity parameters to all motors...")
        for motor_name, motor in self.config.items():
            motor['dev'].controller.config.vel_gain = self.vel_gain
            motor['dev'].controller.config.vel_integrator_gain = self.vel_integrator_gain
            motor['dev'].controller.config.vel_integrator_limit = self.vel_integrator_limit
            self.get_logger().info(f"Applied parameters to motor {motor_name}")

    def set_closed_loop(self):
        self.get_logger().info("Setting closed loop (ramped velocity mode) for all motors...")
        for motor_name, motor in self.config.items():
            motor['dev'].encoder.set_linear_count(0)
            time.sleep(0.1)
            motor['dev'].requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            motor['dev'].controller.config.input_mode = MODE_VEL_RAMP
            motor['dev'].controller.input_vel = 0
            self.get_logger().info(f"Motor {motor_name} set to closed loop.")
    
    def joy_callback(self, msg: Joy):
        # 最新の joy メッセージを保存
        self.last_joy = msg

    def timer_callback(self):
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        # pause 状態でなければ joy の状態に応じて指令速度を更新
        if not self.paused and self.last_joy is not None:
            if len(self.last_joy.buttons) >= 6:
                # R1 (buttons[5]) と L1 (buttons[4]) のどちらかのみ押されている場合に ramp をかける
                if self.last_joy.buttons[5] and not self.last_joy.buttons[4]:
                    if self.commanded_velocity == 0.0:
                        for motor_name, motor in self.config.items():
                            motor['dev'].controller.config.vel_integrator_limit = self.vel_integrator_limit
                    self.commanded_velocity += VEL_RAMP_RATE * dt
                elif self.last_joy.buttons[4] and not self.last_joy.buttons[5]:
                    if self.commanded_velocity == 0.0:
                        for motor_name, motor in self.config.items():
                            motor['dev'].controller.config.vel_integrator_limit = self.vel_integrator_limit
                    self.commanded_velocity -= VEL_RAMP_RATE * dt
                if self.last_joy.buttons[7]:
                    for motor_name, motor in self.config.items():
                        motor['dev'].controller.config.vel_integrator_limit = 0.0
                    self.commanded_velocity = 0.0

        # 各モーターへ指令速度を反映
        for motor_name, motor in self.config.items():
            motor['dev'].controller.input_vel = self.commanded_velocity

        # 実際の速度（encoder.vel_estimate）を各モーターから読み、グラフ用データに追加
        with self.data_lock:
            for motor_name, motor in self.config.items():
                actual_vel = motor['dev'].encoder.vel_estimate
                self.data[motor_name]['time'].append(current_time)
                self.data[motor_name]['commanded'].append(self.commanded_velocity)
                self.data[motor_name]['actual'].append(actual_vel)
                # データが長くなりすぎないように最新の 1000 サンプルのみ保持
                if len(self.data[motor_name]['time']) > 1000:
                    self.data[motor_name]['time'] = self.data[motor_name]['time'][-1000:]
                    self.data[motor_name]['commanded'] = self.data[motor_name]['commanded'][-1000:]
                    self.data[motor_name]['actual'] = self.data[motor_name]['actual'][-1000:]

        # 必要に応じて、RogidriveMultiArray などでステータスを publish 可能
        # status_msg = RogidriveMultiArray()
        # ... フィールドを設定 ...
        # self.status_pub.publish(status_msg)
    
    def parameter_update_callback(self, params):
        for param in params:
            if param.name == 'vel_gain' and param.type_ == param.Type.DOUBLE:
                self.vel_gain = param.value
                self.get_logger().info(f'vel_gain updated to: {self.vel_gain}')
            elif param.name == 'vel_integrator_gain' and param.type_ == param.Type.DOUBLE:
                self.vel_integrator_gain = param.value
                self.get_logger().info(f'vel_integrator_gain updated to: {self.vel_integrator_gain}')
            elif param.name == 'vel_integrator_limit' and param.type_ == param.Type.DOUBLE:
                self.vel_integrator_limit = param.value
                self.get_logger().info(f'vel_integrator_limit updated to: {self.vel_integrator_limit}')

        self.apply_velocity_parameters()
        # 変更を受け入れる場合は、成功のレスポンスを返す
        return SetParametersResult(successful=True)

    def toggle_pause(self):
        self.paused = not self.paused
        self.get_logger().info(f"Paused: {self.paused}")


# -------------------------------
# Qt（PyQt5）＋ matplotlib によるグラフ表示用 GUI クラス
# -------------------------------
import matplotlib
matplotlib.use('Qt5Agg')
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from PyQt5 import QtWidgets, QtCore

class GraphWindow(QtWidgets.QMainWindow):
    def __init__(self, ros_node: OdriveControllerNode):
        super().__init__()
        self.ros_node = ros_node
        self.setWindowTitle("Odrive Velocity Graph")

        # メインウィジェットを作成
        central_widget = QtWidgets.QWidget()
        self.setCentralWidget(central_widget)

        # matplotlib の Figure と Canvas を作成
        self.figure = Figure()
        self.canvas = FigureCanvas(self.figure)
        self.ax = self.figure.add_subplot(111)
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Velocity")
        self.line_commanded, = self.ax.plot([], [], label="Commanded Velocity")
        self.line_actual, = self.ax.plot([], [], label="Actual Velocity")
        self.ax.legend()

        # 一時停止／再開用トグルボタン
        self.pause_button = QtWidgets.QPushButton("Pause")
        self.pause_button.setCheckable(True)
        self.pause_button.toggled.connect(self.on_pause_toggled)

        # プルダウンメニュー：どのモーターのグラフを表示するか
        self.motor_combo = QtWidgets.QComboBox()
        self.motor_combo.addItems(list(self.ros_node.config.keys()))
        self.motor_combo.currentIndexChanged.connect(self.on_motor_changed)
        self.selected_motor = self.motor_combo.currentText()

        # レイアウト設定
        layout = QtWidgets.QVBoxLayout(central_widget)
        layout.addWidget(self.canvas)
        controls_layout = QtWidgets.QHBoxLayout()
        controls_layout.addWidget(QtWidgets.QLabel("Select Motor:"))
        controls_layout.addWidget(self.motor_combo)
        controls_layout.addWidget(self.pause_button)
        layout.addLayout(controls_layout)

        # プロット更新用タイマー（100ms ごと）
        self.plot_timer = QtCore.QTimer()
        self.plot_timer.timeout.connect(self.update_plot)
        self.plot_timer.start(100)

    def on_pause_toggled(self, checked):
        if checked:
            self.pause_button.setText("Resume")
        else:
            self.pause_button.setText("Pause")
        self.ros_node.toggle_pause()

    def on_motor_changed(self, index):
        self.selected_motor = self.motor_combo.itemText(index)

    def update_plot(self):
        with self.ros_node.data_lock:
            data = self.ros_node.data.get(self.selected_motor, None)
            if data is None:
                return
            times = data['time']
            commanded = data['commanded']
            actual = data['actual']
        if not times:
            return
        # x軸は相対時間（最初のタイムスタンプを 0 に）
        t0 = times[0]
        times_rel = [t - t0 for t in times]
        self.line_commanded.set_data(times_rel, commanded)
        self.line_actual.set_data(times_rel, actual)
        self.ax.relim()
        self.ax.autoscale_view()
        self.canvas.draw()


# -------------------------------
# main 関数
# -------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = OdriveControllerNode()
    rclpy.spin(node)
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    app = QtWidgets.QApplication(sys.argv)
    window = GraphWindow(node)
    window.show()

    try:
        ret = app.exec_()
    except KeyboardInterrupt:
        pass
    finally:
        window.close()  # closeEvent() 内で shutdown を行う
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(ret)



if __name__ == '__main__':
    main()
