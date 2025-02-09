# Odrive Gain Checker

**Odrive Gain Checker** は、ROS 2 Humble 上で動作する ODrive (ver 0.5.6) 用のノードです。  
本パッケージは、以下の機能を提供します。

- **ODrive への接続・キャリブレーション**  
  接続されているすべての ODrive デバイスを検出し、各モーターに対してキャリブレーションを実施します。

- **パラメータの設定と動的更新**  
  起動時に `vel_gain`、`vel_integrator_gain`、`vel_integrator_limit` の各パラメータを設定し、実行中に `ros2 param set` コマンドで動的に更新可能です。  
  更新は内部のパラメータ更新コールバックにより、リアルタイムに反映されます。

- **ジョイスティックによるモーター制御**  
  - R1 ボタン (buttons[5]) を押すと、指令速度が正方向に増加します。  
  - L1 ボタン (buttons[4]) を押すと、指令速度が負方向に増加します。  
  - どちらのボタンも押されていない、または両方押されている場合は、指令速度が 0 にリセットされます。

- **GUI によるリアルタイム可視化**  
  PyQt5 と matplotlib を用いた GUI を提供し、時間軸上に「指令速度」と「実際の速度」を表示します。  
  GUI 上には以下の操作が可能です:
  - **一時停止/再開**: 更新とジョイスティック入力の受付を停止・再開します。
  - **モーター選択**: プルダウンメニューからグラフ表示するモーターを指定できます。


## 依存関係

- **ROS 2 Humble**
- **Python 3.10** 以上
- **rclpy**
- **ODrive Python API (ver 0.5.6)**
- **PyQt5**
- **matplotlib**

その他、標準 Python モジュール（`json`、`time`、`threading` など）を利用しています。

## インストール

1. **ソースのクローン**  
   ROS 2 ワークスペースの `src` ディレクトリに本パッケージをクローンします:

   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/ImANoob1122/odrve_gainChecker.git
   ```

## 使い方

1. **実行**  

    ```bash
    ros2 run odrive_gainChecker odrive_controller --ros-args -p config_path:=<コンフィグのpath>
    ```
2. **パラメーターの変更**
    vel_gain, vel_integrator_gain, vel_integrator_limitが変更できます
    ```bash
    ros2 param set /odrive_controller <変数> <値(double)>
    ```