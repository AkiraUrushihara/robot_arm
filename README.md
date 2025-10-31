# ロボットアーム開発

## 概要
オープンソースのロボットアームを用いて、個人でロボットの知能の研究をするプロジェクト

2025/10/30ではPS4コントローラーからのテレオペレーションが出来た

2026/6までは学ロボで忙しいため進捗は遅い

## 使い方(テレオペ)
1. PS4コントローラーの接続:
```bash
ros2 run joy joy_node
```
2. 実機との接続(ロボットアームが初期位置に移動するので注意)
```bash
cd ~/robot_arm
source install/setup.bash
ros2 run so101_hw_interface so101_motor_bridge
```
3. rvizによる可視化
```bash
cd ~/robot_arm
source install/setup.bash
ros2 launch so101_follower_description display.launch.py     use_gui:=false     joint_states_topic:=/so101_follower/joint_states
```
4. コントロールノードの起動
```bash
cd ~/robot_arm/Lerobot_ros2/src/so101_hw_interface/controll/
python3 controller.py
```
