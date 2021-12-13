# automatic_sorting_machine
こちらはCRANE-X7でゴミと貴重品を分別するためのROSパッケージです。

# 実装内容
crane_x7を用いてゴミと貴重品を分別するサンプルコードです。
crane_x7とIntelRealSenseを用いて動作を行っています。

# 動作環境
以下の環境で動作確認しています。

ROS Galactic Geochelone
　- OS: Ubuntu 20.04.5 LTS
　- ROS Distribution: Galactic Geochelone
　- Rviz 1.14.10
　- Gazebo 11.9.0
crane_x7
IntelRealSense

# 環境構築
1. 2021年度ロボットシステム学での講義資料を参考にROSをUbuntu 20.04 LTSにインストールします。
2. 本パッケージをインストールします。
'''
cd ~/catkin_ws/src  
git clone git@github.com:takanezawa0829/automatic_sorting_machine.git
cd ~/catkin_ws  
catkin_make  
'''

# 実行方法
## シミュレータを使う場合
1. RealSenseをPCに接続した状態でUbuntuを起動してください。
2. シュミレータを起動します。
'''
roslaunch crane_x7_gazebo crane_x7_with_table.launch
'''
3. 本パッケージのmain.pyを実行します。
'''
rosrun automatic_sorting_machine main.py
'''

## 実機を使う場合
1. crane_x7とガチャガチャ、う◯◯を以下の画像のように配置してください。


