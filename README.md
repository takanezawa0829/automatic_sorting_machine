# automatic_sorting_machine


CRANE-X7のROSパッケージです。このパッケージはhttps://github.com/rt-net/crane_x7_ros をオリジナルとして、千葉工業大学未来ロボティクス学科で開講された講義内でのグループ３班が変更を加えたものです。

製品ページはこちらです。


https://www.rt-net.jp/products/crane-x7


ROSWikiはこちらです。


https://wiki.ros.org/crane_x7


ROSのサンプルコード集はこちらです。


https://github.com/rt-net/crane_x7_ros/tree/master/crane_x7_examples


# 実装内容


crane-x7を用いてうんこと貴重品を分別するコードです。


この動作では、crane-x7とIntelRealSenseを用いて実機動作を行っています。


# 動作環境
以下の環境で動作確認しています。

ROS Galactic Geochelone
* OS: Ubuntu 20.04.5 LTS
* ROS Distribution: Galactic Geochelone
* Rviz 1.14.10
* Gazebo 11.9.0
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
1. RealSense、crane_x7をPCに接続した状態でUbuntuを起動してください。
2. MoveIt!、rosを起動します。
```
roslaunch crane_x7_gazebo demo.launch fake-execution:=false
```
3. 本パッケージのmain.pyを実行します。
'''
rosrun automatic_sorting_machine main.py
'''