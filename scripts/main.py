#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import geometry_msgs.msg
import rosnode
from tf.transformations import quaternion_from_euler
import cv2
import os
import numpy as np
import time

# 色判定したボールがゴミか貴重品か（1: 貴重品、0: ゴミ）
result = [0, 0, 0, 0]

# acquisition_image.pyの内容

# 画像データを取り出す
def save_frame_camera(device_num, basename, ext='jpg', delay=1, window_name='frame'):
    cap = cv2.VideoCapture(device_num)

    if not cap.isOpened():
        return

    os.makedirs('data/temp', exist_ok=True)
    base_path = os.path.join('data/temp', basename)
        
    while True:
        ret, frame = cap.read()
        cv2.imshow(window_name, frame)
        key = cv2.waitKey(delay) & 0xFF

        time.sleep(10)

        img_0 = frame[0:768, 0:1152]
        img_1 = frame[0:768, 1152:2304]
        img_2 = frame[768:1536, 0:1152]
        img_3 = frame[768:1536, 1152:2304]
        cv2.imwrite('{}_{}.{}'.format(base_path, 'original', 'jpg'), frame)
        cv2.imwrite('{}_{}.{}'.format(base_path, 'original_0', 'jpg'), img_0)
        cv2.imwrite('{}_{}.{}'.format(base_path, 'original_1', 'jpg'), img_1)
        cv2.imwrite('{}_{}.{}'.format(base_path, 'original_2', 'jpg'), img_2)
        cv2.imwrite('{}_{}.{}'.format(base_path, 'original_3', 'jpg'), img_3)
        
        detection_collar_red(img_0, base_path, 0)
        detection_collar_red(img_1, base_path, 1)
        detection_collar_red(img_2, base_path, 2)
        detection_collar_red(img_3, base_path, 3)

        break
        
    cv2.destroyWindow(window_name)
    
# カメラを使わず画像を使用する場合はこちらを使用する
def read_image(image, basename):
    img = cv2.imread(os.getcwd() + '/data/temp/' + image)
    base_path = os.path.join('data/temp', basename)
    img_0 = img[0:540, 0:960]
    img_1 = img[0:540, 960:1920]
    img_2 = img[540:1080, 0:960]
    img_3 = img[540:1080, 960:1920]
    cv2.imwrite('{}_{}.{}'.format(base_path, 'original', 'jpg'), img)
    cv2.imwrite('{}_{}.{}'.format(base_path, 'original_0', 'jpg'), img_0)
    cv2.imwrite('{}_{}.{}'.format(base_path, 'original_1', 'jpg'), img_1)
    cv2.imwrite('{}_{}.{}'.format(base_path, 'original_2', 'jpg'), img_2)
    cv2.imwrite('{}_{}.{}'.format(base_path, 'original_3', 'jpg'), img_3)
    
    detection_collar_red(img_0, base_path, 0)
    detection_collar_red(img_1, base_path, 1)
    detection_collar_red(img_2, base_path, 2)
    detection_collar_red(img_3, base_path, 3)

    
# 画像データから色を判定する
def detection_collar_red(frame, base_path, file_num):
    
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # 色の範囲を指定する
    lower_color = np.array([0, 64, 0])
    upper_color = np.array([5, 255, 255])
    
    # 指定した色に基づいたマスク画像の生成
    mask = cv2.inRange(hsv, lower_color, upper_color)
    masked_image = cv2.bitwise_and(hsv, hsv, mask = mask)
        
    cv2.imwrite('{}_{}_{}.{}'.format(base_path, 'mask', file_num, 'jpg'), masked_image)
    
    # ラベリング結果書き出し用に画像を準備
    out_image = masked_image
    
    num_labels, label_image, stats, center = cv2.connectedComponentsWithStats(mask)

    # 最大のラベルは画面全体を覆う黒なので不要．データを削除
    num_labels = num_labels - 1
    stats = np.delete(stats, 0, 0)
    center = np.delete(center, 0, 0)

    
    # 検出したラベルの数だけ繰り返す
    for index in range(num_labels):
        if stats[index][4] > 40000:
            # resultに判定結果を代入
            result[file_num] = 1
    
            # ラベルのx,y,w,h,面積s,重心位置mx,myを取り出す
            x = stats[index][0]
            y = stats[index][1]
            w = stats[index][2]
            h = stats[index][3]
            s = stats[index][4]
            mx = int(center[index][0])
            my = int(center[index][1])
            #print("(x,y)=%d,%d (w,h)=%d,%d s=%d (mx,my)=%d,%d"%(x, y, w, h, s, mx, my) )

            # ラベルを囲うバウンディングボックスを描画
            cv2.rectangle(out_image, (x, y), (x+w, y+h), (255, 0, 255))

            # 重心位置の座標と面積を表示
            cv2.putText(out_image, "%d,%d"%(mx,my), (x-15, y+h+15), cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 0))
            cv2.putText(out_image, "%d"%(s), (x, y+h+30), cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 0))
    
    cv2.imwrite('{}_{}_{}.{}'.format(base_path, 'output', file_num, 'jpg'), out_image)

    
    return

# acquisition_image.pyの内容 end

# 準備体制に移動する。
def pre_position():
    rospy.init_node("crane_x7_pick_and_place_controller")
    robot = moveit_commander.RobotCommander()
    arm = moveit_commander.MoveGroupCommander("arm")
    arm.set_max_velocity_scaling_factor(0.1)
    arm.set_max_acceleration_scaling_factor(1.0)
    gripper = moveit_commander.MoveGroupCommander("gripper")
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = 0.15
    target_pose.position.y = 0.14
    target_pose.position.z = 0.32
    q = quaternion_from_euler(-3.14, 0.0, -6.28/2.0)
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3]
    arm.set_pose_target(target_pose)
    arm.go()  # 実行する。

# ボールを掴んでかごに入れる。（box_num: ゴミ->0, 貴重品->1）
def move(ball_num, box_num):
    rospy.init_node("crane_x7_pick_and_place_controller")
    robot = moveit_commander.RobotCommander()
    arm = moveit_commander.MoveGroupCommander("arm")
    arm.set_max_velocity_scaling_factor(0.1)
    arm.set_max_acceleration_scaling_factor(1.0)
    gripper = moveit_commander.MoveGroupCommander("gripper")
    
    # ボールの座標
    ball_position = {'x': 0, 'y': 0, 'z': 0}
    box_position = {'x': 0, 'y': 0}
    if ball_num == 0:
        ball_position['x'] = 0.07
        ball_position['y'] = 0.26
        ball_position['z'] = 0.12
    elif ball_num == 1:
        ball_position['x'] = 0.23
        ball_position['y'] = 0.26
        ball_position['z'] = 0.12
    elif ball_num == 2:
        ball_position['x'] = 0.07
        ball_position['y'] = 0.14
        ball_position['z'] = 0.12
    else:
        ball_position['x'] = 0.23
        ball_position['y'] = 0.14
        ball_position['z'] = 0.12
        
    # 箱の座標
    if box_num == 0:
        box_position['x'] = 0.2
        box_position['y'] = -0.19
    elif box_num == 1:
        box_position['x'] = 0.04
        box_position['y'] = -0.2
    
    # ハンドを開く
    gripper.set_joint_value_target([0.7, 0.7])
    gripper.go()

    # 掴みに行く　各ボールの座標を入れる ボール編 右下(0.07,0.14,0.12),右上(0.23,0.14,0.12),左上(0.23,0.26,0.12),左下(0.07,0.26,0.12)
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = ball_position['x']
    target_pose.position.y = ball_position['y']
    target_pose.position.z = ball_position['z']
    q = quaternion_from_euler(-3.14, 0.0, -3.14/2.0)  # 上方から掴みに行く場合
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3]
    arm.set_pose_target(target_pose)  # 目標ポーズ設定
    arm.go()  # 実行

    # ハンドを閉じる
    gripper.set_joint_value_target([0.5, 0.5])
    gripper.go()

    # 持ち上げる 各ボールの座標を入れる
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = ball_position['x']
    target_pose.position.y = ball_position['y']
    target_pose.position.z = ball_position['z']
    q = quaternion_from_euler(-3.14, 0.0, -3.14/2.0)  # 上方から掴みに行く場合
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3]
    arm.set_pose_target(target_pose)  # 目標ポーズ設定
    arm.go()							# 実行

    # 移動する1
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = 0.3
    target_pose.position.y = 0
    target_pose.position.z = 0.3
    q = quaternion_from_euler(-3.14, 0.0, -3.14/2.0)  # 上方から掴みに行く場合
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3]
    arm.set_pose_target(target_pose)  # 目標ポーズ設定
    arm.go()  # 実行

    # 移動する2 箱の座標を入れる 上の箱(0.2,-0.19,1.0),下の箱(0.04,-0.2,1.0) z座標は0.3
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = box_position['x']
    target_pose.position.y = box_position['y']
    target_pose.position.z = 0.3
    q = quaternion_from_euler(-3.14, 0.0, -3.14/2.0)  # 上方から掴みに行く場合
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3]
    arm.set_pose_target(target_pose)  # 目標ポーズ設定
    arm.go()  # 実行

    # 下ろす　箱の座標入れる　z座標は0.2
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = box_position['x']
    target_pose.position.y = box_position['y']
    target_pose.position.z = 0.2
    q = quaternion_from_euler(-3.14, 0.0, -3.14/2.0)  # 上方から掴みに行く場合
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3]
    arm.set_pose_target(target_pose)  # 目標ポーズ設定
    arm.go()  # 実行

    # ハンドを開く
    gripper.set_joint_value_target([0.9, 0.9])
    gripper.go()

    # 少しだけハンドを持ち上げる　箱の座標入れる z座標は0.3
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = box_position['x']
    target_pose.position.y = box_position['y']
    target_pose.position.z = 0.3
    q = quaternion_from_euler(-3.14, 0.0, -3.14/2.0)  # 上方から掴みに行く場合
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3]
    arm.set_pose_target(target_pose)  # 目標ポーズ設定
    arm.go()  # 実行

def main():
    rospy.init_node("crane_x7_pick_and_place_controller")
    robot = moveit_commander.RobotCommander()
    arm = moveit_commander.MoveGroupCommander("arm")
    arm.set_max_velocity_scaling_factor(0.1)
    arm.set_max_acceleration_scaling_factor(1.0)
    gripper = moveit_commander.MoveGroupCommander("gripper")

    while len([s for s in rosnode.get_node_names() if 'rviz' in s]) == 0:
        rospy.sleep(1.0)
    rospy.sleep(1.0)

    # アーム初期ポーズを表示
    arm_initial_pose = arm.get_current_pose().pose
    print("Arm initial pose:")
    print(arm_initial_pose)

    # 何かを掴んでいた時のためにハンドを開く
    gripper.set_joint_value_target([0.9, 0.9])
    gripper.go()

    # SRDFに定義されている"home"の姿勢にする
    arm.set_named_target("home")
    arm.go()
    gripper.set_joint_value_target([0.7, 0.7])
    gripper.go()
    
    # bring_arm_above_ball.pyの内容
    pre_position()
    
    # acquisition_image.pyの内容
    save_frame_camera(0, 'camera_capture')
    # read_image('sample_1.jpg', 'camera_capture')
    
    
    # 判定結果を出力する。
    print("判定結果: ")
    print("左上: " + str(result[0]))
    print("右上: " + str(result[1]))
    print("左下: " + str(result[2]))
    print("右下: " + str(result[3]))
    
    # 判定結果をもとにボールを移動させる。
    for index, result_one in enumerate(result):
        move(index, result_one)
        pre_position()
            
    # SRDFに定義されている"home"の姿勢にする
    arm.set_named_target("home")
    arm.go()
    gripper.set_joint_value_target([0.7, 0.7])
    gripper.go()
    
    
    
    return

if __name__ == '__main__':

    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass
