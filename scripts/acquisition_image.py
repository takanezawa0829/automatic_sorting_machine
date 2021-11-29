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

# 画像データを取り出す
def save_frame_camera(device_num, basename, ext='jpg', delay=1, window_name='frame'):
    cap = cv2.VideoCapture(device_num)

    if not cap.isOpened():
        return

    os.makedirs('data/temp', exist_ok=True)
    base_path = os.path.join('data/temp', basename)

    n = 0
    while True:
        ret, frame = cap.read()
        cv2.imshow(window_name, frame)
        n += 1
        if n == 100:
            cv2.imwrite('{}_{}.{}'.format(base_path, 'original', ext), frame)
            detection_collar(frame, base_path)
            break
    cv2.destroyWindow(window_name)
    
# カメラを使わず画像を使用する場合はこちらを使用する
def read_image(image, basename):
    img = cv2.imread(os.getcwd() + '/data/temp/' + image)
    base_path = os.path.join('data/temp', basename)
    print(img)
    img_0 = img[0:540, 0:960]
    img_1 = img[0:540, 960:1920]
    img_2 = img[540:1080, 0:960]
    img_3 = img[540:1080, 960:1920]
    cv2.imwrite('{}_{}.{}'.format(base_path, 'original', 'jpg'), img)
    cv2.imwrite('{}_{}.{}'.format(base_path, 'original_0', 'jpg'), img_0)
    cv2.imwrite('{}_{}.{}'.format(base_path, 'original_1', 'jpg'), img_1)
    cv2.imwrite('{}_{}.{}'.format(base_path, 'original_2', 'jpg'), img_2)
    cv2.imwrite('{}_{}.{}'.format(base_path, 'original_3', 'jpg'), img_3)
    
    detection_collar(img_0, base_path, 0)
    detection_collar(img_1, base_path, 1)
    detection_collar(img_2, base_path, 2)
    detection_collar(img_3, base_path, 3)

    
# 画像データから色を判定する
def detection_collar(frame, base_path, file_num):
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

    print("Group names:")
    print(robot.get_group_names())

    print("Current state:")
    print(robot.get_current_state())
    
    # save_frame_camera(4, 'camera_capture')
    read_image('sample_1.jpg', 'camera_capture')

    # SRDFに定義されている"home"の姿勢にする
    arm.set_named_target("home")
    arm.go()

    print("done")


if __name__ == '__main__':

    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass
