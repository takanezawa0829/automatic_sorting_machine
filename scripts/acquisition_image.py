#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import geometry_msgs.msg
import rosnode
from tf.transformations import quaternion_from_euler
import cv2
import os

# 画像データを取り出す
def save_frame_camera(device_num, dir_path, basename, ext='jpg', delay=1, window_name='frame'):
    cap = cv2.VideoCapture(device_num)

    if not cap.isOpened():
        return

    os.makedirs(dir_path, exist_ok=True)
    base_path = os.path.join(dir_path, basename)

    n = 0
    while True:
        ret, frame = cap.read()
        cv2.imshow(window_name, frame)
        n += 1
        if n == 100:
            cv2.imwrite('{}_{}.{}'.format(base_path, n, ext), frame)
            break
    cv2.destroyWindow(window_name)

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
    
    save_frame_camera(4, 'data/temp', 'camera_capture')
    

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
