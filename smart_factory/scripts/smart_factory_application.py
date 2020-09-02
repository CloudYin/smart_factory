#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import rospy
from geometry_msgs.msg import Pose, Point
from pilz_robot_programming import Lin, Ptp, Robot, from_euler
from pymodbus.client.sync import ModbusTcpClient
from smart_factory.srv import pss_communication, pss_communicationResponse
from take_picture import take_picutre, undistort_pic


# API版本号（不允许修改）
REQUIRED_API_VERSION = "1"    # API version

# 照片存放路径
cap_original_file_path = "/home/pilz/Pictures/smart_factory/cap.png" 
cap_calibrated_file_path = "/home/pilz/Pictures/smart_factory/cap_calibrated.png"

# 速度常量
PTP_SCALE = 0.2        # Ptp移动速度比例
LIN_SCALE = 0.1        # Lin移动速度比例
ACC_SCALE = 0.1        # 加速度比例

# 初始化变量
robot_sto_last = True
box_request_in_process = False
box_request_finished = False
pen_request_in_process = False
pen_request_finished = False
box_handout_in_process = False
box_handout_finished = False
pen_handout_in_process = False
pen_handout_finished = False
robot_at_home = False
robot_moving = False
robot_stopped = False
box_missing = False
pen_missing = False
use_gripper = False
use_sucker = False
gripper_open = False
gripper_close = False
sucker_on = False


class PssCommunicationNode(object):
    def __init__(self):
        ModbusTcpClient('192.168.1.2', port=502)
        rospy.Service('192.168.1.2',
                      pss_communication,
                      self.handle_pss_communication)

    def handle_pss_communication(self, req):
        global robot_sto_last
        global box_request_in_process
        global box_request_finished
        global pen_request_in_process
        global pen_request_finished
        global box_handout_in_process
        global box_handout_finished
        global pen_handout_in_process
        global pen_handout_finished
        global robot_at_home
        global robot_moving
        global robot_stopped
        global box_missing
        global pen_missing
        global use_gripper
        global use_sucker
        global gripper_open
        global gripper_close
        global sucker_on
        robot_sto = req.pss_virtual_outputs[0]
        pss_communication_response = pss_communicationResponse()
        pss_communication_response.box_request_in_process = box_request_in_process
        pss_communication_response.box_request_finished = box_request_finished
        pss_communication_response.pen_request_in_process = pen_request_in_process
        return pss_communication_response


# 主程序
def start_program():
    global box_request_in_process
    global box_request_finished
    global pen_request_in_process
    global pen_request_finished
    global box_handout_in_process
    global box_handout_finished
    global pen_handout_in_process
    global pen_handout_finished
    global robot_at_home
    global robot_moving
    global robot_stopped
    global box_missing
    global pen_missing
    global use_gripper
    global use_sucker
    global gripper_open
    global gripper_close
    global sucker_on

    pss4000 = PssCommunicationNode()

    rospy.loginfo("Program started")  # log

    # important positions
    home_pose = [-0.785, 0, 1.57, 0, 1.57, 0]   # start joint values

    # move to start point with joint values to avoid random trajectory
    r.move(Ptp(goal=home_pose, vel_scale=PTP_SCALE))

    rospy.loginfo("Start loop")  # log
    while not rospy.is_shutdown():
        box_request_in_process = True


if __name__ == "__main__":
    # init a rosnode
    rospy.init_node('robot_program_node')

    # initialisation
    r = Robot(__REQUIRED_API_VERSION__)  # instance of the robot

    # start the main program
    start_program()
    rospy.spin()
