#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import rospy
from geometry_msgs.msg import Pose, Point
from pilz_robot_programming import Lin, Ptp, Robot, from_euler
from pymodbus.client.sync import ModbusTcpClient
from smart_factory.srv import pss_communication, pss_communicationResponse
from take_picture import take_picutre, undistort_pic
from get_pen_pose import get_pen_pose
from get_box_pose import get_box_pose


# API版本号（不允许修改）
REQUIRED_API_VERSION = "1"    # API version

# 位置常量（因涉及到实际机械位置，因此不要修改）
START_POSE = [-0.785, 0, 1.57, 0, 1.57, 0]    # joint values
GRIPPER_STOCK_ORIENTATION = from_euler(0, math.radians(180),  math.radians(45))
GRIPPER_PLATE_ORIENTATION = from_euler(0, math.radians(180),  math.radians(135))
BOX_STOCK_ORIENTATION = from_euler(0, math.radians(180),  math.radians(-45))
BOX_PLATE_ORIENTATION = from_euler(0, math.radians(180),  math.radians(45))
STOCK_Z_UP = 0.15
STOCK_PEN_Y = -0.07
STOCK_PEN_Z_DOWN = 0.083
STOCK_BOX_Y = 0.035
STOCK_BOX_Z_DOWN = 0.042

# 照片存放路径
cap_original_file_path = "/home/pilz/Pictures/smart_factory/cap.png" 
cap_calibrated_file_path = "/home/pilz/Pictures/smart_factory/cap_calibrated.png"

# 速度常量
PTP_SCALE = 0.1        # Ptp移动速度比例
LIN_SCALE = 0.05        # Lin移动速度比例
ACC_SCALE = 0.1        # 加速度比例

# 初始化变量
robot_sto_last = True
robot_sto = False
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
        global robot_sto
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

    def box_request_in_process(self, state):
        self.client.write_coil(50, state)

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
    global robot_sto

    # pss4000 = PssCommunicationNode()

    rospy.loginfo("Program started")  # log

    # move to start point with joint values to avoid random trajectory
    r.move(Ptp(goal=START_POSE, vel_scale=PTP_SCALE))

    take_picutre(cap_original_file_path)
    undistort_pic(cap_original_file_path, cap_calibrated_file_path)
    rospy.sleep(0.5)

    # pen_pick_X_list = get_pen_pose(cap_calibrated_file_path)
    box_pick_X_list = get_box_pose(cap_calibrated_file_path)

    # for i in range(len(pen_pick_X_list)):
    #     pen_stock_pose_up = Pose(position=Point(pen_pick_X_list[i], STOCK_PEN_Y, STOCK_Z_UP), orientation=GRIPPER_STOCK_ORIENTATION)
    #     pen_stock_pose_down = Pose(position=Point(pen_pick_X_list[i], STOCK_PEN_Y, STOCK_PEN_Z_DOWN), orientation=GRIPPER_STOCK_ORIENTATION)
    #     r.move(Lin(goal=pen_stock_pose_up, reference_frame="prbt_base_link", vel_scale=LIN_SCALE, relative=False))
    #     r.move(Lin(goal=pen_stock_pose_down, reference_frame="prbt_base_link", vel_scale=LIN_SCALE, relative=False))
    #     r.move(Lin(goal=pen_stock_pose_up, reference_frame="prbt_base_link", vel_scale=LIN_SCALE, relative=False))
    
    for i in range(len(box_pick_X_list)):
        box_stock_pose_up = Pose(position=Point(box_pick_X_list[i], STOCK_BOX_Y, STOCK_Z_UP), orientation=BOX_STOCK_ORIENTATION)
        box_stock_pose_down = Pose(position=Point(box_pick_X_list[i], STOCK_BOX_Y, STOCK_BOX_Z_DOWN), orientation=BOX_STOCK_ORIENTATION)
        r.move(Lin(goal=box_stock_pose_up, reference_frame="prbt_base_link", vel_scale=LIN_SCALE, relative=False))
        r.move(Lin(goal=box_stock_pose_down, reference_frame="prbt_base_link", vel_scale=LIN_SCALE, relative=False))
        r.move(Lin(goal=box_stock_pose_up, reference_frame="prbt_base_link", vel_scale=LIN_SCALE, relative=False))


if __name__ == "__main__":
    # init a rosnode
    rospy.init_node('robot_program_node')

    # initialisation
    r = Robot(REQUIRED_API_VERSION)  # instance of the robot

    # start the main program
    start_program()
    rospy.spin()
