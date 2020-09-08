#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import rospy
from geometry_msgs.msg import Pose, Point
from pilz_robot_programming import Lin, Ptp, Robot, from_euler, Sequence
from pymodbus.client.sync import ModbusTcpClient
from smart_factory.srv import pss_communication, pss_communicationResponse
from take_picture import take_picutre, undistort_pic
from get_pen_pose import get_pen_pose
from get_box_pose import get_box_pose


# API版本号（不允许修改）
REQUIRED_API_VERSION = "1"    # API版本号

# 位置常量（因涉及到实际机械位置，因此不要修改）
START_POSE = [math.radians(-30), 0, math.radians(123), 0, math.radians(57), math.radians(17)]    # 起始关节角度
GRIPPER_STOCK_ORIENTATION = from_euler(0, math.radians(180),  math.radians(47))
GRIPPER_PLATE_PLACE_ORIENTATION = from_euler(0, math.radians(180),  math.radians(137))
GRIPPER_PLATE_PICK_ORIENTATION = from_euler(0, math.radians(180),  math.radians(-43))
SUCKER_STOCK_ORIENTATION = from_euler(0, math.radians(180),  math.radians(-45))
SUCKER_PLATE_ORIENTATION = from_euler(0, math.radians(180),  math.radians(45))
SAFETY_HEIGHT = 0.1
STOCK_Z_UP = 0.15
STOCK_PEN_Y = -0.07
STOCK_PEN_Z_DOWN = 0.083
PLATE_PEN_Z_DOWN = 0.081
STOCK_BOX_Y = 0.0355
STOCK_BOX_Z_DOWN = 0.0495
PLATE_BOX_Z_DOWN = 0.062



# 照片存放路径
cap_original_file_path = "/home/pilz/Pictures/smart_factory/cap.png" 
cap_calibrated_file_path = "/home/pilz/Pictures/smart_factory/cap_calibrated.png"

# 速度常量
PTP_SCALE = 0.5        # Ptp移动速度比例
LIN_SCALE = 0.4        # Lin移动速度比例
PnP_SCALE = 0.1       # Pick & Place

# 初始化变量
# 接收自pss信号
robot_sto_last = True
robot_stop_last = True
robot_start_last = True
back_home_last = True
robot_sto = False
box_request = False
pen_request = False
box_handout = False
pen_handout = False
back_home = False

# 发送至pss信号
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

pen_pick_X_list = []
box_pick_X_list = []


class PssCommunicationNode(object):
    def __init__(self):
        ModbusTcpClient('192.168.1.2', port=502)
        rospy.Service('pss_communication',
                      pss_communication,
                      self.handle_pss_communication)

    def handle_pss_communication(self, req):
        # 接收自pss信号
        global robot_sto_last
        global robot_stop_last
        global robot_start_last
        global back_home_last
        global robot_sto
        global box_request
        global pen_request
        global box_handout
        global pen_handout
        global back_home

        # 发送至pss信号
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
        
        # 接收自pss信号
        robot_sto = req.pss_virtual_outputs[0]
        box_request = req.pss_virtual_outputs[1]
        pen_request = req.pss_virtual_outputs[2]
        box_handout = req.pss_virtual_outputs[3]
        pen_handout = req.pss_virtual_outputs[4]
        robot_start = req.pss_virtual_outputs[5]
        robot_stop = req.pss_virtual_outputs[6]
        robot_reset = req.pss_virtual_outputs[7]
        back_home = req.pss_virtual_outputs[8]
        
        # 逻辑处理
        if (not robot_sto and robot_sto_last) or (not robot_stop_last and robot_stop) or (not back_home_last and back_home):
            r.pause()
            rospy.loginfo("Robot paused")
        
        if (not robot_start_last and robot_start):
            rospy.sleep(1)
            r.resume()
        
        if (not back_home_last and back_home):
            rospy.sleep(1)
            r.resume()
            r.move(Ptp(goal=START_POSE, vel_scale=PTP_SCALE))
            robot_at_home = True
        
        if robot_reset:
            box_missing = False
            pen_missing = False
        
        robot_sto_last = robot_sto
        robot_stop_last = robot_stop
        robot_start_last = robot_start
        back_home_last = back_home
        
        # 发送至pss信号
        pss_communication_response = pss_communicationResponse()
        pss_communication_response.box_request_in_process = box_request_in_process
        pss_communication_response.box_request_finished = box_request_finished
        pss_communication_response.pen_request_in_process = pen_request_in_process
        pss_communication_response.pen_request_finished = pen_request_finished
        pss_communication_response.box_handout_in_process = box_handout_in_process
        pss_communication_response.box_handout_finished = box_handout_finished
        pss_communication_response.pen_handout_in_process = pen_handout_in_process
        pss_communication_response.pen_handout_finished = pen_handout_finished
        pss_communication_response.robot_at_home = robot_at_home
        pss_communication_response.robot_moving = robot_moving
        pss_communication_response.robot_stopped = robot_stopped
        pss_communication_response.box_missing = box_missing
        pss_communication_response.pen_missing = pen_missing
        pss_communication_response.use_gripper = use_gripper
        pss_communication_response.use_sucker = use_sucker
        pss_communication_response.gripper_open = gripper_open
        pss_communication_response.gripper_close = gripper_close
        pss_communication_response.sucker_on = sucker_on
        return pss_communication_response


def cap_and_analyze():
    global pen_pick_X_list
    global box_pick_X_list
    global box_missing
    global pen_missing
    take_picutre(cap_original_file_path)
    undistort_pic(cap_original_file_path, cap_calibrated_file_path)
    pen_pick_X_list = get_pen_pose(cap_calibrated_file_path)
    box_pick_X_list = get_box_pose(cap_calibrated_file_path)
    if len(pen_pick_X_list) == 0:
        rospy.loginfo("pen missing!")
        pen_missing = True
    else:
        pen_missing = False
    if len(box_pick_X_list) == 0:
        rospy.loginfo("box missing!")
        box_missing = True
    else:
        box_missing = False

# 主程序
def start_program():
    # 接收自pss信号
    global box_request
    global pen_request
    global box_handout
    global pen_handout
    # 发送至pss信号
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

    global pen_pick_X_list
    global box_pick_X_list

    pss4000 = PssCommunicationNode()

    rospy.loginfo("Program started")  # log

    cap_and_analyze()
    robot_moving = True
    robot_at_home = False
    current_pose = r.get_current_pose()
    if current_pose.position.z < SAFETY_HEIGHT:
        r.move(Lin(goal=Pose(position=Point(0, 0, -0.05)), reference_frame="prbt_tcp", vel_scale=LIN_SCALE, acc_scale=0.1))
    elif current_pose.position.y > 0.33 and current_pose.position.z > 0.4:
        r.move(Lin(goal=Pose(position=Point(0, 0, -0.1)), reference_frame="prbt_tcp", vel_scale=LIN_SCALE, acc_scale=0.1))
    r.move(Ptp(goal=START_POSE, vel_scale=PTP_SCALE, acc_scale=0.1))
    robot_at_home = True

    while not rospy.is_shutdown():
        cap_and_analyze()
        if box_request:
            box_request_in_process = True
            box_request_finished = False
            use_gripper = False
            use_sucker = True
            sucker_on = False
            if len(box_pick_X_list) != 0:
                if box_pick_X_list[0] < -0.55:
                    box_pick_X_list[0] = -0.578
                    STOCK_BOX_Y = 0.037
                elif box_pick_X_list[0] > -0.35:
                    STOCK_BOX_Y = 0.034
                else:
                    STOCK_BOX_Y = 0.0355
                box_stock_pick_up_pose = Pose(position=Point(box_pick_X_list[0], STOCK_BOX_Y, STOCK_Z_UP), orientation=SUCKER_STOCK_ORIENTATION)
                box_stock_pick_down_pose = Pose(position=Point(box_pick_X_list[0], STOCK_BOX_Y, STOCK_BOX_Z_DOWN), orientation=SUCKER_STOCK_ORIENTATION)
                box_conveyor_place_up_pose = Pose(position=Point(-0.381, 0.1832, STOCK_Z_UP), orientation=SUCKER_PLATE_ORIENTATION)
                box_conveyor_place_down_pose = Pose(position=Point(-0.381, 0.1832, PLATE_BOX_Z_DOWN), orientation=SUCKER_PLATE_ORIENTATION)
                
                robot_at_home = False
                r.move(Ptp(goal=START_POSE, vel_scale=PTP_SCALE, acc_scale=0.1))
                r.move(Lin(goal=box_stock_pick_up_pose, reference_frame="prbt_base_link", vel_scale=LIN_SCALE, acc_scale=0.1))
                r.move(Lin(goal=box_stock_pick_down_pose, reference_frame="prbt_base_link", vel_scale=PnP_SCALE, acc_scale=0.1))
                sucker_on = True
                rospy.sleep(0.5)
                if len(box_pick_X_list) == 1:
                    box_missing = True

                r.move(Lin(goal=box_stock_pick_up_pose, reference_frame="prbt_base_link", vel_scale=PnP_SCALE, acc_scale=0.1))
                r.move(Lin(goal=box_conveyor_place_up_pose, reference_frame="prbt_base_link", vel_scale=LIN_SCALE, acc_scale=0.1))
                r.move(Lin(goal=box_conveyor_place_down_pose, reference_frame="prbt_base_link", vel_scale=PnP_SCALE, acc_scale=0.1))
                sucker_on = False
                rospy.sleep(0.5)
                r.move(Lin(goal=box_conveyor_place_up_pose, reference_frame="prbt_base_link", vel_scale=PnP_SCALE, acc_scale=0.1))
                box_request_in_process = False
                box_request_finished = True
                r.move(Ptp(goal=START_POSE, vel_scale=PTP_SCALE, acc_scale=0.1))
                robot_at_home = True
        else:
            box_request_finished = False


        if pen_request:
            pen_request_in_process = True
            pen_request_finished = False
            use_gripper = True
            use_sucker = False
            gripper_open = True
            gripper_close = False
            if len(pen_pick_X_list) != 0:
                if pen_pick_X_list[0] < -0.55:
                    STOCK_PEN_Y = -0.071
                elif box_pick_X_list[0] > -0.28:
                    STOCK_PEN_Y = -0.071
                else:
                    STOCK_PEN_Y = -0.07
                pen_stock_pick_up_pose = Pose(position=Point(pen_pick_X_list[0], STOCK_PEN_Y, STOCK_Z_UP), orientation=GRIPPER_STOCK_ORIENTATION)
                pen_stock_pick_down_pose = Pose(position=Point(pen_pick_X_list[0], STOCK_PEN_Y, STOCK_PEN_Z_DOWN), orientation=GRIPPER_STOCK_ORIENTATION)
                pen_conveyor_place_up_pose = Pose(position=Point(-0.368, 0.173, STOCK_Z_UP), orientation=GRIPPER_PLATE_PLACE_ORIENTATION)
                pen_conveyor_place_down_pose = Pose(position=Point(-0.368, 0.173, PLATE_PEN_Z_DOWN), orientation=GRIPPER_PLATE_PLACE_ORIENTATION)
                
                robot_at_home = False
                r.move(Ptp(goal=START_POSE, vel_scale=PTP_SCALE, acc_scale=0.1))
                r.move(Lin(goal=pen_stock_pick_up_pose, reference_frame="prbt_base_link", vel_scale=LIN_SCALE, acc_scale=0.1))
                r.move(Lin(goal=pen_stock_pick_down_pose, reference_frame="prbt_base_link", vel_scale=PnP_SCALE, acc_scale=0.1))
                gripper_open = False
                gripper_close = True
                rospy.sleep(0.5)
                r.move(Lin(goal=pen_stock_pick_up_pose, reference_frame="prbt_base_link", vel_scale=PnP_SCALE, acc_scale=0.1))
                if len(pen_pick_X_list) == 1:
                    pen_missing = True
                
                r.move(Lin(goal=pen_conveyor_place_up_pose, reference_frame="prbt_base_link", vel_scale=LIN_SCALE, acc_scale=0.1))
                r.move(Lin(goal=pen_conveyor_place_down_pose, reference_frame="prbt_base_link", vel_scale=PnP_SCALE, acc_scale=0.1))
                gripper_open = True
                gripper_close = False
                rospy.sleep(0.5)
                r.move(Lin(goal=pen_conveyor_place_up_pose, reference_frame="prbt_base_link", vel_scale=PnP_SCALE, acc_scale=0.1))
                pen_request_in_process = False
                pen_request_finished = True
                r.move(Ptp(goal=START_POSE, vel_scale=PTP_SCALE, acc_scale=0.1))
                robot_at_home = True
        else:
            pen_request_finished = False
        

        if box_handout:
            box_handout_in_process = True
            box_handout_finished = False
            use_gripper = False
            use_sucker = True
            sucker_on = False
            box_conveyor_pick_up_pose = Pose(position=Point(-0.137, 0.331, STOCK_Z_UP), orientation=SUCKER_PLATE_ORIENTATION)
            box_conveyor_pick_down_pose = Pose(position=Point(-0.137, 0.331, PLATE_BOX_Z_DOWN), orientation=SUCKER_PLATE_ORIENTATION)
            box_outlet_in_pose = [-0.018, -0.02, 1.434, -0.769, 1.646, 0.685]
            box_outlet_out_pose = [-0.447, -0.13, 1.3, -0.693, 1.377, 0.354]
                
            robot_at_home = False
            r.move(Ptp(goal=START_POSE, vel_scale=PTP_SCALE, acc_scale=0.1))
            r.move(Lin(goal=box_conveyor_pick_up_pose, reference_frame="prbt_base_link", vel_scale=LIN_SCALE, acc_scale=0.1))
            r.move(Lin(goal=box_conveyor_pick_down_pose, reference_frame="prbt_base_link", vel_scale=PnP_SCALE, acc_scale=0.1))
            sucker_on = True
            rospy.sleep(0.5)
            r.move(Lin(goal=box_conveyor_pick_up_pose, reference_frame="prbt_base_link", vel_scale=PnP_SCALE, acc_scale=0.1))
            box_handout_in_process = False
            box_handout_finished = True
            
            box_handout_seq = Sequence()
            box_handout_seq.append(Lin(goal=START_POSE, vel_scale=LIN_SCALE, acc_scale=0.1), blend_radius=0.05)
            box_handout_seq.append(Lin(goal=box_outlet_in_pose, reference_frame="prbt_base_link", vel_scale=LIN_SCALE, acc_scale=0.1), blend_radius=0.05)
            box_handout_seq.append(Lin(goal=box_outlet_out_pose, reference_frame="prbt_base_link", vel_scale=PnP_SCALE, acc_scale=0.1))
            r.move(box_handout_seq)
            sucker_on = False
            rospy.sleep(0.5)
            r.move(Lin(goal=box_outlet_in_pose, reference_frame="prbt_base_link", vel_scale=PnP_SCALE, acc_scale=0.1))
            r.move(Ptp(goal=START_POSE, vel_scale=PTP_SCALE, acc_scale=0.1))
            robot_at_home = True


        if pen_handout:
            pen_handout_in_process = True
            pen_handout_finished = False
            use_gripper = True
            use_sucker = False
            gripper_open = True
            gripper_close = False
            pen_conveyor_pick_up_pose = Pose(position=Point(-0.137, 0.332, STOCK_Z_UP), orientation=GRIPPER_PLATE_PICK_ORIENTATION)
            pen_conveyor_pick_down_pose = Pose(position=Point(-0.137, 0.332, PLATE_PEN_Z_DOWN + 0.001), orientation=GRIPPER_PLATE_PICK_ORIENTATION)
            pen_outlet_in_pose = [0.148, -0.104, 1.235, -0.791, 1.843, -0.789]
            pen_outlet_out_pose = [-0.403, -0.155, 1.229, -0.7, 1.44, -1.155]
                
            robot_at_home = False
            r.move(Ptp(goal=START_POSE, vel_scale=PTP_SCALE, acc_scale=0.1))
            r.move(Lin(goal=pen_conveyor_pick_up_pose, reference_frame="prbt_base_link", vel_scale=LIN_SCALE, acc_scale=0.1))
            r.move(Lin(goal=pen_conveyor_pick_down_pose, reference_frame="prbt_base_link", vel_scale=PnP_SCALE, acc_scale=0.1))
            gripper_open = False
            gripper_close = True
            rospy.sleep(0.5)
            r.move(Lin(goal=pen_conveyor_pick_up_pose, reference_frame="prbt_base_link", vel_scale=PnP_SCALE, acc_scale=0.1))
            pen_handout_in_process = False
            pen_handout_finished = True

            pen_handout_seq = Sequence()
            pen_handout_seq.append(Lin(goal=START_POSE, vel_scale=LIN_SCALE, acc_scale=0.1), blend_radius=0.05)
            pen_handout_seq.append(Lin(goal=pen_outlet_in_pose, reference_frame="prbt_base_link", vel_scale=LIN_SCALE, acc_scale=0.1), blend_radius=0.05)
            pen_handout_seq.append(Lin(goal=pen_outlet_out_pose, reference_frame="prbt_base_link", vel_scale=PnP_SCALE, acc_scale=0.1))
            r.move(pen_handout_seq)
            gripper_open = True
            gripper_close = False
            rospy.sleep(0.5)
            r.move(Lin(goal=pen_outlet_in_pose, reference_frame="prbt_base_link", vel_scale=PnP_SCALE, acc_scale=0.1))
            r.move(Ptp(goal=START_POSE, vel_scale=PTP_SCALE, acc_scale=0.1))
            robot_at_home = True
        else:
            pen_handout_finished = False


if __name__ == "__main__":
    # init a rosnode
    rospy.init_node('robot_program_node')

    # initialisation
    r = Robot(REQUIRED_API_VERSION)  # instance of the robot

    # start the main program
    start_program()
    rospy.spin()
