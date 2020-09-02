#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
import math
import socket
from geometry_msgs.msg import Pose, Point
from pilz_robot_programming import Robot, Ptp, Lin, from_euler
from pymodbus.client.sync import ModbusTcpClient
from intellegent_platform.srv import PnozVirtualOutputs, PnozVirtualOutputsResponse
from intellegent_platform.srv import RevPi_Communication, RevPi_CommunicationResponse
from take_picture import take_picutre, undistort_pic
from get_cylinder_pose import get_cylinder_pose
from get_cube_pose import get_cube_pose
from std_msgs.msg import String


# API版本号（不允许修改）
REQUIRED_API_VERSION = "1"    # API version


# 位置常量（因涉及到实际机械位置，因此不要修改）
START_POS = [0, 0, 1.57, 0, 1.57, 1.658]    # joint values
GRIPPER_ORIENTATION = from_euler(0, math.radians(180),  math.radians(95))
SORTING_CAP_ORIENTATION = from_euler(0, math.radians(180),  math.radians(6))
CYLINDER_PLACE_START_X = -0.331
CYLINDER_PLACE_START_Y = -0.471
CYLINDER_PLACE_START_Z = 0.192
CUBE_PLACE_START_X = -0.231
CUBE_PLACE_START_Y = -0.472
CUBE_PLACE_START_Z = 0.192
DISTANCE = 0.05
SAFETY_HEIGHT = 0.223


# 照片存放路径
sorting_original_file_path = "/home/dell/Pictures/intellegent_platform_pictures/sorting_cap.png" 
sorting_calibrated_file_path = "/home/dell/Pictures/intellegent_platform_pictures/sorting_cap_calibrated.png"

# 速度常量
PTP_SCALE = 0.2        # Ptp移动速度比例
LIN_SCALE = 0.1        # Lin移动速度比例
ACC_SCALE = 0.1        # 加速度比例

# 初始化变量
robot_sto_last = True
scanner_warning_zone1_last = True
scanner_warning_zone2_last = True
HMI_stop_button = False
HMI_resume_button = False
HMI_sorting_button = False
sorting_pnp_status = [False] * 8
sorting_cylinder_status = [False] * 8
sorting_cube_status = [False] * 8
socket.setdefaulttimeout(30)
gestures = ' '


class PLCControllerNode(object):
    def __init__(self):
        self.client = ModbusTcpClient('10.50.4.101', port=502)
        self.PNOZmulti_service = rospy.Service('pnoz_virtual_outputs', PnozVirtualOutputs, self.handle_pnoz_virtual_outputs)

    def handle_pnoz_virtual_outputs(self, req):
        # 来自PNOZmulti的虚拟输出
        global robot_sto_last                 # 存储上一周期的状态
        global scanner_warning_zone1_last     # 存储上一周期的状态
        global scanner_warning_zone2_last     # 存储上一周期的状态
        global conveyor_start_sensor          # PNOZmulti O5
        global conveyor_end_sensor            # PNOZmulti O6
        global HMI_stop_button
        global HMI_resume_button
        global PTP_SCALE
        global LIN_SCALE
        robot_sto = req.pnoz_virtual_outputs[0]
        scanner_warning_zone1 = req.pnoz_virtual_outputs[1]
        scanner_warning_zone2 = req.pnoz_virtual_outputs[2]
        conveyor_start_sensor = req.pnoz_virtual_outputs[5]
        conveyor_end_sensor = req.pnoz_virtual_outputs[6]
        gatebox_reset_button = req.pnoz_virtual_outputs[37]
        gatebox_stop_button = req.pnoz_virtual_outputs[44]

        if (not robot_sto and robot_sto_last) or gatebox_stop_button or HMI_stop_button:
            r.pause()
            rospy.loginfo("Robot paused")
        elif HMI_resume_button or gatebox_reset_button:
            rospy.sleep(3)
            r.resume()
            rospy.loginfo("Robot resumed")
            current_pose = r.get_current_pose()
            if current_pose.position.z < SAFETY_HEIGHT:
                r.move(Lin(goal=Pose(position=Point(0, 0, -0.03)), reference_frame="prbt_tcp", vel_scale=LIN_SCALE))
        robot_sto_last = robot_sto

        if not scanner_warning_zone2 and scanner_warning_zone2_last:
            rospy.loginfo("Warning zone 2 triggered")
            PTP_SCALE = 0.1
            LIN_SCALE = 0.05
        elif scanner_warning_zone2 and not scanner_warning_zone2_last:
            rospy.loginfo("Warning zone 2 recovered")
            PTP_SCALE = 0.2
            LIN_SCALE = 0.1
        scanner_warning_zone2_last = scanner_warning_zone2

        if not scanner_warning_zone1 and scanner_warning_zone1_last:
            rospy.loginfo("Warning zone 1 triggered")
            PTP_SCALE = 0.05
            LIN_SCALE = 0.01
        elif scanner_warning_zone1 and scanner_warning_zone2 and not scanner_warning_zone1_last:
            rospy.loginfo("Warning zone 1 recovered")
            PTP_SCALE = 0.2
            LIN_SCALE = 0.1
        scanner_warning_zone1_last = scanner_warning_zone1

        return PnozVirtualOutputsResponse() 

    def gripper(self, state):
        self.client.write_coil(0, state)
        self.client.write_coil(1, not state)

    def tool_change(self, state):
        self.client.write_coil(2, state)
        self.client.write_coil(3, not state)

    def sucker(self, state):
        self.client.write_coil(4, state)
        self.client.write_coil(5, not state)

    def conveyor(self, state):
        self.client.write_coil(6, state)


class RevPiCommunicationNode(object):
    def __init__(self):
        ModbusTcpClient('10.50.4.103', port=502)
        rospy.Service('RevPi_communication',
                      RevPi_Communication,
                      self.handle_RevPi_communication)

    def handle_RevPi_communication(self, req):
        global HMI_stop_button                       # RevPi bit o2
        global HMI_sorting_button                    # RevPi bit o3
        global HMI_resume_button                     # RevPi bit o8
        global sorting_pnp_status
        global sorting_cylinder_status
        global sorting_cube_status
        HMI_stop_button = req.RevPi_virtual_outputs[2]
        HMI_sorting_button = req.RevPi_virtual_outputs[3]
        HMI_resume_button = req.RevPi_virtual_outputs[8]
        RevPi_communication_response = RevPi_CommunicationResponse()
        RevPi_communication_response.sorting_cylinder_status = sorting_cylinder_status
        RevPi_communication_response.sorting_cube_status = sorting_cube_status
        RevPi_communication_response.sorting_pnp_status = sorting_pnp_status
        return RevPi_communication_response


def sucker_pick(PNOZmulti):
    """使用吸盘抓取"""

    # 相对于TCP的移动位置
    r.move(Lin(goal=Pose(position=Point(0, 0, 0.03)), reference_frame="prbt_tcp", vel_scale=LIN_SCALE))
    PNOZmulti.sucker(True)
    rospy.sleep(0.5)
    r.move(Lin(goal=Pose(position=Point(0, 0, -0.03)), reference_frame="prbt_tcp", vel_scale=LIN_SCALE))


def sucker_place(PNOZmulti):
    """使用吸盘放置"""

    # 相对于TCP的移动位置
    r.move(Lin(goal=Pose(position=Point(0, 0, 0.03)), reference_frame="prbt_tcp", vel_scale=LIN_SCALE))
    PNOZmulti.sucker(False)
    rospy.sleep(0.5)
    r.move(Lin(goal=Pose(position=Point(0, 0, -0.03)), reference_frame="prbt_tcp", vel_scale=LIN_SCALE))


def deep_sensor_callback(data):
    global gestures
    gestures = data.data


# main program
def start_program():

    global HMI_sorting_button
    global sorting_cylinder_status
    global sorting_cube_status
    global sorting_pnp_status
    global gestures

    rospy.loginfo("Program started")

    PNOZmulti = PLCControllerNode()
    PNOZmulti.sucker(False)

    RevPi = RevPiCommunicationNode()

    while not rospy.is_shutdown():
        current_pose = r.get_current_pose()
        if HMI_sorting_button:
            if current_pose.position.z < SAFETY_HEIGHT:
                r.move(Lin(goal=Pose(position=Point(0, 0, -0.03)), reference_frame="prbt_tcp", vel_scale=LIN_SCALE))

            # # Create a TCP/IP socket
            # sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

            # # Bind the socket to the port
            # server_address = ('0.0.0.0', 10000)
            # print('starting up on %s port %s' % server_address)
            # sock.bind(server_address)

            # # Listen for incoming connections
            # sock.listen(1)
            # print('waiting for a connection')
            # connection, client_address = sock.accept()
            # print('connection from' + str(client_address))

            sorting_cap_pose = Pose(position=Point(0.06, -0.33, 0.51),
                                    orientation=SORTING_CAP_ORIENTATION)

            # 移动至起始位置
            r.move(Ptp(goal=START_POS, vel_scale=PTP_SCALE))

            # 移动至拍照位置
            r.move(Ptp(goal=sorting_cap_pose, reference_frame="prbt_base_link",
                       vel_scale=PTP_SCALE, relative=False))

            # 拍照并校正图像
            take_picutre(sorting_original_file_path)
            undistort_pic(sorting_original_file_path, sorting_calibrated_file_path)
            rospy.sleep(0.5)

            # 获取拾取位置及角度
            cyl_pick_X_list, cyl_pick_Y_list = get_cylinder_pose(sorting_calibrated_file_path)
            cube_pick_X_list, cube_pick_Y_list, cube_angle_list = get_cube_pose(sorting_calibrated_file_path)
            # data = str(cyl_pick_X_list + cyl_pick_Y_list + cube_pick_X_list + cube_pick_Y_list + cube_angle_list)
            # connection.sendall(data)
            # rospy.sleep(2)

            # 存储圆柱体放置位置
            cylinder_place_poses = []
            for i in range(2):
                for j in range(4):
                    cylinder_place_pose = Pose(position=Point(CYLINDER_PLACE_START_X+i*DISTANCE, CYLINDER_PLACE_START_Y+j*DISTANCE, CYLINDER_PLACE_START_Z), orientation=SORTING_CAP_ORIENTATION)
                    cylinder_place_poses.append(cylinder_place_pose)

            # 拾取圆柱体
            # for i in range(len(cyl_pick_X_list)):
            #     cylinder_pose = Pose(position=Point(cyl_pick_X_list[i], cyl_pick_Y_list[i], CYLINDER_PLACE_START_Z - 0.002), orientation=SORTING_CAP_ORIENTATION)
            #     r.move(Lin(goal=cylinder_pose, reference_frame="prbt_base_link", vel_scale=LIN_SCALE, relative=False))
            #     sucker_pick(PNOZmulti)
            #     sorting_pnp_status[0] = False
            #     sorting_cylinder_status[i] = True
            #     r.move(Lin(goal=cylinder_place_poses[i], reference_frame="prbt_base_link", vel_scale=LIN_SCALE, relative=False))
            #     sucker_place(PNOZmulti)
            #     sorting_pnp_status[0] = True
            #     sorting_cylinder_status[i] = False
            cylinder_pose = Pose(position=Point(cyl_pick_X_list[0], cyl_pick_Y_list[0], CYLINDER_PLACE_START_Z - 0.002), orientation=SORTING_CAP_ORIENTATION)
            r.move(Lin(goal=cylinder_pose, reference_frame="prbt_base_link", vel_scale=LIN_SCALE, relative=False))

            i = 0
            count_left = 0
            count_right = 0
            while i < len(cyl_pick_X_list):
                if gestures == 'left':
                    if i == 0:
                        cylinder_pose = Pose(position=Point(cyl_pick_X_list[i], cyl_pick_Y_list[i], CYLINDER_PLACE_START_Z - 0.002), orientation=SORTING_CAP_ORIENTATION)
                    else:
                        count_left += 1
                        count_right -= 1
                        if count_left <= i:
                            cylinder_pose = Pose(position=Point(cyl_pick_X_list[i-count_left], cyl_pick_Y_list[i-count_left], CYLINDER_PLACE_START_Z - 0.002), orientation=SORTING_CAP_ORIENTATION)
                        elif count_left > i:
                            count_left = i
                            count_right = -i
                    r.move(Lin(goal=cylinder_pose, reference_frame="prbt_base_link", vel_scale=LIN_SCALE, relative=False))
                    gestures = ' '
                    continue

                if gestures == 'right':
                    if i == len(cyl_pick_X_list)-1:
                        cylinder_pose = Pose(position=Point(cyl_pick_X_list[i], cyl_pick_Y_list[i], CYLINDER_PLACE_START_Z - 0.002), orientation=SORTING_CAP_ORIENTATION)
                    else:
                        count_right += 1
                        count_left -= 1
                        if count_right <= len(cyl_pick_X_list)-1-i:
                            cylinder_pose = Pose(position=Point(cyl_pick_X_list[i+count_right], cyl_pick_Y_list[i+count_right], CYLINDER_PLACE_START_Z - 0.002), orientation=SORTING_CAP_ORIENTATION)
                        elif count_right > len(cyl_pick_X_list)-1-i:
                            count_right = len(cyl_pick_X_list)-1-i
                            count_left = -(len(cyl_pick_X_list)-1-i)
                    r.move(Lin(goal=cylinder_pose, reference_frame="prbt_base_link", vel_scale=LIN_SCALE, relative=False))
                    gestures = ' '
                    continue

                if gestures == 'down':
                    sucker_pick(PNOZmulti)
                    sorting_pnp_status[0] = False
                    sorting_cylinder_status[i] = True
                    r.move(Lin(goal=cylinder_place_poses[i], reference_frame="prbt_base_link", vel_scale=LIN_SCALE, relative=False))
                    sucker_place(PNOZmulti)
                    sorting_pnp_status[0] = True
                    sorting_cylinder_status[i] = False
                    r.move(Lin(goal=cylinder_pose, reference_frame="prbt_base_link", vel_scale=LIN_SCALE, relative=False))
                    i += 1
                    gestures = ' '
                    continue


            # 存储立方体放置位置
            cube_place_poses = []
            for i in range(2):
                for j in range(4):
                    cube_place_pose = Pose(position=Point(CUBE_PLACE_START_X+i*DISTANCE, CUBE_PLACE_START_Y+j*DISTANCE, CUBE_PLACE_START_Z), orientation=SORTING_CAP_ORIENTATION)
                    cube_place_poses.append(cube_place_pose)

            # 拾取立方体
            for i in range(len(cube_pick_X_list)):
                cube_pose = Pose(position=Point(cube_pick_X_list[i], cube_pick_Y_list[i], CUBE_PLACE_START_Z - 0.002), orientation=from_euler(0, math.radians(180),  math.radians(-cube_angle_list[i]+6)))
                r.move(Lin(goal=cube_pose, reference_frame="prbt_base_link", vel_scale=LIN_SCALE, relative=False))
                sucker_pick(PNOZmulti)
                sorting_pnp_status[0] = False
                sorting_cube_status[i] = True
                r.move(Lin(goal=cube_place_poses[i], reference_frame="prbt_base_link", vel_scale=LIN_SCALE, relative=False))
                sucker_place(PNOZmulti)
                sorting_pnp_status[0] = True
                sorting_cube_status[i] = False

            # 移动至起始位置
            r.move(Ptp(goal=START_POS, reference_frame="prbt_base_link", vel_scale=PTP_SCALE, relative=False))
            sorting_pnp_status[0] = False
            # connection.close()
            rospy.loginfo("Program finished")


if __name__ == "__main__":
    # init a rosnode
    rospy.init_node('robot_program_node')
    rospy.Subscriber("gesture", String, deep_sensor_callback)

    # initialisation
    r = Robot(REQUIRED_API_VERSION)  # instance of the robot

    # start the main program
    start_program()
    rospy.spin()
