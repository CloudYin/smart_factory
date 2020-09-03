#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from smart_factory.srv import pss_communication
from pymodbus.client.sync import ModbusTcpClient


# Send to pss 4000
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


def connect_pss(client):
    pss_virtual_outputs = []
    loop_rate = rospy.Rate(1)
    if client.connect():
        result = client.read_coils(50, 30)  # read(start address, length)
        for i in range(0, 30):
            pss_virtual_outputs.append(result.bits[i])

        # write(start address, contents)
        client.write_coil(0, box_request_in_process)
        client.write_coil(1, box_request_finished)
        client.write_coil(2, pen_request_in_process)
        client.write_coil(3, pen_request_finished)
        client.write_coil(4, box_handout_in_process)
        client.write_coil(5, box_handout_finished)
        client.write_coil(6, pen_handout_in_process)
        client.write_coil(7, pen_handout_finished)
        client.write_coil(8, robot_at_home)
        client.write_coil(9, robot_moving)
        client.write_coil(10, robot_stopped)
        client.write_coil(11, box_missing)
        client.write_coil(12, pen_missing)
        client.write_coil(13, use_gripper)
        client.write_coil(14, use_sucker)
        client.write_coil(15, gripper_open)
        client.write_coil(16, gripper_close)
        client.write_coil(17, sucker_on)
    else:
        print("Modbus TCP connect to PSS is interrupted. Connecting...")
        client = ModbusTcpClient(host='192.168.1.2', port=502)
        loop_rate.sleep()
    return pss_virtual_outputs


if __name__ == "__main__":
    rospy.init_node('pss_communication_node')
    loop_rate = rospy.Rate(100)
    client = ModbusTcpClient(host='192.168.1.2', port=502)
    while not rospy.is_shutdown():
        pss_virtual_outputs = connect_pss(client)
        rospy.wait_for_service('pss_communication')
        try:
            pss_comm_client = rospy.ServiceProxy('pss_communication', pss_communication)
            res = pss_comm_client(pss_virtual_outputs)
            box_request_in_process = res.box_request_in_process
            box_request_finished = res.box_request_finished
            pen_request_in_process = res.pen_request_in_process
            pen_request_finished = res.pen_request_finished
            box_handout_in_process = res.box_handout_in_process
            box_handout_finished = res.box_handout_finished
            pen_handout_in_process = res.pen_handout_in_process
            pen_handout_finished = res.pen_handout_finished
            robot_at_home = res.robot_at_home
            robot_moving = res.robot_moving
            robot_stopped = res.robot_stopped
            box_missing = res.box_missing
            pen_missing = res.pen_missing
            use_gripper = res.use_gripper
            use_sucker = res.use_sucker
            gripper_open = res.gripper_open
            gripper_close = res.gripper_close
            sucker_on = res.sucker_on
        except rospy.ServiceException:
            print("Service call failed.")
        loop_rate.sleep()
    client.close()
