#!/usr/bin/env python

"""This example demonstrates how to follow a trajectory with the inverse3. """

__author__ = "Antoine Weill--Duflos"
__copyright__ = "Copyright 2023, HaplyRobotics"

import HaplyHardwareAPI
import time
import math
connected_devices = HaplyHardwareAPI.detect_inverse3s()
com_stream = HaplyHardwareAPI.SerialStream(connected_devices[0])
inverse3 = HaplyHardwareAPI.Inverse3(com_stream)
response_to_wakeup = inverse3.device_wakeup_dict()
print("connected to device {}".format(response_to_wakeup["device_id"]))
start_time = time.perf_counter()
loop_time = 0.001  # 1ms
center_workspace = [0.03, -0.13, 0.20]
target_position = center_workspace
frequency = 0.5
while True:
    # do a circle on the xz plane with radius 0.0001
    target_position[0] = center_workspace[0] + \
        (0.0001 * math.cos(time.perf_counter()*math.pi*2*frequency))
    target_position[2] = center_workspace[2] + 0.0001 * \
        math.sin(time.perf_counter()*math.pi*2*frequency)
    
    
    position, velocity = inverse3.end_effector_force(10 * target_position)
    # joint_angles = inverse3.ReceiveJointState()
    # position = inverse3.joint_angles([10, 10, 10])
    print("position: {}".format(position))
    while time.perf_counter() - start_time < loop_time:  # wait for loop time to be reached
        pass
    start_time = time.perf_counter()
