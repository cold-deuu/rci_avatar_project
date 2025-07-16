#!/usr/bin/env python

"""This example demonstrates how to display a haptic wall. """

__author__ = "Antoine Weill--Duflos"
__copyright__ = "Copyright 2023, HaplyRobotics"

import HaplyHardwareAPI
import time
connected_devices = HaplyHardwareAPI.detect_inverse3s()
com_stream = HaplyHardwareAPI.SerialStream(connected_devices[0])
inverse3 = HaplyHardwareAPI.Inverse3(com_stream)
response_to_wakeup = inverse3.device_wakeup_dict()
print("connected to device {}".format(response_to_wakeup["device_id"]))
start_time = time.perf_counter()
loop_time = 0.001  # 1ms
forces = [0, 0, 0]
while True:
    position, velocity = inverse3.end_effector_force(forces)
    if position[2] < 0.1:
        forces[2] = (0.1 - position[2]) * 1000.0
    else:
        forces[2] = 0
    print("position: {}".format(position))
    while time.perf_counter() - start_time < loop_time:  # wait for loop time to be reached
        pass
    start_time = time.perf_counter()
