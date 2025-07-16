#!/usr/bin/env python

"""This example demonstrates how to make the end effector vibrate. """

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
forces = [0, 0, 0]
frequence = 110  # Hz
amplitude = 1  # N
while True:
    forces[1] = math.sin(time.perf_counter() * 2 *
                         math.pi * frequence) * amplitude
    position, velocity = inverse3.end_effector_force(forces)
    while time.perf_counter() - start_time < loop_time:  # wait for loop time to be reached
        pass
    start_time = time.perf_counter()
