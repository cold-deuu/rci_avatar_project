#!/usr/bin/env python

"""This example demonstrates how to read the handle quaternion and display it in vpython. """

__author__ = "Antoine Weill--Duflos"
__copyright__ = "Copyright 2023, HaplyRobotics"

import HaplyHardwareAPI
import threading
import time
from vpython import *
import math
import pyquaternion

inverse_tip = arrow(pos=vector(0, 0, 0), color=color.yellow)


class Handle(HaplyHardwareAPI.Handle):
    quat = [0, 0, 0, 0]

    def __init__(self, com):
        HaplyHardwareAPI.Handle.__init__(self, com)

    def OnReceiveHandleInfo(self, data_remaining, device_id, device_model, hardware_version, firmware_version):
        print("Tool info received, device ID: " + str(device_id) +
              " device model: " + str(device_model))

    def OnReceiveHandleStatusMessage(self, device_id, quaternion, error_flag, hall_effect_sensor_level, user_data_length, user_data):
        self.quat = quaternion

    def OnReceiveHandleErrorResponse(self):
        print("Tool error received")

    def RequestStatus(self):
        HaplyHardwareAPI.Handle.RequestStatus(self)


connected_devices = HaplyHardwareAPI.detect_inverse3s()
connected_handles = HaplyHardwareAPI.detect_handles()
com_stream = HaplyHardwareAPI.SerialStream(connected_devices[0])
inverse3 = HaplyHardwareAPI.Inverse3(com_stream)
response_to_wakeup = inverse3.device_wakeup_dict()
for key in response_to_wakeup:
    print(key, response_to_wakeup[key])
handle_stream = HaplyHardwareAPI.SerialStream(connected_handles[0])
handle = Handle(handle_stream)
handle.SendDeviceWakeup()
handle.Receive()
run = True


def handle_thread():
    global run
    handle_time = time.perf_counter()
    count = 0
    while run:
        count += 1
        handle.RequestStatus()
        handle.Receive()
        if time.perf_counter() - handle_time > 1:
            print(" Handle thread rate: " +
                  str(count / (time.perf_counter() - handle_time)))
            handle_time = time.perf_counter()
            count = 0


position = [0, 0, 0]


def inverse_thread():
    global position, run
    inverse_time = time.perf_counter()
    count = 0
    while run:
        position, velocity = inverse3.end_effector_force([0, 0, 0])
        count += 1
        if time.perf_counter() - inverse_time > 1:
            print("Inverse thread rate: " + str(count /
                  (time.perf_counter() - inverse_time)))
            inverse_time = time.perf_counter()
            count = 0


handle_thread = threading.Thread(target=handle_thread)
inverse_thread = threading.Thread(target=inverse_thread)
handle_thread.start()
inverse_thread.start()
scene.up = vector(0, 0, 1)
scene.forward = vector(0, 1, 0)
while True:
    scaled_pos = [position[i] * 10.0 for i in range(3)]
    inverse_tip.pos = vector(scaled_pos[0], scaled_pos[1], scaled_pos[2])
    # convert quaternion to euler angles
    # invert yaw and roll

    # change vpython to match the Inverse coordinate frames.
    forward = (0, 1, 0)
    up = (0, 0, 1)
    # rotate forward by the quaternion
    quat = pyquaternion.Quaternion(
        handle.quat[0], handle.quat[1], handle.quat[2], handle.quat[3])
    forward = quat.rotate(forward)
    up = quat.rotate(up)
    inverse_tip.axis = vector(forward[0], forward[1], forward[2])
    inverse_tip.up = vector(up[0], up[1], up[2])

run = False
# stop the threads
handle_thread.join()
inverse_thread.join()
