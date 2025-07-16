#!/usr/bin/env python

"""This example demonstrates how to read the handle quaternion in a separate thread. """

__author__ = "Antoine Weill--Duflos"
__copyright__ = "Copyright 2023, HaplyRobotics"

import HaplyHardwareAPI
import threading
import time


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
# print the response to the wakeup command
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

# print the position and quaternion 10 times then stop
loop = 0
while loop < 10:
    print("position: {}".format(position))
    print("quaternion: {}".format(handle.quat))
    time.sleep(1)
    loop += 1

run = False
# stop the threads
handle_thread.join()
inverse_thread.join()
