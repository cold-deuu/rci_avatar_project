#!/usr/bin/env python

"""This example demonstrates how to read the handle infos. """

__author__ = "Antoine Weill--Duflos"
__copyright__ = "Copyright 2023, HaplyRobotics"

import HaplyHardwareAPI


class Handle(HaplyHardwareAPI.Handle):
    def __init__(self, com):
        HaplyHardwareAPI.Handle.__init__(self, com)

    def OnReceiveHandleInfo(self, data_remaining, device_id, device_model, hardware_version, firmware_version):
        print("Tool info received, device ID: " + str(device_id) + " device model: " + str(device_model) +
              " hardware version: " + str(hardware_version) + " firmware version: " + str(firmware_version))

    def OnReceiveHandleStatusMessage(self, device_id, quaternion, error_flag, hall_effect_sensor_level, user_data_length, user_data):
        print("Tool status received")
        print("Device ID: " + str(device_id))
        print("Error flag: " + str(error_flag))
        print("Hall effect sensor level: " + str(hall_effect_sensor_level))
        print("quaternion: " + str(quaternion[0]) + " " + str(
            quaternion[1]) + " " + str(quaternion[2]) + " " + str(quaternion[3]))
        for i in range(user_data_length):
            print("user_data: " + str(user_data[i]))

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
print(connected_handles)
handle_stream = HaplyHardwareAPI.SerialStream(connected_handles[0])
handle = Handle(handle_stream)
handle.SendDeviceWakeup()
handle.Receive()
while True:
    handle.RequestStatus()
    handle.Receive()
