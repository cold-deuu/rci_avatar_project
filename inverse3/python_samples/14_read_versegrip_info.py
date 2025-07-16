#!/usr/bin/env python

"""This example demonstrates how to read the handle infos."""

__author__ = "Antoine Weill--Duflos"
__copyright__ = "Copyright 2024, HaplyRobotics"

import HaplyHardwareAPI


connected_handles = HaplyHardwareAPI.detect_handles()
print(connected_handles)
handle_stream = HaplyHardwareAPI.SerialStream(connected_handles[0])
versegrip = HaplyHardwareAPI.Handle(handle_stream)
while True:
    response = versegrip.GetVersegripStatus()
    print(response)
