#!/usr/bin/env python

"""This example demonstrates how to display a haptic ball and visualise it in vpython. """

__author__ = "Antoine Weill--Duflos"
__copyright__ = "Copyright 2023, HaplyRobotics"

import HaplyHardwareAPI
import time
import math
from vpython import *
connected_devices = HaplyHardwareAPI.detect_inverse3s()
com_stream = HaplyHardwareAPI.SerialStream(connected_devices[0])
inverse3 = HaplyHardwareAPI.Inverse3(com_stream)
response_to_wakeup = inverse3.device_wakeup_dict()
print("connected to device {}".format(response_to_wakeup["device_id"]))
start_time = time.perf_counter()
loop_time = 0.001  # 1ms
forces = [0, 0, 0]
# create spheres
inverse_tip = sphere(pos=vector(0, 0, 0), radius=0.02)
object_ball = sphere(pos=vector(0, 0.2, 0.14), radius=0.08, color=color.red)


def force_sphere(sphere_center, sphere_radius, device_position, stiffness):
    distance = math.sqrt(
        sum([(device_position[i] - sphere_center[i])**2 for i in range(3)]))
    if distance > sphere_radius:
        return [0, 0, 0]
    else:
        # Compute the normalised direction of the forces
        direction = [(device_position[i] - sphere_center[i])/sphere_radius
                     for i in range(3)]
        # Compute the force
        force = [direction[i]*(sphere_radius-distance)
                 * stiffness for i in range(3)]
        return force


while True:
    position, velocity = inverse3.end_effector_force(forces)
    forces = force_sphere([0, -0.14, 0.2], 0.08, position, stiffness=800)
    # NB axes are switched and reoriented
    inverse_tip.pos = vector(position[0], position[2], -position[1])
    print("position: {}".format(position))
    while time.perf_counter() - start_time < loop_time:  # wait for loop time to be reached
        pass
    start_time = time.perf_counter()
