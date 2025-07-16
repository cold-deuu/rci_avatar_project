#!/usr/bin/env python

"""This example demonstrates how to display a haptic ball and visualise it in vpython with multithread. """

__author__ = "Antoine Weill--Duflos"
__copyright__ = "Copyright 2023, HaplyRobotics"

import HaplyHardwareAPI
import time
import math
from vpython import *
import threading
import os
connected_devices = HaplyHardwareAPI.detect_inverse3s()
com_stream = HaplyHardwareAPI.SerialStream(connected_devices[0])
inverse3 = HaplyHardwareAPI.Inverse3(com_stream)
response_to_wakeup = inverse3.device_wakeup_dict()
print("connected to device {}".format(response_to_wakeup["device_id"]))
loop_time = 0.0005  # 1ms
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


position = [0, 0, 0]
velocity = [0, 0, 0]
running = True


def visualise_thread():
    global position, running
    while running:
        inverse_tip.pos = vector(position[0], position[2], -position[1])
        rate(60)


def haptic_thread():
    global position, running
    forces = [0, 0, 0]
    start_time = time.perf_counter()
    while running:
        position, velocity = inverse3.end_effector_force(forces)
        forces = force_sphere([0, -0.14, 0.2], 0.08, position, stiffness=800)
        if time.process_time() - start_time > loop_time:
            print("spent {}s in loop".format(time.process_time() - start_time))
        while time.perf_counter() - start_time < loop_time:  # wait for loop time to be reached
            pass
        start_time = time.perf_counter()


haptic_thread = threading.Thread(target=haptic_thread)
visualise_thread = threading.Thread(target=visualise_thread)
haptic_thread.start()
visualise_thread.start()
# wait for user to press enter to exit
input("Press Enter to exit")
running = False
haptic_thread.join()
print("haptic thread stopped")
visualise_thread.join()
print("visualise thread stopped")
# force the program to exit
os._exit(0)
