/* -*- mode: c++ -*-
 * Copyright 2022 Haply Robotics Inc. All rights reserved.
 *
 * Simple haptic feedback loop using an arbitrary wall (floor) in the device's
 * workspace.
 */

#include <chrono>
#include <cstdio>

// The primary include file for the Haply C++ API.
#include "inverse3/HardwareAPI.h"

// Used to reduce verbosity within the examples.
namespace API = Haply::HardwareAPI;
using namespace std::chrono_literals;

// Arbitrary coordinate of the wall (floor) on the Z axis roughly located in the
// center of the device's workspace.
constexpr float wall = 0.17f;

// Constant used determine the strength of the device's haptic response as the
// user pushes against the wall. The higher the value the harder it will be to
// push through the wall.
constexpr float stiffness = 2500;

int main(int argc, const char *argv[]) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: 03-hello-wall [com-port]\n");
        return 1;
    }

    // Basic device setup. See earlier examples for a more detailed explanation.
    API::IO::SerialStream stream{argv[1]};
    API::Devices::Inverse3 device{&stream};
    (void)device.DeviceWakeup();

    // Haptic loops feel best when operating at or above 1000Hz. Given that
    // Windows doesn't provide any sleep facilities that can reliably operate at
    // these frequencies, we instead use busy loops which are timed using
    // high-resolution clocks (~100ns on Windows). For this example, we'll
    // target an update rate of 5 kHz which means a delta of 200 micro-seconds
    // in between each state update.
    typedef std::chrono::high_resolution_clock clock;
    auto next = clock::now();
    auto delay = 200us; // 5kHz

    API::Devices::Inverse3::EndEffectorStateResponse state;

    while (true) {

        API::Devices::Inverse3::EndEffectorForceRequest request;

        // If our end-effector position is positioned below our wall (floor)
        // we'll apply an upward force that is proportional to how deep into the
        // wall the end-effector currently is. Using proportional forces gives a
        // smoother haptic experience.
        if (state.position[2] < wall)
            request.force[2] = (wall - state.position[2]) * stiffness;

        // Send the current force value to the device and retrieve the current
        // position of the end-effector for our next loop iteration.
        state = device.EndEffectorForce(request);

        // Implementation of our busy-loop to regulate the frequency of our
        // state updates.
        next += delay;
        while (next > clock::now())
            ;
    }

    return 0;
}
