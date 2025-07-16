/* -*- mode: c++ -*-
 * Copyright 2022 Haply Robotics Inc. All rights reserved.
 *
 * Simple example of how to use the Inverse3 as a 3D mouse where the position
 * and velocity of the device's end-effector will be sent to stdout as it is
 * moved.
 */

#include <cstdio>

// The primary include file for the Haply C++ API.
#include "inverse3/HardwareAPI.h"

// Used to reduce verbosity within the examples.
namespace API = Haply::HardwareAPI;

int main(int argc, const char *argv[]) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: 01-print-inverse3 [com-port]\n");
        return 1;
    }

    // The `API::IO::SerialStream` object encapsulates the serial COM port that
    // will be used to communicate with a device. The argument given to its
    // constructor should be a COM port (e.g. COM6).
    API::IO::SerialStream stream{argv[1]};

    // Using the `API::IO::SerialStream` object we can initialize the Inverse3
    // object which will encapsulates all the logic needed to interact with an
    // Inverse3 device.
    API::Devices::Inverse3 device{&stream};

    // To start using the device, we first need to wake it up using the
    // `DeviceWakeup` function. Once awake, the LED colour on the device should
    // change to indicate that it's ready to receive commands. The method
    // returns a struct of type `Inverse3::DeviceInfoResponse` which contains
    // the device's general information.
    auto info = device.DeviceWakeup();
    std::fprintf(stdout,
                 "info: id=%u, model=%u, version={hardware:%u, firmware:%u}\n",
                 info.device_id, info.device_model_number,
                 info.hardware_version, info.firmware_version);

    while (true) {
        // Given that the API works synchronously, we call `EndEffectorForce`
        // with a null force vector to query the current position of the
        // end-effector. This method returns a struct of type
        // `Inverse3::EndEffectorStateResponse` which will contain a position
        // and velocity vector representing the current state of the device's
        // end-effector.
        auto state = device.EndEffectorForce({});
        std::fprintf(stdout,
                     "\r"
                     "position=[ % 0.3f % 0.3f % 0.3f ] "
                     "velocity=[ % 0.3f % 0.3f % 0.3f ]",
                     state.position[0], state.position[1], state.position[2],
                     state.velocity[0], state.velocity[1], state.velocity[2]);
    }

    return 0;
}
