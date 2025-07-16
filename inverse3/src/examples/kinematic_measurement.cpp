/* -*- mode: c++ -*-
 * Copyright 2024 Haply Robotics Inc. All rights reserved.
 *
 * Simple example of how to use the Inverse3 to do a kinematic measurement. The
 * kinematic measurement will be sent to stdout as it is moved.
 */

#include <cstdio>
#include <string>
#include <thread>

// The primary include file for the Haply C++ API.
#include "inverse3/HardwareAPI.h"

// Used to reduce verbosity within the examples.
namespace API = Haply::HardwareAPI;

int main() {
    std::string first_port;

    // Here we use the `Haply::HardwareAPI::DeviceDetection::DetectInverse3s`
    // static function to list all the Inverse3 devices currently connected. The
    // return value is a vector of string representing the serial COM ports that
    // can be used to communicate with the device.
    {
        auto list = API::Devices::DeviceDetection::DetectInverse3s();
        for (const auto &port : list)
            std::fprintf(stdout, "inverse3: %s\n", port.c_str());

        if (list.empty())
            fprintf(stderr, "no inverse3 detected\n");

        first_port = list[0];
        fprintf(stdout, "using the first inverse3 found: %s\n",
                first_port.c_str());
    }

    // The `API::IO::SerialStream` object encapsulates the serial COM port
    // that will be used to communicate with a device. The argument given to
    // its constructor should be a COM port (e.g. COM6).
    API::IO::SerialStream stream{first_port.c_str()};

    // Using the `API::IO::SerialStream` object we can initialize the
    // Inverse3 object which will encapsulates all the logic needed to
    // interact with an Inverse3 device.
    API::Devices::Inverse3 device{&stream};

    // To start using the device, we first need to wake it up using the
    // `DeviceWakeup` function. Once awake, the LED colour on the device
    // should change to indicate that it's ready to receive commands. The
    // method returns a struct of type `Inverse3::DeviceInfoResponse` which
    // contains the device's general information.
    auto info = device.DeviceWakeup();
    std::fprintf(stdout,
                 "info: id=%u, model=%u, version={hardware:%u, firmware:%u}\n",
                 info.device_id, info.device_model_number,
                 info.hardware_version, info.firmware_version);

    while (true) {
        // To use the Inverse3 as a pure measurement device, use the following
        // functions Inverse3.GetEndEffectorPosition();
        // for standard Kinematics.
        // For added precision, the kinematics can be resolved on the computer
        // with the following functions Inverse3.GetEndEffectorPosition(false);
        // for on-computer Kinematics When using the on-computer Kinematics, the
        // angles of the base can be adjusted with the following function to
        // match the physical setup: Inverse3.AdjustAngles([angle0, angle1,
        // angle2]);
        Haply::HardwareAPI::Devices::Inverse3::EndEffectorStateResponse
            response = device.GetEndEffectorPosition(false);
        printf("Position: %f %f %f\n", response.position[0],
               response.position[1], response.position[2]);

        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }

    return 0;
}
