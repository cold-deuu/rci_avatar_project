/* -*- mode: c++ -*-
 * Copyright 2022 Haply Robotics Inc. All rights reserved.
 *
 * Lists all the available Inverse3 and Handle devices using the APIs'
 * auto-detection feature.
 */

#include <cstdio>
#include <string>

// The primary include file for the Haply C++ API.
#include "inverse3/HardwareAPI.h"

// Used to reduce verbosity within the examples.
namespace API = Haply::HardwareAPI;

int main(void) {

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
    }

    // Here we use the `Haply::HardwareAPI::DeviceDetection::DetectWiredHandles`
    // static function to list all the handles devices currently connected. The
    // return value is a vector of string representing the serial COM ports that
    // can be used to communicate with the device.
    {
        auto list = API::Devices::DeviceDetection::DetectHandles();
        for (const auto &port : list)
            std::fprintf(stdout, "handle: %s\n", port.c_str());

        if (list.empty())
            fprintf(stderr, "no handles detected\n");
    }

    return 0;
}
