#include <string.h>

#include <chrono>
#include <iostream>
#include <iterator>
#include <string>
#include <thread>

#include "inverse3/HardwareAPI.h"

int main(int argc, char* argv[])
{
    char* portName;

    if (argc < 2)
    {
        std::printf("Usage: %s <port>\n", argv[0]);
    }
    else
    {
#if defined(_WIN32) || defined(_WIN64)
        portName = _strdup(argv[1]);  // argv1;
#endif
#if defined(__linux__)
        portName = strdup(argv[1]);  // argv1;
#endif
    }

    Haply::HardwareAPI::IO::SerialStream serial_stream(portName);

    Haply::HardwareAPI::Devices::Handle handle(&serial_stream);

    while (true)
    {
        Haply::HardwareAPI::Devices::Handle::VersegripStatusResponse data;
        data = handle.GetVersegripStatus();
        std::printf(
            "device_id: %d battery_level: %f quaternion: %f %f %f %f buttons: "
            "%d error_flags: %d\n",
            data.device_id, data.battery_level, data.quaternion[0],
            data.quaternion[1], data.quaternion[2], data.quaternion[3],
            data.buttons, data.error_flag);

    }
}