// copyright 2022 Haply Robotics Inc. All rights reserved.
#pragma once

#include <string>
#include <utility>
#include <vector>

namespace Haply
{
    namespace HardwareAPI
    {
        namespace Devices
        {
            struct SerialDeviceProvider
            {
                enum Type
                {
                    TypeNil = 0,
                    TypeInverse,
                    TypeHandleWired,
                    TypeHandleWireless
                };

                /**
                 * Lists all the serial ports currently available categorized by
                 * the type of devices that could be detected on it.
                 *
                 * Note that not all ports returned is guranteed to contain a
                 * Haply device. Use the DeviceDetection class to get a list of
                 * available and usable serial ports.
                 *
                 * @return Returns a vector of pairs containing the serial port
                 * (e.g. COM7) and the type of device that could be discovered
                 * on it.
                 */
                static std::vector<std::pair<std::string, Type>>
                ListSerialDevices();

                /**
                 * Lists all the serial ports currently available.
                 *
                 * @param portNames an array of strings that will be populated
                 * with the names of the ports found.
                 *
                 * @param portType This is the type of port you're looking for.
                 * For example, if you're looking for a serial port, you would
                 * pass in "USBSER"
                 *
                 * @return the number of ports found.
                 */
                [[deprecated(
                    "Deprecated in favour of ListSerialDevices()")]] static int
                ListSerialDevices(std::string portNames[],
                                  const wchar_t* portType);

                /**
                 * It lists all the available serial ports on the system, and
                 * then prompts the user to select one.
                 *
                 * @param portNames An array of strings that will be populated
                 * with the names of the available ports.
                 *
                 * @return The index of the port name in the array.
                 */
                [[deprecated(
                    "Deprecated in favour of ListSerialDevices()")]] static int
                SelectComPort(std::string portNames[]);
            };
        }  // namespace Devices
    }      // namespace HardwareAPI
}  // namespace Haply
